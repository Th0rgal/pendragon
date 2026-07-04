#include "flight_ctrl.h"

#include <math.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_handler.h"
#include "dshot_esc.h"
#include "evlog.h"
#include "icm42688p_sensor.h"
#include "motor_control.h" // motor_get_trims

static const char *TAG = "FLIGHT_CTRL";

#define LOOP_PERIOD_MS 10 // 100Hz (FreeRTOS tick)
#define COMP_FILTER_ALPHA 0.98f

// Motor order: TR=0, BR=1, TL=2, BL=3.
// Roll mix: positive roll = right side down -> boost right motors (TR, BR).
static const float ROLL_MIX[4] = {+1.0f, +1.0f, -1.0f, -1.0f};
// Pitch mix: positive pitch = nose up -> boost back motors (BR, BL).
static const float PITCH_MIX[4] = {-1.0f, +1.0f, -1.0f, +1.0f};
// Yaw mix: +gz = frame yawing CW(above); boosting the CW-spinning motors
// (BR, TL) adds CCW reaction torque to counter it.
static const float YAW_MIX[4] = {-1.0f, +1.0f, +1.0f, -1.0f};

#define FLIGHT_MAX_COLLECTIVE 1200 // raw dshot (~59%) - "a bit more strength"
#define FLIGHT_MAX_MOTOR 1400
#define FLIGHT_MIN_SPIN 100        // keep motors turning while armed
#define CORRECTION_CLAMP 220.0f
#define COLLECTIVE_SLEW_PER_TICK 6 // 600 units/s - gentle on the weak buck
#define ABORT_ANGLE_DEG 25.0f
#define ABORT_RATE_DPS 400.0f
#define ARM_STILLNESS_DPS 6.0f
#define BIAS_CAPTURE_MS 700

static bool flight_armed = false;
static volatile uint16_t collective_target = 0;
static float collective_current = 0.0f;
static float kp_angle = 3.0f;  // dshot units per degree
static float kd_rate = 0.6f;   // dshot units per deg/s
static float ki_angle = 4.0f;  // dshot units per degree-second
static float kd_yaw = 0.5f;    // dshot units per deg/s of yaw rate
#define INTEGRAL_CLAMP 70.0f
static float roll_i = 0.0f, pitch_i = 0.0f;

static float roll_deg = 0.0f, pitch_deg = 0.0f;
static float gx_bias = 0.0f, gy_bias = 0.0f;
static float ax_ref = 0.0f, ay_ref = 0.0f; // level reference at arm
static uint16_t last_outputs[4] = {0, 0, 0, 0};

static void flight_cut(const char *reason)
{
    collective_target = 0;
    collective_current = 0.0f;
    flight_armed = false;
    roll_i = 0.0f;
    pitch_i = 0.0f;
    dshot_set_test_throttle(0);
    for (int i = 0; i < 4; i++)
    {
        last_outputs[i] = 0;
    }
    evlog("flight disarm: %s", reason);
    ble_log_str("FLT", reason);
}

// Capture gyro bias + accel level reference; requires stillness.
static bool capture_bias(void)
{
    float sgx = 0, sgy = 0, sax = 0, say = 0;
    float max_rate = 0;
    int n = 0;
    for (int elapsed = 0; elapsed < BIAS_CAPTURE_MS; elapsed += LOOP_PERIOD_MS)
    {
        icm42688p_data_t d;
        if (icm42688p_read_data(&d) == ESP_OK)
        {
            sgx += d.gyro_x;
            sgy += d.gyro_y;
            sax += d.accel_x;
            say += d.accel_y;
            float rate = fabsf(d.gyro_x) + fabsf(d.gyro_y);
            if (rate > max_rate)
            {
                max_rate = rate;
            }
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
    if (n < 20 || max_rate > ARM_STILLNESS_DPS * 2)
    {
        return false;
    }
    gx_bias = sgx / n;
    gy_bias = sgy / n;
    ax_ref = sax / n;
    ay_ref = say / n;
    roll_deg = 0.0f;
    pitch_deg = 0.0f;
    roll_i = 0.0f;
    pitch_i = 0.0f;
    return true;
}

static void flight_task(void *pvParameters)
{
    ESP_LOGI(TAG, "flight control task started (100Hz)");
    const float dt = LOOP_PERIOD_MS / 1000.0f;

    while (1)
    {
        if (!flight_armed)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        icm42688p_data_t d;
        if (icm42688p_read_data(&d) != ESP_OK)
        {
            flight_cut("imu read failed");
            continue;
        }

        float gx = d.gyro_x - gx_bias; // + = roll right-down rate
        float gy = d.gyro_y - gy_bias; // + = nose-up rate

        // Accel tilt relative to the captured level reference (small angle).
        // Nose down 21deg measured dax=-0.36g => pitch = asin(dax/g) nose-up+.
        float dax = d.accel_x - ax_ref;
        float day = d.accel_y - ay_ref;
        if (dax > 0.7f) dax = 0.7f;
        if (dax < -0.7f) dax = -0.7f;
        if (day > 0.7f) day = 0.7f;
        if (day < -0.7f) day = -0.7f;
        float pitch_acc = asinf(dax) * 57.2958f;
        float roll_acc = asinf(-day) * 57.2958f;

        pitch_deg = COMP_FILTER_ALPHA * (pitch_deg + gy * dt) +
                    (1.0f - COMP_FILTER_ALPHA) * pitch_acc;
        roll_deg = COMP_FILTER_ALPHA * (roll_deg + gx * dt) +
                   (1.0f - COMP_FILTER_ALPHA) * roll_acc;

        if (fabsf(roll_deg) > ABORT_ANGLE_DEG || fabsf(pitch_deg) > ABORT_ANGLE_DEG)
        {
            flight_cut("attitude abort");
            continue;
        }
        if (fabsf(gx) > ABORT_RATE_DPS || fabsf(gy) > ABORT_RATE_DPS)
        {
            flight_cut("rate abort");
            continue;
        }

        // Slew-limited collective (gentle current ramps for the weak buck).
        float target = (float)collective_target;
        if (collective_current < target)
        {
            collective_current += COLLECTIVE_SLEW_PER_TICK;
            if (collective_current > target) collective_current = target;
        }
        else if (collective_current > target)
        {
            collective_current -= 4.0f * COLLECTIVE_SLEW_PER_TICK; // faster down
            if (collective_current < target) collective_current = target;
        }

        // Integrators only while producing real thrust (anti-windup).
        if (collective_current > 300.0f)
        {
            roll_i += ki_angle * roll_deg * dt;
            pitch_i += ki_angle * pitch_deg * dt;
            if (roll_i > INTEGRAL_CLAMP) roll_i = INTEGRAL_CLAMP;
            if (roll_i < -INTEGRAL_CLAMP) roll_i = -INTEGRAL_CLAMP;
            if (pitch_i > INTEGRAL_CLAMP) pitch_i = INTEGRAL_CLAMP;
            if (pitch_i < -INTEGRAL_CLAMP) pitch_i = -INTEGRAL_CLAMP;
        }

        // Angle-PI + rate-D corrections (positive = tilt that needs fixing).
        float roll_corr = kp_angle * roll_deg + kd_rate * gx + roll_i;
        float pitch_corr = kp_angle * pitch_deg + kd_rate * gy + pitch_i;
        float yaw_corr = kd_yaw * (d.gyro_z); // rate damping only
        if (roll_corr > CORRECTION_CLAMP) roll_corr = CORRECTION_CLAMP;
        if (roll_corr < -CORRECTION_CLAMP) roll_corr = -CORRECTION_CLAMP;
        if (pitch_corr > CORRECTION_CLAMP) pitch_corr = CORRECTION_CLAMP;
        if (pitch_corr < -CORRECTION_CLAMP) pitch_corr = -CORRECTION_CLAMP;
        if (yaw_corr > 120.0f) yaw_corr = 120.0f;
        if (yaw_corr < -120.0f) yaw_corr = -120.0f;

        uint8_t trims[4];
        motor_get_trims(trims);

        // Fully stopped when no collective is commanded - no idle twitch.
        if (collective_current < 1.0f && collective_target == 0)
        {
            uint16_t zeros[4] = {0, 0, 0, 0};
            dshot_write_flight_outputs(zeros);
            for (int m = 0; m < 4; m++)
            {
                last_outputs[m] = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
            continue;
        }

        uint16_t outputs[4];
        for (int m = 0; m < 4; m++)
        {
            float base = collective_current * trims[m] / 100.0f;
            // Roll right-down (+) => right side must RISE => boost right
            // motors (ROLL_MIX right=+1). Pitch nose-up (+) => back is low,
            // back must RISE => boost back motors (PITCH_MIX back=+1).
            float out = base + roll_corr * ROLL_MIX[m] +
                        pitch_corr * PITCH_MIX[m] + yaw_corr * YAW_MIX[m];
            if (collective_current >= FLIGHT_MIN_SPIN && out < FLIGHT_MIN_SPIN)
            {
                out = FLIGHT_MIN_SPIN;
            }
            if (out < 0) out = 0;
            if (out > FLIGHT_MAX_MOTOR) out = FLIGHT_MAX_MOTOR;
            outputs[m] = (uint16_t)out;
        }

        if (dshot_write_flight_outputs(outputs) != ESP_OK)
        {
            flight_cut("output write failed");
            continue;
        }
        for (int m = 0; m < 4; m++)
        {
            last_outputs[m] = outputs[m];
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

esp_err_t flight_ctrl_start(void)
{
    static bool started = false;
    if (started)
    {
        return ESP_OK;
    }
    if (xTaskCreate(flight_task, "flight_ctrl", 6144, NULL, 6, NULL) != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    started = true;
    return ESP_OK;
}

esp_err_t flight_ctrl_arm(bool armed)
{
    if (!armed)
    {
        flight_cut("disarm command");
        return ESP_OK;
    }
    if (flight_armed)
    {
        return ESP_OK;
    }
    if (!dshot_mode_active())
    {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = dshot_output_set(true);
    if (ret != ESP_OK)
    {
        return ret;
    }
    if (!capture_bias())
    {
        ble_log_str("FLT", "arm refused: not still enough for bias capture");
        return ESP_ERR_INVALID_STATE;
    }
    collective_target = 0;
    collective_current = 0.0f;
    flight_armed = true;
    evlog("flight ARMED kp=%.1f kd=%.2f", kp_angle, kd_rate);
    ble_log_str("FLT", "ARMED (collective 0)");
    return ESP_OK;
}

bool flight_ctrl_armed(void)
{
    return flight_armed;
}

esp_err_t flight_ctrl_set_collective(uint16_t target)
{
    if (!flight_armed)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (target > FLIGHT_MAX_COLLECTIVE)
    {
        target = FLIGHT_MAX_COLLECTIVE;
    }
    collective_target = target;
    evlog("flight collective=%u", target);
    return ESP_OK;
}

void flight_ctrl_set_gains(uint8_t kp_x10, uint8_t kd_x100)
{
    if (kp_x10 > 0)
    {
        kp_angle = kp_x10 / 10.0f;
    }
    if (kd_x100 > 0)
    {
        kd_rate = kd_x100 / 100.0f;
    }
    evlog("flight gains kp=%.1f kd=%.2f", kp_angle, kd_rate);
}

void flight_ctrl_get_attitude(float *roll, float *pitch)
{
    if (roll) *roll = roll_deg;
    if (pitch) *pitch = pitch_deg;
}

void flight_ctrl_get_debug(char *buffer, int buffer_len)
{
    snprintf(buffer, buffer_len,
             "flight %s coll=%u->%.0f att=%+.1f,%+.1f out=[%u,%u,%u,%u] kp=%.1f kd=%.2f",
             flight_armed ? "ARMED" : "disarmed",
             collective_target, collective_current,
             roll_deg, pitch_deg,
             last_outputs[0], last_outputs[1], last_outputs[2], last_outputs[3],
             kp_angle, kd_rate);
}
