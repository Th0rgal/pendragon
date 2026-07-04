#include "dshot_esc.h"
#include "ble_handler.h"
#include "icm42688p_sensor.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include <stdio.h>

static const char *TAG = "DSHOT_ESC";

#define DSHOT_MOTOR_COUNT 4

// Same signal pins as the LEDC driver (motor_id_t order: TR, BR, TL, BL).
extern const uint8_t MOTOR_PINS[];

// DShot150 timing at a 20MHz RMT resolution (50ns ticks). DShot150 is the
// baseline digital protocol every BLHeli_S ESC supports.
#define DSHOT_RMT_RESOLUTION_HZ 20000000
#define DSHOT_BIT_TICKS 133 // 6.67us bit period
#define DSHOT_T1H_TICKS 100 // 5.00us high for a 1 bit
#define DSHOT_T0H_TICKS 50  // 2.50us high for a 0 bit
// Inter-frame gap (one all-low symbol) sets the loop rate to ~1kHz.
#define DSHOT_GAP_HALF_TICKS 9000
#define DSHOT_FRAME_SYMBOLS 17 // 16 data bits + gap

// BLHeli_S DShot commands (sent repeatedly with the telemetry bit set).
#define DSHOT_CMD_SPIN_DIRECTION_1 7
#define DSHOT_CMD_SPIN_DIRECTION_2 8
#define DSHOT_CMD_SAVE_SETTINGS 12

#define DSHOT_MAX_TEST_THROTTLE 700 // raw 48..2047; ~34% cap for bench safety

#define NVS_NAMESPACE "pendragon"
#define NVS_KEY_MOTOR_MODE "motor_mode"

typedef enum
{
    DSHOT_REQ_DIRECTION,
    DSHOT_REQ_PROBE,
    DSHOT_REQ_RAW_CMD,
} dshot_request_type_t;

typedef struct
{
    dshot_request_type_t type;
    uint8_t mask;      // DSHOT_REQ_DIRECTION: motors to change
    bool reversed;     // DSHOT_REQ_DIRECTION
    uint8_t motor;     // DSHOT_REQ_PROBE / RAW_CMD: single motor index
    uint16_t throttle; // DSHOT_REQ_PROBE: raw dshot value; RAW_CMD: command
} dshot_dir_request_t;

#define PROBE_PULSE_COUNT 4
#define PROBE_BIAS_MS 500
#define PROBE_PULSE_MS 300
#define PROBE_SETTLE_MS 500
#define PROBE_SAMPLE_MS 10 // one FreeRTOS tick at 100Hz

static bool dshot_active = false;
static rmt_channel_handle_t dshot_channels[DSHOT_MOTOR_COUNT] = {0};
static rmt_encoder_handle_t dshot_copy_encoder = NULL;
static SemaphoreHandle_t dshot_write_mutex = NULL;
static QueueHandle_t dshot_dir_queue = NULL;
static uint16_t dshot_current_values[DSHOT_MOTOR_COUNT] = {0};
static bool dshot_telem_flags[DSHOT_MOTOR_COUNT] = {0};
// Motor lines silent until explicitly enabled; a signal-less ESC disarms.
static bool dshot_output_on = false;

static void build_frame_symbols(uint16_t value, bool telemetry,
                                rmt_symbol_word_t *symbols);

// Burst refresher: instead of an infinite RMT loop restarted on every value
// change (restarts glitch the line; ESCs disarm on signal loss and then
// refuse to re-arm on a nonzero value), a 100Hz task queues finite bursts
// back-to-back. Values change seamlessly on the next burst.
#define DSHOT_BURST_FRAMES 9 // 9 x 1.07ms < 10ms tick: queue never backs up
static rmt_symbol_word_t frame_banks[DSHOT_MOTOR_COUNT][2][DSHOT_FRAME_SYMBOLS];
static int frame_bank_idx[DSHOT_MOTOR_COUNT] = {0};
static volatile int bursts_in_flight[DSHOT_MOTOR_COUNT] = {0};

static bool IRAM_ATTR on_burst_done(rmt_channel_handle_t chan,
                                    const rmt_tx_done_event_data_t *edata,
                                    void *user_ctx)
{
    int motor = (int)(intptr_t)user_ctx;
    bursts_in_flight[motor]--;
    return false;
}

static void dshot_refresh_task(void *pvParameters)
{
    while (1)
    {
        if (dshot_output_on)
        {
            xSemaphoreTake(dshot_write_mutex, portMAX_DELAY);
            for (int m = 0; m < DSHOT_MOTOR_COUNT; m++)
            {
                if (bursts_in_flight[m] >= 2)
                {
                    continue; // previous bursts still playing
                }
                int bank = frame_bank_idx[m] ^ 1;
                build_frame_symbols(dshot_current_values[m],
                                    dshot_telem_flags[m], frame_banks[m][bank]);
                rmt_transmit_config_t tx_config = {
                    .loop_count = DSHOT_BURST_FRAMES};
                if (rmt_transmit(dshot_channels[m], dshot_copy_encoder,
                                 frame_banks[m][bank],
                                 sizeof(frame_banks[m][bank]),
                                 &tx_config) == ESP_OK)
                {
                    bursts_in_flight[m]++;
                    frame_bank_idx[m] = bank;
                }
            }
            xSemaphoreGive(dshot_write_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

uint8_t motor_mode_get_boot(void)
{
    nvs_handle_t handle;
    uint8_t mode = MOTOR_MODE_PWM;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK)
    {
        nvs_get_u8(handle, NVS_KEY_MOTOR_MODE, &mode);
        nvs_close(handle);
    }
    return mode > MOTOR_MODE_DSHOT_CONFIG ? MOTOR_MODE_PWM : mode;
}

esp_err_t motor_mode_set_boot(uint8_t mode)
{
    if (mode > MOTOR_MODE_DSHOT_CONFIG)
    {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = nvs_set_u8(handle, NVS_KEY_MOTOR_MODE, mode);
    if (ret == ESP_OK)
    {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

bool dshot_mode_active(void)
{
    return dshot_active;
}

static void build_frame_symbols(uint16_t value, bool telemetry,
                                rmt_symbol_word_t *symbols)
{
    uint16_t packet = (uint16_t)((value << 1) | (telemetry ? 1 : 0));
    uint8_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    uint16_t frame = (uint16_t)((packet << 4) | crc);

    for (int i = 0; i < 16; i++)
    {
        uint16_t high_ticks =
            ((frame >> (15 - i)) & 1) ? DSHOT_T1H_TICKS : DSHOT_T0H_TICKS;
        symbols[i].level0 = 1;
        symbols[i].duration0 = high_ticks;
        symbols[i].level1 = 0;
        symbols[i].duration1 = DSHOT_BIT_TICKS - high_ticks;
    }
    symbols[16].level0 = 0;
    symbols[16].duration0 = DSHOT_GAP_HALF_TICKS;
    symbols[16].level1 = 0;
    symbols[16].duration1 = DSHOT_GAP_HALF_TICKS;
}

// Set a motor's value; the refresher task streams it continuously with no
// line interruption. Value changes take effect within one burst (~10ms).
static esp_err_t dshot_write_motor(int motor, uint16_t value, bool telemetry)
{
    if (motor < 0 || motor >= DSHOT_MOTOR_COUNT || dshot_channels[motor] == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dshot_output_on)
    {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(dshot_write_mutex, portMAX_DELAY);
    dshot_current_values[motor] = value;
    dshot_telem_flags[motor] = telemetry;
    xSemaphoreGive(dshot_write_mutex);
    return ESP_OK;
}

typedef struct
{
    float ax, ay, az;   // mean accel (g)
    float gz_bias;      // mean gyro z (dps)
    int samples;
} imu_window_t;

// Average accel + gyro Z over `duration_ms` at the tick rate.
static imu_window_t imu_window(int duration_ms)
{
    imu_window_t w = {0};
    for (int elapsed = 0; elapsed < duration_ms; elapsed += PROBE_SAMPLE_MS)
    {
        icm42688p_data_t data;
        if (icm42688p_read_data(&data) == ESP_OK)
        {
            w.ax += data.accel_x;
            w.ay += data.accel_y;
            w.az += data.accel_z;
            w.gz_bias += data.gyro_z;
            w.samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(PROBE_SAMPLE_MS));
    }
    if (w.samples > 0)
    {
        w.ax /= w.samples;
        w.ay /= w.samples;
        w.az /= w.samples;
        w.gz_bias /= w.samples;
    }
    return w;
}

// Integrate bias-corrected gyro Z (degrees) while also averaging accel.
static float gyro_z_integral_with_accel(int duration_ms, float bias_dps,
                                        imu_window_t *accel_out)
{
    float integral_deg = 0.0f;
    imu_window_t w = {0};
    for (int elapsed = 0; elapsed < duration_ms; elapsed += PROBE_SAMPLE_MS)
    {
        icm42688p_data_t data;
        if (icm42688p_read_data(&data) == ESP_OK)
        {
            integral_deg += (data.gyro_z - bias_dps) * (PROBE_SAMPLE_MS / 1000.0f);
            w.ax += data.accel_x;
            w.ay += data.accel_y;
            w.az += data.accel_z;
            w.samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(PROBE_SAMPLE_MS));
    }
    if (w.samples > 0)
    {
        w.ax /= w.samples;
        w.ay /= w.samples;
        w.az /= w.samples;
    }
    *accel_out = w;
    return integral_deg;
}

#define PROBE_SPINUP_SKIP_MS 200
#define PROBE_MEASURE_MS 800

static void run_direction_probe(uint8_t motor, uint16_t throttle)
{
    dshot_set_test_throttle(0);
    vTaskDelay(pdMS_TO_TICKS(PROBE_SETTLE_MS));

    imu_window_t baseline = imu_window(PROBE_BIAS_MS);
    if (baseline.samples == 0)
    {
        ble_log_str("DBG", "probe failed: IMU unreadable");
        return;
    }

    float gz_total = 0.0f;
    float dax_total = 0.0f, day_total = 0.0f, daz_total = 0.0f;
    for (int pulse = 0; pulse < PROBE_PULSE_COUNT; pulse++)
    {
        dshot_write_motor(motor, throttle, false);
        vTaskDelay(pdMS_TO_TICKS(PROBE_SPINUP_SKIP_MS)); // let it spin up
        imu_window_t during;
        gz_total += gyro_z_integral_with_accel(PROBE_MEASURE_MS,
                                               baseline.gz_bias, &during);
        dshot_write_motor(motor, 0, false);
        if (during.samples > 0)
        {
            dax_total += during.ax - baseline.ax;
            day_total += during.ay - baseline.ay;
            daz_total += during.az - baseline.az;
        }
        vTaskDelay(pdMS_TO_TICKS(PROBE_SETTLE_MS));
    }

    // dax/day: sustained tilt from thrust unloading the motor's corner
    // (upward thrust tilts the frame; downward thrust is blocked by the
    // ground). gz: reaction/drag torque, chip +Z down so CCW motor => +gz.
    float gz_avg = gz_total / PROBE_PULSE_COUNT;
    char message[180];
    snprintf(message, sizeof(message),
             "probe motor=%u thr=%u gz=%+.2fdeg dax=%+.4f day=%+.4f daz=%+.4f "
             "n=%d verdict=%s",
             motor, throttle, gz_avg,
             dax_total / PROBE_PULSE_COUNT,
             day_total / PROBE_PULSE_COUNT,
             daz_total / PROBE_PULSE_COUNT,
             baseline.samples,
             gz_avg > 0.3f ? "CCW" : (gz_avg < -0.3f ? "CW" : "UNCLEAR"));
    ble_log_str("DBG", message);
}

static void dshot_worker_task(void *pvParameters)
{
    dshot_dir_request_t request;
    while (1)
    {
        if (xQueueReceive(dshot_dir_queue, &request, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        if (request.type == DSHOT_REQ_PROBE)
        {
            run_direction_probe(request.motor, request.throttle);
            continue;
        }

        if (request.type == DSHOT_REQ_RAW_CMD)
        {
            dshot_write_motor(request.motor, request.throttle, true);
            vTaskDelay(pdMS_TO_TICKS(150));
            dshot_write_motor(request.motor, 0, false);
            char message[80];
            snprintf(message, sizeof(message), "raw cmd %u sent to motor %u",
                     request.throttle, request.motor);
            ble_log_str("DBG", message);
            continue;
        }

        for (int motor = 0; motor < DSHOT_MOTOR_COUNT; motor++)
        {
            if (!(request.mask & (1u << motor)))
            {
                continue;
            }

            uint16_t direction_cmd = request.reversed
                                         ? DSHOT_CMD_SPIN_DIRECTION_2
                                         : DSHOT_CMD_SPIN_DIRECTION_1;

            // Motor must be stopped; command and save must each be seen
            // several consecutive times (the ~1kHz loop repeats them).
            dshot_write_motor(motor, 0, false);
            vTaskDelay(pdMS_TO_TICKS(300));
            dshot_write_motor(motor, direction_cmd, true);
            vTaskDelay(pdMS_TO_TICKS(150));
            dshot_write_motor(motor, 0, false);
            vTaskDelay(pdMS_TO_TICKS(60));
            dshot_write_motor(motor, DSHOT_CMD_SAVE_SETTINGS, true);
            vTaskDelay(pdMS_TO_TICKS(150));
            dshot_write_motor(motor, 0, false);
            vTaskDelay(pdMS_TO_TICKS(250));

            char message[80];
            snprintf(message, sizeof(message), "esc motor=%d direction=%s saved",
                     motor, request.reversed ? "reversed" : "normal");
            ble_log_str("DBG", message);
        }
        ble_log_str("DBG", "esc direction sequence complete");
    }
}

esp_err_t dshot_config_start(void)
{
    if (dshot_active)
    {
        return ESP_OK;
    }

    dshot_write_mutex = xSemaphoreCreateMutex();
    dshot_dir_queue = xQueueCreate(4, sizeof(dshot_dir_request_t));
    if (dshot_write_mutex == NULL || dshot_dir_queue == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    rmt_copy_encoder_config_t encoder_config = {};
    esp_err_t ret = rmt_new_copy_encoder(&encoder_config, &dshot_copy_encoder);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "copy encoder init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    for (int motor = 0; motor < DSHOT_MOTOR_COUNT; motor++)
    {
        rmt_tx_channel_config_t channel_config = {
            .gpio_num = MOTOR_PINS[motor],
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = DSHOT_RMT_RESOLUTION_HZ,
            .mem_block_symbols = 48,
            .trans_queue_depth = 2,
        };
        ret = rmt_new_tx_channel(&channel_config, &dshot_channels[motor]);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "motor %d RMT channel on GPIO%d failed: %s",
                     motor, MOTOR_PINS[motor], esp_err_to_name(ret));
            return ret;
        }
        rmt_tx_event_callbacks_t callbacks = {.on_trans_done = on_burst_done};
        rmt_tx_register_event_callbacks(dshot_channels[motor], &callbacks,
                                        (void *)(intptr_t)motor);
    }

    if (xTaskCreate(dshot_refresh_task, "dshot_refresh", 4096, NULL, 7, NULL) != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }

    dshot_active = true;
    // Lines stay silent until dshot_output_set(true): the ESC sees no signal
    // and stays disarmed, so an unattended board cannot spin motors.

    if (xTaskCreate(dshot_worker_task, "dshot_worker", 4096, NULL, 4, NULL) != pdPASS)
    {
        dshot_active = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "DShot config mode active (DShot150 on 4 motors)");
    return ESP_OK;
}

esp_err_t dshot_output_set(bool enabled)
{
    if (!dshot_active)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (enabled)
    {
        xSemaphoreTake(dshot_write_mutex, portMAX_DELAY);
        for (int motor = 0; motor < DSHOT_MOTOR_COUNT; motor++)
        {
            dshot_current_values[motor] = 0;
            dshot_telem_flags[motor] = false;
            rmt_enable(dshot_channels[motor]); // idempotent-ish; err = already on
            bursts_in_flight[motor] = 0;
        }
        dshot_output_on = true;
        xSemaphoreGive(dshot_write_mutex);
        ESP_LOGI(TAG, "DShot output enabled (zero throttle)");
        return ESP_OK;
    }

    dshot_output_on = false;      // refresher stops queueing
    vTaskDelay(pdMS_TO_TICKS(30)); // let in-flight bursts drain
    xSemaphoreTake(dshot_write_mutex, portMAX_DELAY);
    for (int motor = 0; motor < DSHOT_MOTOR_COUNT; motor++)
    {
        rmt_disable(dshot_channels[motor]);
        dshot_current_values[motor] = 0;
        bursts_in_flight[motor] = 0;
    }
    xSemaphoreGive(dshot_write_mutex);
    ESP_LOGI(TAG, "DShot output disabled (lines silent)");
    return ESP_OK;
}

bool dshot_output_enabled(void)
{
    return dshot_output_on;
}

esp_err_t dshot_request_direction(uint8_t mask, bool reversed)
{
    if (!dshot_active || !dshot_output_on)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (mask == 0 || (mask & ~0x0Fu) != 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Refuse while a test throttle is running.
    for (int motor = 0; motor < DSHOT_MOTOR_COUNT; motor++)
    {
        if (dshot_current_values[motor] >= 48)
        {
            return ESP_ERR_INVALID_STATE;
        }
    }

    dshot_dir_request_t request = {
        .type = DSHOT_REQ_DIRECTION, .mask = mask, .reversed = reversed};
    return xQueueSend(dshot_dir_queue, &request, 0) == pdTRUE ? ESP_OK
                                                              : ESP_ERR_NO_MEM;
}

esp_err_t dshot_request_probe(uint8_t motor, uint16_t throttle)
{
    if (!dshot_active || !dshot_output_on)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (motor >= DSHOT_MOTOR_COUNT || throttle < 48)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (throttle > DSHOT_MAX_TEST_THROTTLE)
    {
        throttle = DSHOT_MAX_TEST_THROTTLE;
    }

    dshot_dir_request_t request = {
        .type = DSHOT_REQ_PROBE, .motor = motor, .throttle = throttle};
    return xQueueSend(dshot_dir_queue, &request, 0) == pdTRUE ? ESP_OK
                                                              : ESP_ERR_NO_MEM;
}

esp_err_t dshot_request_raw_command(uint8_t motor, uint8_t command)
{
    if (!dshot_active || !dshot_output_on)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (motor >= DSHOT_MOTOR_COUNT || command < 1 || command > 47)
    {
        return ESP_ERR_INVALID_ARG;
    }
    dshot_dir_request_t request = {
        .type = DSHOT_REQ_RAW_CMD, .motor = motor, .throttle = command};
    return xQueueSend(dshot_dir_queue, &request, 0) == pdTRUE ? ESP_OK
                                                              : ESP_ERR_NO_MEM;
}

void dshot_get_values(uint16_t values[4])
{
    for (int i = 0; i < DSHOT_MOTOR_COUNT; i++)
    {
        values[i] = dshot_current_values[i];
    }
}

esp_err_t dshot_set_motor_throttle(uint8_t motor, uint16_t value)
{
    if (!dshot_active)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (motor >= DSHOT_MOTOR_COUNT || (value != 0 && value < 48))
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (value > DSHOT_MAX_TEST_THROTTLE)
    {
        value = DSHOT_MAX_TEST_THROTTLE;
    }
    return dshot_write_motor(motor, value, false);
}

#define DSHOT_FLIGHT_MAX 1500

esp_err_t dshot_write_flight_outputs(const uint16_t values[4])
{
    if (!dshot_active || !dshot_output_on)
    {
        return ESP_ERR_INVALID_STATE;
    }
    for (int motor = 0; motor < DSHOT_MOTOR_COUNT; motor++)
    {
        uint16_t value = values[motor];
        if (value != 0 && value < 48)
        {
            value = 48;
        }
        if (value > DSHOT_FLIGHT_MAX)
        {
            value = DSHOT_FLIGHT_MAX;
        }
        esp_err_t ret = dshot_write_motor(motor, value, false);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t dshot_set_test_throttle(uint16_t value)
{
    if (!dshot_active)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (value != 0 && value < 48)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (value > DSHOT_MAX_TEST_THROTTLE)
    {
        value = DSHOT_MAX_TEST_THROTTLE;
    }

    esp_err_t ret = ESP_OK;
    for (int motor = 0; motor < DSHOT_MOTOR_COUNT; motor++)
    {
        esp_err_t motor_ret = dshot_write_motor(motor, value, false);
        if (motor_ret != ESP_OK)
        {
            ret = motor_ret;
        }
    }
    return ret;
}

void dshot_get_debug_status(char *buffer, size_t buffer_len)
{
    if (!buffer || buffer_len == 0)
    {
        return;
    }
    snprintf(buffer, buffer_len,
             "dshot mode active output=%s values=[%u,%u,%u,%u] (0=stop, 48-2047=throttle)",
             dshot_output_on ? "on" : "OFF",
             dshot_current_values[0], dshot_current_values[1],
             dshot_current_values[2], dshot_current_values[3]);
}
