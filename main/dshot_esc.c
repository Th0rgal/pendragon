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
// BLHeli_32 / later BLHeli_S aliases (no-op on ESCs that ignore them).
#define DSHOT_CMD_SPIN_DIRECTION_NORMAL 20
#define DSHOT_CMD_SPIN_DIRECTION_REVERSED 21

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
    uint8_t flags;     // DSHOT_REQ_DIRECTION: DSHOT_DIR_*
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
// One copy encoder per channel: RMT encoders keep per-transaction internal
// state and must not be shared across concurrently-transmitting channels
// (sharing corrupts frames when bursts overlap, i.e. constantly at 100Hz).
static rmt_encoder_handle_t dshot_copy_encoders[DSHOT_MOTOR_COUNT] = {0};
static SemaphoreHandle_t dshot_write_mutex = NULL;
static QueueHandle_t dshot_dir_queue = NULL;
static uint16_t dshot_current_values[DSHOT_MOTOR_COUNT] = {0};
static bool dshot_telem_flags[DSHOT_MOTOR_COUNT] = {0};
// Motor lines silent until explicitly enabled; a signal-less ESC disarms.
static bool dshot_output_on = false;
// Zero-throttle prime: after every silent->active transition the ESC re-runs
// its input-protocol scan (~100ms per protocol, DShot150 mid-list) and then
// requires ~300ms of zero throttle to arm. Emit only DShot0 for this window;
// throttle values are stored and take effect when the prime expires.
#define DSHOT_PRIME_MS 2000
// After rmt_disable the ESC has disarmed and ignores a first frame that is
// already a throttle value. Emit DShot-0 this long on a newly live line
// before applying throttle >= 48. Commands 1-47 skip this (they are the
// arm/direction protocol and must go out as-is).
#define DSHOT_REARM_MS 2000
static volatile TickType_t dshot_enable_tick = 0;

static bool dshot_in_prime(void)
{
    return dshot_output_on && (xTaskGetTickCount() - dshot_enable_tick) <
                                  pdMS_TO_TICKS(DSHOT_PRIME_MS);
}

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
// After the zero-throttle prime, DShot-0 on unused channels is mis-read
// as analog/oneshot (camera: several props disc at once on keep-alive).
// Silence unused lines (ESC disarms on signal loss). The motor that is
// about to get throttle is re-armed with DShot-0 first.
static bool dshot_channel_live[DSHOT_MOTOR_COUNT] = {false, false, false, false};
static bool dshot_rearming[DSHOT_MOTOR_COUNT] = {false, false, false, false};
static TickType_t dshot_rearm_start[DSHOT_MOTOR_COUNT] = {0};
static bool dshot_tx_fail_logged[DSHOT_MOTOR_COUNT] = {false, false, false, false};
static bool dshot_en_fail_logged[DSHOT_MOTOR_COUNT] = {false, false, false, false};

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
            bool prime = dshot_in_prime();
            char tx_fail_ble[4][80];
            int tx_fail_n = 0;
            xSemaphoreTake(dshot_write_mutex, portMAX_DELAY);
            for (int m = 0; m < DSHOT_MOTOR_COUNT; m++)
            {
                uint16_t value = dshot_current_values[m];
                bool telem = dshot_telem_flags[m];
                // Do not broadcast DShot-0 on unused lines, even during
                // the global prime: the 4-in-1 treats that as analog and
                // neighbouring props creep. Only a motor that has a
                // command (throttle or special cmd) is driven; if that
                // happens inside the prime window it still gets zeros.
                bool want_signal = (value != 0) || telem;
                if (prime && want_signal)
                {
                    value = 0;
                    telem = false;
                }
                bool silence = !want_signal;
                if (silence)
                {
                    dshot_rearming[m] = false;
                    if (bursts_in_flight[m] >= 1)
                    {
                        continue;
                    }
                    if (dshot_channel_live[m])
                    {
                        rmt_disable(dshot_channels[m]);
                        dshot_channel_live[m] = false;
                        bursts_in_flight[m] = 0;
                    }
                    continue;
                }
                if (!dshot_channel_live[m])
                {
                    esp_err_t en = rmt_enable(dshot_channels[m]);
                    // INVALID_STATE = already enabled; still usable.
                    if (en != ESP_OK && en != ESP_ERR_INVALID_STATE)
                    {
                        ESP_LOGE(TAG, "rmt_enable motor %d GPIO%d: %s",
                                 m, MOTOR_PINS[m], esp_err_to_name(en));
                        if (!dshot_en_fail_logged[m] && tx_fail_n < 4)
                        {
                            dshot_en_fail_logged[m] = true;
                            snprintf(tx_fail_ble[tx_fail_n],
                                     sizeof(tx_fail_ble[tx_fail_n]),
                                     "rmt_enable motor=%d gpio=%d %s",
                                     m, MOTOR_PINS[m], esp_err_to_name(en));
                            tx_fail_n++;
                        }
                        continue;
                    }
                    dshot_channel_live[m] = true;
                    bursts_in_flight[m] = 0;
                    // If a channel was actually off, the ESC disarmed on
                    // signal loss and will ignore a first nonzero throttle.
                    if (value >= 48)
                    {
                        dshot_rearming[m] = true;
                        dshot_rearm_start[m] = xTaskGetTickCount();
                    }
                }
                if (dshot_rearming[m])
                {
                    if ((xTaskGetTickCount() - dshot_rearm_start[m]) <
                        pdMS_TO_TICKS(DSHOT_REARM_MS))
                    {
                        value = 0;
                        telem = false;
                    }
                    else
                    {
                        dshot_rearming[m] = false;
                    }
                }
                if (bursts_in_flight[m] >= 1)
                {
                    continue; // previous burst still playing (queue depth 1)
                }
                int bank = frame_bank_idx[m] ^ 1;
                build_frame_symbols(value, telem, frame_banks[m][bank]);
                rmt_transmit_config_t tx_config = {
                    .loop_count = DSHOT_BURST_FRAMES};
                esp_err_t tx = rmt_transmit(dshot_channels[m],
                                            dshot_copy_encoders[m],
                                            frame_banks[m][bank],
                                            sizeof(frame_banks[m][bank]),
                                            &tx_config);
                if (tx == ESP_OK)
                {
                    bursts_in_flight[m]++;
                    frame_bank_idx[m] = bank;
                }
                else
                {
                    ESP_LOGE(TAG, "rmt_transmit motor %d GPIO%d: %s",
                             m, MOTOR_PINS[m], esp_err_to_name(tx));
                    if (!dshot_tx_fail_logged[m] && tx_fail_n < 4)
                    {
                        dshot_tx_fail_logged[m] = true;
                        snprintf(tx_fail_ble[tx_fail_n],
                                 sizeof(tx_fail_ble[tx_fail_n]),
                                 "rmt_transmit motor=%d gpio=%d %s",
                                 m, MOTOR_PINS[m], esp_err_to_name(tx));
                        tx_fail_n++;
                    }
                }
            }
            xSemaphoreGive(dshot_write_mutex);
            for (int i = 0; i < tx_fail_n; i++)
            {
                ble_log_str("DBG", tx_fail_ble[i]);
            }
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

        // Direction/probe/raw sequences rely on their values actually being
        // emitted: wait out the zero-throttle prime window first.
        while (dshot_in_prime())
        {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (request.type == DSHOT_REQ_PROBE)
        {
            run_direction_probe(request.motor, request.throttle);
            continue;
        }

        if (request.type == DSHOT_REQ_RAW_CMD)
        {
            // BLHeli_S wants ~10 identical command frames, then a live
            // DShot-0 so a follow-up throttle or save does not see signal-loss.
            dshot_write_motor(request.motor, request.throttle, true);
            vTaskDelay(pdMS_TO_TICKS(800));
            dshot_write_motor(request.motor, 0, true);
            vTaskDelay(pdMS_TO_TICKS(400));
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

            // BLHeli_S: cmd 7 = direction 1 (normal/CW), 8 = direction 2
            // (reversed/CCW). Cmd 20/21 are BLHeli_32 aliases; some BLHeli_S
            // builds ignore them, others only accept them.
            uint16_t dir_cmd;
            if (request.flags & DSHOT_DIR_CMD_20_21)
            {
                dir_cmd = request.reversed ? DSHOT_CMD_SPIN_DIRECTION_REVERSED
                                           : DSHOT_CMD_SPIN_DIRECTION_NORMAL;
            }
            else
            {
                dir_cmd = request.reversed ? DSHOT_CMD_SPIN_DIRECTION_2
                                           : DSHOT_CMD_SPIN_DIRECTION_1;
            }
            bool keep_all = (request.flags & DSHOT_DIR_KEEP_ALL) != 0;
            bool do_save = (request.flags & DSHOT_DIR_NO_SAVE) == 0;

            // Isolated by default: unused DShot-0 analog-creeps neighbouring
            // props on this 4-in-1. KEEP_ALL is the 4-in-1 programming
            // window (zeros+telem only, no throttle) for ESCs that ignore
            // special commands unless every input is a live DShot stream.
            if (keep_all)
            {
                for (int m = 0; m < DSHOT_MOTOR_COUNT; m++)
                {
                    dshot_write_motor(m, 0, true);
                }
            }
            else
            {
                dshot_write_motor(motor, 0, true);
            }
            vTaskDelay(pdMS_TO_TICKS(800));
            dshot_write_motor(motor, dir_cmd, true);
            vTaskDelay(pdMS_TO_TICKS(800));
            dshot_write_motor(motor, 0, true);
            vTaskDelay(pdMS_TO_TICKS(80));
            if (do_save)
            {
                dshot_write_motor(motor, DSHOT_CMD_SAVE_SETTINGS, true);
                vTaskDelay(pdMS_TO_TICKS(800));
                dshot_write_motor(motor, 0, true);
                vTaskDelay(pdMS_TO_TICKS(400));
            }
            if (keep_all)
            {
                for (int m = 0; m < DSHOT_MOTOR_COUNT; m++)
                {
                    if (m != motor)
                    {
                        dshot_write_motor(m, 0, false);
                    }
                }
            }
            // Leave the target live at DShot-0+telem so a follow-up pulse
            // does not pay another 2s re-arm.
            dshot_write_motor(motor, 0, true);
            vTaskDelay(pdMS_TO_TICKS(200));

            char message[96];
            snprintf(message, sizeof(message),
                     "esc motor=%d direction=%s cmd=%u save=%u keep_all=%u",
                     motor, request.reversed ? "reversed" : "normal",
                     dir_cmd, do_save ? 1 : 0, keep_all ? 1 : 0);
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

    for (int motor = 0; motor < DSHOT_MOTOR_COUNT; motor++)
    {
        rmt_copy_encoder_config_t encoder_config = {};
        esp_err_t ret = rmt_new_copy_encoder(&encoder_config,
                                             &dshot_copy_encoders[motor]);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "copy encoder %d init failed: %s", motor,
                     esp_err_to_name(ret));
            return ret;
        }
        rmt_tx_channel_config_t channel_config = {
            .gpio_num = MOTOR_PINS[motor],
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = DSHOT_RMT_RESOLUTION_HZ,
            .mem_block_symbols = 48,
            // Depth 1: at most one driver-owned transaction per channel, so
            // rmt_disable() cancels everything and no stale transaction can
            // auto-start on the next rmt_enable(). Max inter-burst idle is
            // ~1.9ms, far below the ESC's ~320ms signal-loss timeout.
            .trans_queue_depth = 1,
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
            // Stay silent until a motor is actually commanded. Enabling
            // all four here was broadcasting DShot-0 and analog-creep.
            dshot_channel_live[motor] = false;
            dshot_rearming[motor] = false;
            dshot_tx_fail_logged[motor] = false;
            dshot_en_fail_logged[motor] = false;
            bursts_in_flight[motor] = 0;
        }
        dshot_enable_tick = xTaskGetTickCount();
        dshot_output_on = true;
        xSemaphoreGive(dshot_write_mutex);
        ESP_LOGI(TAG, "DShot output enabled (zero-throttle prime %dms, rearm %dms)",
                 DSHOT_PRIME_MS, DSHOT_REARM_MS);
        return ESP_OK;
    }

    dshot_output_on = false;      // refresher stops queueing
    vTaskDelay(pdMS_TO_TICKS(30)); // let in-flight bursts drain
    xSemaphoreTake(dshot_write_mutex, portMAX_DELAY);
    for (int motor = 0; motor < DSHOT_MOTOR_COUNT; motor++)
    {
        rmt_disable(dshot_channels[motor]);
        dshot_channel_live[motor] = false;
        dshot_rearming[motor] = false;
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

esp_err_t dshot_request_direction(uint8_t mask, bool reversed, uint8_t flags)
{
    if (!dshot_active || !dshot_output_on)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (mask == 0 || (mask & ~0x0Fu) != 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (flags & ~(DSHOT_DIR_KEEP_ALL | DSHOT_DIR_NO_SAVE | DSHOT_DIR_CMD_20_21))
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
        .type = DSHOT_REQ_DIRECTION,
        .mask = mask,
        .reversed = reversed,
        .flags = flags};
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
