#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_handler.h"
#include "motor_control.h"
#include "icm42688p_sensor.h"
#include "dshot_esc.h"
#include "ble_ota.h"
#include "evlog.h"
#include "esp_timer.h"

static const char *TAG = "BLE_HANDLER";

// BLE connection handle
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

// BLE characteristic handles - to be assigned in app_gatt_register_cb
static uint16_t command_char_handle = 0;
static uint16_t telemetry_char_handle = 0;

// BLE Address information
static uint8_t own_addr_type;
// static uint8_t addr_val[6] = {0}; // Not used for now, but kept for future scan response

// Forward declarations
static void app_advertise(void);
static int app_gap_event_handler(struct ble_gap_event *event, void *arg);
static void app_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
static void ble_on_reset(int reason); // Renamed from ble_hs_reset_cb for consistency
static void app_on_sync(void);
static int handle_command_payload(const uint8_t *payload, uint16_t len);
static void delayed_restart_task(void *pvParameters);

// BLE service UUIDs (128-bit)
static const ble_uuid128_t DRONE_SERVICE_UUID128 = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = PENDRAGON_SERVICE_UUID_BYTES};

static const ble_uuid128_t DRONE_CHARACTERISTIC_UUID128 = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = PENDRAGON_COMMAND_CHARACTERISTIC_UUID_BYTES};

static const ble_uuid128_t TELEMETRY_CHARACTERISTIC_UUID128 = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = PENDRAGON_TELEMETRY_CHARACTERISTIC_UUID_BYTES};

// BLE GAP access callback
static int ble_gap_access_cb(uint16_t conn_handle_arg, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        // Handle read request
        return 0;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (ctxt->om->om_len < 1)
        {
            ESP_LOGW(TAG, "GATT Write: empty payload");
            return BLE_ATT_ERR_UNLIKELY;
        }

        uint8_t payload[512];
        if (ctxt->om->om_len > sizeof(payload))
        {
            ESP_LOGW(TAG, "GATT Write too large: %d", ctxt->om->om_len);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        int rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, payload);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "Failed to copy GATT write payload: rc=%d", rc);
            return BLE_ATT_ERR_UNLIKELY;
        }
        return handle_command_payload(payload, ctxt->om->om_len);

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// BLE service definition
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &DRONE_SERVICE_UUID128.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &DRONE_CHARACTERISTIC_UUID128.u,
                .access_cb = ble_gap_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &command_char_handle,
            },
            {
                .uuid = &TELEMETRY_CHARACTERISTIC_UUID128.u,
                .access_cb = ble_gap_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &telemetry_char_handle,
            },
            {
                0, // No more characteristics
            },
        },
    },
    {
        0, // No more services
    },
};

static uint16_t read_le_u16(const uint8_t *bytes)
{
    return ((uint16_t)bytes[0]) | ((uint16_t)bytes[1] << 8);
}

static void send_info_telemetry(const char *prefix)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);

    size_t ota_received = 0, ota_expected = 0;
    ble_ota_get_progress(&ota_received, &ota_expected);

    char message[180];
    snprintf(message, sizeof(message),
             "%s name=%s version=%s running=%s next=%s heap=%lu ota=%u/%u "
             "reset=%d up=%llus mode=%s",
             prefix,
             app ? app->project_name : "unknown",
             app ? app->version : "unknown",
             running ? running->label : "none",
             next ? next->label : "none",
             esp_get_free_heap_size(),
             (unsigned)ota_received,
             (unsigned)ota_expected,
             esp_reset_reason(),
             (unsigned long long)(esp_timer_get_time() / 1000000),
             dshot_mode_active() ? "dshot" : "pwm");
    ble_log_str("DBG", message);
}

// Dump the event log as a burst of telemetry notifications (own task: the
// GATT callback must not block on the notify pacing delays).
static void evlog_dump_task(void *pvParameters)
{
    // Static: 2.4KB is too large for the task stack (single-instance task).
    static evlog_entry_t snapshot[EVLOG_CAPACITY];
    uint32_t count = evlog_snapshot(snapshot, EVLOG_CAPACITY);
    char line[EVLOG_LINE + 24];
    for (uint32_t i = 0; i < count; i++)
    {
        snprintf(line, sizeof(line), "[%lu.%03lus] %s",
                 snapshot[i].ms / 1000, snapshot[i].ms % 1000, snapshot[i].text);
        ble_log_str("EVT", line);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    snprintf(line, sizeof(line), "end of event log (%lu entries)", count);
    ble_log_str("EVT", line);
    vTaskDelete(NULL);
}

// Continuous telemetry stream: IMU + motor outputs at a fixed rate.
static volatile bool stream_active = false;
static volatile bool stream_stop_requested = false;

static void telemetry_stream_task(void *pvParameters)
{
    uint32_t packed = (uint32_t)(uintptr_t)pvParameters;
    uint32_t rate_hz = packed >> 8;
    uint32_t seconds = packed & 0xFF;
    int64_t end_us = esp_timer_get_time() + (int64_t)seconds * 1000000;
    uint32_t period_ms = 1000 / rate_hz;

    while (!stream_stop_requested && esp_timer_get_time() < end_us &&
           conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        icm42688p_data_t imu = {0};
        bool imu_ok = icm42688p_read_data(&imu) == ESP_OK;
        uint16_t motors[4];
        if (dshot_mode_active())
        {
            dshot_get_values(motors);
        }
        else
        {
            motor_get_speeds(motors);
        }

        char line[160];
        snprintf(line, sizeof(line),
                 "st a=%+.3f,%+.3f,%+.3f g=%+.1f,%+.1f,%+.1f m=[%u,%u,%u,%u]%s",
                 imu.accel_x, imu.accel_y, imu.accel_z,
                 imu.gyro_x, imu.gyro_y, imu.gyro_z,
                 motors[0], motors[1], motors[2], motors[3],
                 imu_ok ? "" : " imu_err");
        ble_log_str("STR", line);
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
    ble_log_str("STR", "stream end");
    stream_active = false;
    vTaskDelete(NULL);
}

static void send_sensor_telemetry(void)
{
    icm42688p_data_t data = {0};
    esp_err_t ret = icm42688p_read_data(&data);
    if (ret != ESP_OK)
    {
        char message[80];
        snprintf(message, sizeof(message), "sensor read failed: %s", esp_err_to_name(ret));
        ble_log_str("DBG", message);
        return;
    }

    char message[180];
    snprintf(message, sizeof(message),
             "sensor ax=%.3f ay=%.3f az=%.3f gx=%.1f gy=%.1f gz=%.1f temp=%.1fC t=%lu",
             data.accel_x, data.accel_y, data.accel_z,
             data.gyro_x, data.gyro_y, data.gyro_z,
             data.temperature, data.timestamp);
    ble_log_str("DBG", message);
}

static void send_motor_telemetry(void)
{
    char message[240];
    if (dshot_mode_active())
    {
        dshot_get_debug_status(message, sizeof(message));
    }
    else
    {
        motor_get_debug_status(message, sizeof(message));
    }
    ble_log_str("DBG", message);
}

static int handle_command_payload(const uint8_t *payload, uint16_t len)
{
    uint8_t opcode = payload[0];
    if (opcode < PENDRAGON_BLE_OPCODE_MIN)
    {
        ESP_LOGW(TAG, "BLE: Unsupported opcode 0x%02X", opcode);
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    switch ((pendragon_ble_opcode_t)opcode)
    {
    case PENDRAGON_BLE_CMD_POWER_UP:
    case PENDRAGON_BLE_CMD_POWER_DOWN:
    {
        if (dshot_mode_active())
        {
            ble_log_str("DBG", "power cmd ignored: dshot config mode (use 0xD2)");
            return 0;
        }
        uint16_t step = PENDRAGON_BLE_DEFAULT_POWER_STEP;
        if (len >= 2)
        {
            step = payload[1] == 0 ? PENDRAGON_BLE_DEFAULT_POWER_STEP : payload[1];
        }
        ESP_LOGI(TAG, "BLE: %s step=%u",
                 opcode == PENDRAGON_BLE_CMD_POWER_UP ? "POWER_UP" : "POWER_DOWN",
                 step);
        evlog("cmd pwr %s%u", opcode == PENDRAGON_BLE_CMD_POWER_UP ? "+" : "-", step);
        motor_adjust_power(opcode == PENDRAGON_BLE_CMD_POWER_UP ? step : -(int16_t)step);
        return 0;
    }
    case PENDRAGON_BLE_CMD_MOTOR_MODE:
    {
        if (len < 2 || payload[1] > MOTOR_MODE_DSHOT_CONFIG)
        {
            ble_log_str("DBG", "motor mode rejected: expected [0xD0, 0|1]");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (motor_mode_set_boot(payload[1]) != ESP_OK)
        {
            ble_log_str("DBG", "motor mode NVS write failed");
            return BLE_ATT_ERR_UNLIKELY;
        }
        evlog("cmd mode=%u reboot", payload[1]);
        char message[120];
        snprintf(message, sizeof(message),
                 "motor mode=%s saved; rebooting - power-cycle battery so ESC re-detects protocol",
                 payload[1] == MOTOR_MODE_DSHOT_CONFIG ? "dshot-config" : "pwm");
        ble_log_str("DBG", message);
        ble_schedule_restart();
        return 0;
    }
    case PENDRAGON_BLE_CMD_ESC_DIRECTION:
    {
        if (len < 3)
        {
            ble_log_str("DBG", "esc direction rejected: expected [0xD1, mask, dir]");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        evlog("cmd dir mask=%02x rev=%u", payload[1], payload[2]);
        esp_err_t ret = dshot_request_direction(payload[1], payload[2] != 0);
        if (ret != ESP_OK)
        {
            char message[100];
            snprintf(message, sizeof(message), "esc direction rejected: %s",
                     esp_err_to_name(ret));
            ble_log_str("DBG", message);
            return BLE_ATT_ERR_UNLIKELY;
        }
        ble_log_str("DBG", "esc direction sequence queued");
        return 0;
    }
    case PENDRAGON_BLE_CMD_ESC_DIRECTION_PROBE:
    {
        if (len < 4)
        {
            ble_log_str("DBG", "probe rejected: expected [0xD3, motor, thr_lo, thr_hi]");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        evlog("cmd probe m=%u thr=%u", payload[1], read_le_u16(&payload[2]));
        esp_err_t ret = dshot_request_probe(payload[1], read_le_u16(&payload[2]));
        if (ret != ESP_OK)
        {
            char message[100];
            snprintf(message, sizeof(message), "probe rejected: %s",
                     esp_err_to_name(ret));
            ble_log_str("DBG", message);
            return BLE_ATT_ERR_UNLIKELY;
        }
        ble_log_str("DBG", "probe queued");
        return 0;
    }
    case PENDRAGON_BLE_CMD_MOTOR_TRIM:
    {
        char message[100];
        if (len >= 5)
        {
            esp_err_t ret = motor_set_trims(&payload[1]);
            if (ret != ESP_OK)
            {
                snprintf(message, sizeof(message), "trim rejected: %s (valid 50-150)",
                         esp_err_to_name(ret));
                ble_log_str("DBG", message);
                return BLE_ATT_ERR_UNLIKELY;
            }
        }
        uint8_t trims[4];
        motor_get_trims(trims);
        snprintf(message, sizeof(message), "trim tr=%u br=%u tl=%u bl=%u",
                 trims[0], trims[1], trims[2], trims[3]);
        ble_log_str("DBG", message);
        return 0;
    }
    case PENDRAGON_BLE_CMD_ESC_RAW_CMD:
    {
        if (len < 3)
        {
            ble_log_str("DBG", "raw cmd rejected: expected [0xD5, motor, cmd]");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        evlog("cmd raw m=%u c=%u", payload[1], payload[2]);
        esp_err_t ret = dshot_request_raw_command(payload[1], payload[2]);
        if (ret != ESP_OK)
        {
            char message[100];
            snprintf(message, sizeof(message), "raw cmd rejected: %s",
                     esp_err_to_name(ret));
            ble_log_str("DBG", message);
            return BLE_ATT_ERR_UNLIKELY;
        }
        return 0;
    }
    case PENDRAGON_BLE_CMD_ESC_MOTOR_THROTTLE:
    {
        if (len < 4)
        {
            ble_log_str("DBG", "motor throttle rejected: expected [0xD6, motor, lo, hi]");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint16_t value = read_le_u16(&payload[2]);
        evlog("cmd mthr m=%u thr=%u", payload[1], value);
        esp_err_t ret = dshot_set_motor_throttle(payload[1], value);
        if (ret != ESP_OK)
        {
            char message[100];
            snprintf(message, sizeof(message), "motor throttle rejected: %s",
                     esp_err_to_name(ret));
            ble_log_str("DBG", message);
            return BLE_ATT_ERR_UNLIKELY;
        }
        return 0;
    }
    case PENDRAGON_BLE_CMD_ESC_TEST_THROTTLE:
    {
        if (len < 3)
        {
            ble_log_str("DBG", "esc throttle rejected: expected [0xD2, lo, hi]");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint16_t value = read_le_u16(&payload[1]);
        evlog("cmd dshot thr=%u", value);
        esp_err_t ret = dshot_set_test_throttle(value);
        if (ret != ESP_OK)
        {
            char message[100];
            snprintf(message, sizeof(message), "esc throttle rejected: %s",
                     esp_err_to_name(ret));
            ble_log_str("DBG", message);
            return BLE_ATT_ERR_UNLIKELY;
        }
        char message[80];
        snprintf(message, sizeof(message), "esc test throttle set to %u", value);
        ble_log_str("DBG", message);
        return 0;
    }
    case PENDRAGON_BLE_CMD_PING:
        ble_log_str("DBG", "pong");
        return 0;
    case PENDRAGON_BLE_CMD_INFO:
        send_info_telemetry("info");
        return 0;
    case PENDRAGON_BLE_CMD_SENSOR_SNAPSHOT:
        send_sensor_telemetry();
        return 0;
    case PENDRAGON_BLE_CMD_MOTOR_STATUS:
        send_motor_telemetry();
        return 0;
    case PENDRAGON_BLE_CMD_TELEMETRY_STREAM:
    {
        if (len < 2 || payload[1] == 0)
        {
            stream_stop_requested = true;
            ble_log_str("DBG", "stream stop requested");
            return 0;
        }
        if (stream_active)
        {
            ble_log_str("DBG", "stream already running");
            return 0;
        }
        uint32_t rate_hz = payload[1] > 20 ? 20 : payload[1];
        uint32_t seconds = len >= 3 && payload[2] > 0 ? payload[2] : 10;
        if (seconds > 60)
        {
            seconds = 60;
        }
        stream_stop_requested = false;
        stream_active = true;
        uint32_t packed = (rate_hz << 8) | seconds;
        if (xTaskCreate(telemetry_stream_task, "ble_stream", 6144,
                        (void *)(uintptr_t)packed, 3, NULL) != pdPASS)
        {
            stream_active = false;
            return BLE_ATT_ERR_UNLIKELY;
        }
        evlog("stream start %luHz %lus", rate_hz, seconds);
        return 0;
    }
    case PENDRAGON_BLE_CMD_EVENT_LOG:
        if (xTaskCreate(evlog_dump_task, "evlog_dump", 6144, NULL, 3, NULL) != pdPASS)
        {
            return BLE_ATT_ERR_UNLIKELY;
        }
        return 0;
    case PENDRAGON_BLE_CMD_OTA_BEGIN:
        return ble_ota_handle_begin(payload, len);
    case PENDRAGON_BLE_CMD_OTA_DATA:
        return ble_ota_handle_data(payload, len);
    case PENDRAGON_BLE_CMD_OTA_END:
        return ble_ota_handle_end();
    case PENDRAGON_BLE_CMD_OTA_ABORT:
        return ble_ota_handle_abort();
    case PENDRAGON_BLE_CMD_OTA_STATUS:
        return ble_ota_handle_status();
    default:
        ESP_LOGW(TAG, "BLE: Unknown opcode 0x%02X", opcode);
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }
}

static void delayed_restart_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(700));
    esp_restart();
}

void ble_schedule_restart(void)
{
    xTaskCreate(delayed_restart_task, "delayed_restart", 2048, NULL, 5, NULL);
}

// BLE GAP event callback
static int app_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);

        evlog("ble connect status=%d", event->connect.status);
        if (event->connect.status == 0)
        {
            conn_handle = event->connect.conn_handle;
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc == 0)
            {
                // print_conn_desc(&desc); // Utility from example, can be added later
            }
        }
        else
        {
            // Connection failed, restart advertising
            app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
        evlog("ble disconnect reason=%d, motors zeroed", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;

        // Failsafe: reduce collective to 0 quickly
        if (dshot_mode_active())
        {
            dshot_set_test_throttle(0);
        }
        else
        {
            motor_adjust_power(-1000);
        }

        // OTA session is intentionally kept alive across disconnects so the
        // client can resume (query 0xC4). A new OTA_BEGIN supersedes it.

        // Restart advertising
        app_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete; reason=%d", event->adv_complete.reason);
        // Optionally restart advertising if needed, e.g., if it was time-limited
        // app_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "Connection updated; status=%d", event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc == 0)
        {
            // print_conn_desc(&desc); // Utility from example
        }
        return 0;

        // TODO: Handle other GAP events like repeat pairing, subscribe, mtu, etc. if needed.

    default:
        return 0;
    }
}

// Start advertising
static void app_advertise(void)
{
    int rc;
    struct ble_hs_adv_fields fields;
    const char *name;
    struct ble_gap_adv_params adv_params;

    // Infer address type
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    // rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL); // For scan response later
    // if (rc != 0) {
    //     ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
    //     return;
    // }

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.appearance = 0x0180; // Generic Remote Control (was BLE_GAP_APPEARANCE_GENERIC_REMOTE_CONTROL)
    fields.appearance_is_present = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    // Scan response - not setting for now to simplify
    // struct ble_hs_adv_fields rsp_fields = {0};
    // rsp_fields.device_addr = addr_val;
    // rsp_fields.device_addr_type = own_addr_type;
    // rsp_fields.device_addr_is_present = 1;
    // rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    // if (rc != 0) {
    //    ESP_LOGE(TAG, "error setting scan response data; rc=%d", rc);
    //    return;
    // }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    // adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500); // Example values
    // adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           app_gap_event_handler, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "error starting advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Advertising started");

    // Log the service UUID
    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&DRONE_SERVICE_UUID128.u, uuid_str);
    ESP_LOGI(TAG, "Advertising with Service UUID: %s", uuid_str);
}

// BLE host sync callback
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Reset callback (renamed for consistency)
static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

// GATT server register callback
static void app_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op)
    {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG, "registering characteristic %s with def_handle=0x%x val_handle=0x%x",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        // Characteristic handles are also assigned directly via .val_handle in gatt_svcs.
        // This callback can still be used for logging or other actions if needed.
        // If .val_handle in gatt_svcs isn't working as expected, this is where you'd assign it:
        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &DRONE_CHARACTERISTIC_UUID128.u) == 0)
        {
            command_char_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "Command characteristic registered, val_handle=0x%x", command_char_handle);
        }
        else if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &TELEMETRY_CHARACTERISTIC_UUID128.u) == 0)
        {
            telemetry_char_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "Telemetry characteristic registered, val_handle=0x%x", telemetry_char_handle);
        }
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        // Not used in this example
        break;
    default:
        break;
    }
}

// New: Sync callback
static void app_on_sync(void)
{
    ESP_LOGI(TAG, "Bluetooth host synced.");
    // Ensure address is available and own_addr_type is set before advertising
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to ensure address; rc=%d", rc);
        return;
    }

    // Start advertising
    app_advertise();
}

// Initialize BLE (refactored)
esp_err_t init_ble(void)
{
    esp_err_t ret;
    int rc;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize NimBLE port
    nimble_port_init();

    // Configure the host
    ble_hs_cfg.sync_cb = app_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.gatts_register_cb = app_gatt_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    // Security parameters (can be expanded later)
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT; // Example capability
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    // Initialize NimBLE GAP service
    ble_svc_gap_init();
    rc = ble_svc_gap_device_name_set(PENDRAGON_BLE_DEVICE_NAME);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set device name; rc=%d", rc);
        return ESP_FAIL;
    }
    rc = ble_svc_gap_device_appearance_set(0x0180); // Generic Remote Control (was BLE_GAP_APPEARANCE_GENERIC_REMOTE_CONTROL)
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set device appearance; rc=%d", rc);
        return ESP_FAIL;
    }

    // Initialize NimBLE GATT service
    ble_svc_gatt_init();

    // Register our custom services
    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to count GATT services; rc=%d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to add GATT services; rc=%d", rc);
        return ESP_FAIL;
    }

    // Start the BLE host task
    nimble_port_freertos_init(ble_host_task);

    return ESP_OK;
}

// Start BLE advertising (public function, now calls internal app_advertise logic via sync mechanism)
esp_err_t start_ble_advertising(void)
{
    // Advertising is started by the app_on_sync callback when the host is ready.
    // If we need to manually restart it later (e.g., from another part of the app),
    // we might need a flag or a direct call to app_advertise if the host is already synced.
    // For now, this function can ensure the host is started, or be a NOP if auto-start on sync.
    if (ble_hs_is_enabled())
    {                    // Check if host is enabled
        app_advertise(); // Or rely on sync_cb to call it. This provides an explicit way.
        return ESP_OK;
    }
    ESP_LOGW(TAG, "start_ble_advertising: BLE host not enabled/synced yet.");
    return ESP_ERR_INVALID_STATE;
}

// Stop BLE advertising
esp_err_t stop_ble_advertising(void)
{
    int rc = ble_gap_adv_stop();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error stopping advertisement; rc=%d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Advertising stopped.");
    return ESP_OK;
}

// Send telemetry data via BLE
esp_err_t send_telemetry(const void *data, size_t len)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE)
    {
        ESP_LOGW(TAG, "Send telemetry: No connection");
        return ESP_ERR_NOT_FOUND;
    }
    if (telemetry_char_handle == 0)
    {
        ESP_LOGE(TAG, "Send telemetry: Invalid telemetry_char_handle");
        return ESP_ERR_INVALID_STATE;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gattc_notify_custom(conn_handle, telemetry_char_handle, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error sending telemetry notification; rc=%d", rc);
        // os_mbuf_free_chain(om); // ble_gattc_notify_custom frees the mbuf on success or error
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Convenience: send a single tagged log line to the app
esp_err_t ble_log_str(const char *tag, const char *msg)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || telemetry_char_handle == 0)
    {
        return ESP_ERR_INVALID_STATE;
    }
    char buf[180];
    int n = snprintf(buf, sizeof(buf), "%s: %s", tag ? tag : "LOG", msg ? msg : "");
    if (n < 0)
        return ESP_FAIL;
    if ((size_t)n > sizeof(buf))
        n = sizeof(buf);
    return send_telemetry(buf, (size_t)n);
}
