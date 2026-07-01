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

static const char *TAG = "BLE_HANDLER";

// BLE connection handle
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

// BLE characteristic handles - to be assigned in app_gatt_register_cb
static uint16_t command_char_handle = 0;
static uint16_t telemetry_char_handle = 0;
static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t *ota_partition = NULL;
static size_t ota_expected_size = 0;
static size_t ota_received_size = 0;
static uint16_t ota_expected_seq = 0;
static bool ota_in_progress = false;

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

static uint32_t read_le_u32(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0]) |
           ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) |
           ((uint32_t)bytes[3] << 24);
}

static uint16_t read_le_u16(const uint8_t *bytes)
{
    return ((uint16_t)bytes[0]) | ((uint16_t)bytes[1] << 8);
}

static void send_info_telemetry(const char *prefix)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);

    char message[180];
    snprintf(message, sizeof(message),
             "%s name=%s version=%s running=%s next=%s heap=%lu ota=%u/%u",
             prefix,
             app ? app->project_name : "unknown",
             app ? app->version : "unknown",
             running ? running->label : "none",
             next ? next->label : "none",
             esp_get_free_heap_size(),
             (unsigned)ota_received_size,
             (unsigned)ota_expected_size);
    ble_log_str("DBG", message);
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

static void reset_ota_state(void)
{
    ota_handle = 0;
    ota_partition = NULL;
    ota_expected_size = 0;
    ota_received_size = 0;
    ota_expected_seq = 0;
    ota_in_progress = false;
}

static int handle_ota_begin(const uint8_t *payload, uint16_t len)
{
    if (len != PENDRAGON_BLE_OTA_BEGIN_LEN)
    {
        ble_log_str("OTA", "begin rejected: expected 5-byte payload");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (ota_in_progress)
    {
        ble_log_str("OTA", "begin rejected: OTA already in progress");
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint32_t firmware_size = read_le_u32(&payload[1]);
    ota_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_partition == NULL)
    {
        ble_log_str("OTA", "begin failed: no update partition");
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (firmware_size == 0 || firmware_size > ota_partition->size)
    {
        char message[120];
        snprintf(message, sizeof(message), "begin rejected: size=%lu partition=%lu",
                 firmware_size, ota_partition->size);
        ble_log_str("OTA", message);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    motor_adjust_power(-1000);
    esp_err_t ret = esp_ota_begin(ota_partition, firmware_size, &ota_handle);
    if (ret != ESP_OK)
    {
        char message[120];
        snprintf(message, sizeof(message), "begin failed: %s", esp_err_to_name(ret));
        ble_log_str("OTA", message);
        reset_ota_state();
        return BLE_ATT_ERR_UNLIKELY;
    }

    ota_expected_size = firmware_size;
    ota_received_size = 0;
    ota_expected_seq = 0;
    ota_in_progress = true;

    char message[120];
    snprintf(message, sizeof(message), "begin ok partition=%s size=%lu",
             ota_partition->label, firmware_size);
    ble_log_str("OTA", message);
    return 0;
}

static int handle_ota_data(const uint8_t *payload, uint16_t len)
{
    if (!ota_in_progress || ota_handle == 0)
    {
        ble_log_str("OTA", "data rejected: no OTA in progress");
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (len <= PENDRAGON_BLE_OTA_DATA_HEADER_LEN)
    {
        ble_log_str("OTA", "data rejected: empty chunk");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint16_t seq = read_le_u16(&payload[1]);
    if (seq != ota_expected_seq)
    {
        char message[120];
        snprintf(message, sizeof(message), "data rejected: seq=%u expected=%u",
                 seq, ota_expected_seq);
        ble_log_str("OTA", message);
        return BLE_ATT_ERR_UNLIKELY;
    }

    const uint8_t *chunk = &payload[PENDRAGON_BLE_OTA_DATA_HEADER_LEN];
    size_t chunk_len = len - PENDRAGON_BLE_OTA_DATA_HEADER_LEN;
    if (ota_received_size + chunk_len > ota_expected_size)
    {
        ble_log_str("OTA", "data rejected: would exceed image size");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    esp_err_t ret = esp_ota_write(ota_handle, chunk, chunk_len);
    if (ret != ESP_OK)
    {
        char message[120];
        snprintf(message, sizeof(message), "write failed: %s", esp_err_to_name(ret));
        ble_log_str("OTA", message);
        return BLE_ATT_ERR_UNLIKELY;
    }

    ota_received_size += chunk_len;
    ota_expected_seq++;

    if ((ota_expected_seq % 32) == 0 || ota_received_size == ota_expected_size)
    {
        char message[120];
        snprintf(message, sizeof(message), "progress %u/%u seq=%u",
                 (unsigned)ota_received_size, (unsigned)ota_expected_size,
                 ota_expected_seq);
        ble_log_str("OTA", message);
    }
    return 0;
}

static int handle_ota_end(void)
{
    if (!ota_in_progress || ota_handle == 0)
    {
        ble_log_str("OTA", "end rejected: no OTA in progress");
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (ota_received_size != ota_expected_size)
    {
        char message[120];
        snprintf(message, sizeof(message), "end rejected: received=%u expected=%u",
                 (unsigned)ota_received_size, (unsigned)ota_expected_size);
        ble_log_str("OTA", message);
        return BLE_ATT_ERR_UNLIKELY;
    }

    esp_err_t ret = esp_ota_end(ota_handle);
    if (ret != ESP_OK)
    {
        char message[120];
        snprintf(message, sizeof(message), "end failed: %s", esp_err_to_name(ret));
        ble_log_str("OTA", message);
        reset_ota_state();
        return BLE_ATT_ERR_UNLIKELY;
    }

    ret = esp_ota_set_boot_partition(ota_partition);
    if (ret != ESP_OK)
    {
        char message[120];
        snprintf(message, sizeof(message), "set boot failed: %s", esp_err_to_name(ret));
        ble_log_str("OTA", message);
        reset_ota_state();
        return BLE_ATT_ERR_UNLIKELY;
    }

    ble_log_str("OTA", "complete; rebooting into new firmware");
    reset_ota_state();
    xTaskCreate(delayed_restart_task, "ble_ota_restart", 2048, NULL, 5, NULL);
    return 0;
}

static int handle_ota_abort(void)
{
    if (ota_in_progress && ota_handle != 0)
    {
        esp_ota_abort(ota_handle);
    }
    reset_ota_state();
    ble_log_str("OTA", "aborted");
    return 0;
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
        uint16_t step = PENDRAGON_BLE_DEFAULT_POWER_STEP;
        if (len >= 2)
        {
            step = payload[1] == 0 ? PENDRAGON_BLE_DEFAULT_POWER_STEP : payload[1];
        }
        ESP_LOGI(TAG, "BLE: %s step=%u",
                 opcode == PENDRAGON_BLE_CMD_POWER_UP ? "POWER_UP" : "POWER_DOWN",
                 step);
        motor_adjust_power(opcode == PENDRAGON_BLE_CMD_POWER_UP ? step : -(int16_t)step);
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
    case PENDRAGON_BLE_CMD_OTA_BEGIN:
        return handle_ota_begin(payload, len);
    case PENDRAGON_BLE_CMD_OTA_DATA:
        return handle_ota_data(payload, len);
    case PENDRAGON_BLE_CMD_OTA_END:
        return handle_ota_end();
    case PENDRAGON_BLE_CMD_OTA_ABORT:
        return handle_ota_abort();
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
        conn_handle = BLE_HS_CONN_HANDLE_NONE;

        // Failsafe: reduce collective to 0 quickly
        motor_adjust_power(-1000);

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
