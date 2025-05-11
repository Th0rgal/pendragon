#include <string.h>
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
#include "ble_handler.h"
#include "command_handler.h"

static const char *TAG = "BLE_HANDLER";

// BLE connection handle
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

// BLE characteristic handle - to be assigned in app_gatt_register_cb
static uint16_t drone_char_handle = 0;

// BLE Address information
static uint8_t own_addr_type;
// static uint8_t addr_val[6] = {0}; // Not used for now, but kept for future scan response

// Forward declarations
static void app_advertise(void);
static int app_gap_event_handler(struct ble_gap_event *event, void *arg);
static void app_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
static void ble_on_reset(int reason); // Renamed from ble_hs_reset_cb for consistency
static void app_on_sync(void);

// BLE service UUIDs (128-bit)
static const ble_uuid128_t DRONE_SERVICE_UUID128 = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
              0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}};

static const ble_uuid128_t DRONE_CHARACTERISTIC_UUID128 = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
              0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00}};

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
        // Handle write request
        if (ctxt->om->om_len == sizeof(ble_command_t))
        {
            ble_command_t cmd;
            memcpy(&cmd, ctxt->om->om_data, sizeof(ble_command_t));
            ESP_LOGI(TAG, "BLE Command Received: Throttle=%d, Pitch=%d, Roll=%d, Yaw=%d, Mode=%d",
                     cmd.throttle, cmd.pitch, cmd.roll, cmd.yaw, cmd.mode);
            // TODO: Add command processing logic (simplified for now)
        }
        else
        {
            ESP_LOGW(TAG, "GATT Write: Received data of len %d, expected %d for ble_command_t",
                     ctxt->om->om_len, sizeof(ble_command_t));
        }
        return 0;

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
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &drone_char_handle, // Assign characteristic value handle pointer
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
        // drone_char_handle is now assigned directly via .val_handle in gatt_svcs
        // This callback can still be used for logging or other actions if needed.
        // If .val_handle in gatt_svcs isn't working as expected, this is where you'd assign it:
        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &DRONE_CHARACTERISTIC_UUID128.u) == 0)
        {
            drone_char_handle = ctxt->chr.val_handle;
            ESP_LOGI(TAG, "Drone characteristic registered via callback, val_handle=0x%x", drone_char_handle);
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
    rc = ble_svc_gap_device_name_set("Pendragon");
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
    if (drone_char_handle == 0)
    {
        ESP_LOGE(TAG, "Send telemetry: Invalid drone_char_handle");
        return ESP_ERR_INVALID_STATE;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gattc_notify_custom(conn_handle, drone_char_handle, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error sending telemetry notification; rc=%d", rc);
        // os_mbuf_free_chain(om); // ble_gattc_notify_custom frees the mbuf on success or error
        return ESP_FAIL;
    }
    return ESP_OK;
}