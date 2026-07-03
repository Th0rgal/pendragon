#include "ble_ota.h"

#include <stdio.h>
#include "esp_ota_ops.h"
#include "host/ble_hs.h"

#include "ble_handler.h"
#include "ble_protocol.h"
#include "dshot_esc.h"
#include "evlog.h"
#include "motor_control.h"

static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t *ota_partition = NULL;
static size_t ota_expected_size = 0;
static size_t ota_received_size = 0;
static uint16_t ota_expected_seq = 0;
static bool ota_in_progress = false;

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

static void reset_ota_state(void)
{
    ota_handle = 0;
    ota_partition = NULL;
    ota_expected_size = 0;
    ota_received_size = 0;
    ota_expected_seq = 0;
    ota_in_progress = false;
}

void ble_ota_get_progress(size_t *received, size_t *expected)
{
    if (received)
    {
        *received = ota_received_size;
    }
    if (expected)
    {
        *expected = ota_expected_size;
    }
}

int ble_ota_handle_begin(const uint8_t *payload, uint16_t len)
{
    if (len != PENDRAGON_BLE_OTA_BEGIN_LEN)
    {
        ble_log_str("OTA", "begin rejected: expected 5-byte payload");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (ota_in_progress)
    {
        // A fresh begin supersedes any stale/interrupted session.
        if (ota_handle != 0)
        {
            esp_ota_abort(ota_handle);
        }
        reset_ota_state();
        evlog("ota begin superseded stale session");
        ble_log_str("OTA", "stale session aborted; starting fresh");
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

    // Motors off before any flash-heavy work.
    if (dshot_mode_active())
    {
        dshot_set_test_throttle(0);
    }
    else
    {
        motor_adjust_power(-1000);
    }

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
    evlog("ota begin size=%lu", firmware_size);

    char message[120];
    snprintf(message, sizeof(message), "begin ok partition=%s size=%lu",
             ota_partition->label, firmware_size);
    ble_log_str("OTA", message);
    return 0;
}

int ble_ota_handle_data(const uint8_t *payload, uint16_t len)
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

int ble_ota_handle_end(void)
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

    evlog("ota complete, rebooting");
    ble_log_str("OTA", "complete; rebooting into new firmware");
    reset_ota_state();
    ble_schedule_restart();
    return 0;
}

int ble_ota_handle_abort(void)
{
    if (ota_in_progress && ota_handle != 0)
    {
        esp_ota_abort(ota_handle);
    }
    reset_ota_state();
    ble_log_str("OTA", "aborted");
    return 0;
}

int ble_ota_handle_status(void)
{
    char message[120];
    snprintf(message, sizeof(message),
             "status in_progress=%d received=%u expected=%u seq=%u",
             ota_in_progress ? 1 : 0,
             (unsigned)ota_received_size,
             (unsigned)ota_expected_size,
             ota_expected_seq);
    ble_log_str("OTA", message);
    return 0;
}
