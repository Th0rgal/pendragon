#ifndef BLE_OTA_H
#define BLE_OTA_H

#include <stddef.h>
#include <stdint.h>

// BLE OTA update session (opcodes 0xC0-0xC4). The session survives BLE
// disconnects so a client can query 0xC4 and resume where it left off; a
// reboot loses it. A fresh OTA_BEGIN supersedes any stale session.
// Handlers return 0 or a BLE_ATT_ERR_* code, matching GATT access callbacks.

int ble_ota_handle_begin(const uint8_t *payload, uint16_t len);
int ble_ota_handle_data(const uint8_t *payload, uint16_t len);
int ble_ota_handle_end(void);
int ble_ota_handle_abort(void);
int ble_ota_handle_status(void);

// Bytes received/expected of the current session (0/0 when idle).
void ble_ota_get_progress(size_t *received, size_t *expected);

#endif // BLE_OTA_H
