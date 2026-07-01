#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <stdint.h>
#include "esp_err.h"
#include "host/ble_uuid.h"

// New: Tiny protocol for power control via BLE writes
// Payload formats supported by the characteristic write handler:
//  - 1 byte: opcode only
//  - 2 bytes: opcode, step_units (0-1000 mapped in motor domain). Typical step: 25..100
// Opcodes below chosen in 0xA0+ range to avoid collision with ASCII
typedef enum
{
    BLE_CMD_POWER_UP = 0xA0,
    BLE_CMD_POWER_DOWN = 0xA1
} ble_cmd_opcode_t;

// Initialize BLE
esp_err_t init_ble(void);

// Start BLE advertising
esp_err_t start_ble_advertising(void);

// Stop BLE advertising
esp_err_t stop_ble_advertising(void);

// Send telemetry data via BLE
esp_err_t send_telemetry(const void *data, size_t len);

// Send a single log line to the app via BLE notification.
// Format on wire: "<TAG>: <MSG>" truncated to current MTU.
esp_err_t ble_log_str(const char *tag, const char *msg);

#endif // BLE_HANDLER_H