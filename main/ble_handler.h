#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <stdint.h>
#include "esp_err.h"
#include "host/ble_uuid.h"
#include "ble_protocol.h"

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

// Restart the chip after a short delay (lets pending notifications flush).
void ble_schedule_restart(void);

#endif // BLE_HANDLER_H
