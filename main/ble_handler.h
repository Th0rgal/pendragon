#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <stdint.h>
#include "esp_err.h"
#include "host/ble_uuid.h"

// BLE command structure
typedef struct
{
    uint8_t throttle; // 0-255
    uint8_t pitch;    // 0-255
    uint8_t roll;     // 0-255
    uint8_t yaw;      // 0-255
    uint8_t mode;     // Flight mode
} ble_command_t;

// Initialize BLE
esp_err_t init_ble(void);

// Start BLE advertising
esp_err_t start_ble_advertising(void);

// Stop BLE advertising
esp_err_t stop_ble_advertising(void);

// Send telemetry data via BLE
esp_err_t send_telemetry(const void *data, size_t len);

#endif // BLE_HANDLER_H