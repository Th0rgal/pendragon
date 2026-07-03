#ifndef DSHOT_ESC_H
#define DSHOT_ESC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Boot-time motor output mode, persisted in NVS.
#define MOTOR_MODE_PWM 0          // Normal flight firmware: 50Hz LEDC PWM
#define MOTOR_MODE_DSHOT_CONFIG 1 // ESC configuration mode: DShot150 output

uint8_t motor_mode_get_boot(void);
esp_err_t motor_mode_set_boot(uint8_t mode);

// Whether this boot is running in DShot ESC-configuration mode.
bool dshot_mode_active(void);

// Start DShot output (zero throttle) on all motor pins and the worker task.
// Only call when the LEDC motor driver has NOT been initialized.
esp_err_t dshot_config_start(void);

// Queue a spin-direction change + save for the motors in `mask` (bit per
// motor, motor_id_t order). The worker acks over BLE telemetry when done.
esp_err_t dshot_request_direction(uint8_t mask, bool reversed);

// Drive all motors at a raw DShot throttle (0 = stop, 48..2047 = spin).
esp_err_t dshot_set_test_throttle(uint16_t value);

// Drive a single motor at a raw DShot throttle (0 = stop, 48..cap).
esp_err_t dshot_set_motor_throttle(uint8_t motor, uint16_t value);

// Queue a direction probe: pulse `motor` at `throttle` several times while
// integrating gyro Z. Results are reported over BLE telemetry.
esp_err_t dshot_request_probe(uint8_t motor, uint16_t throttle);

// Queue a raw DShot command (1-47) burst to one motor (~150ms repeated).
esp_err_t dshot_request_raw_command(uint8_t motor, uint8_t command);

// Current raw DShot output values, motor_id_t order.
void dshot_get_values(uint16_t values[4]);

void dshot_get_debug_status(char *buffer, size_t buffer_len);

#endif // DSHOT_ESC_H
