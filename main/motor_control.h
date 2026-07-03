#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Motor identifiers
typedef enum
{
    MOTOR_TOP_RIGHT = 0,    // GPIO 5   (ESC wire 1: White)
    MOTOR_BOTTOM_RIGHT = 1, // GPIO 13  (ESC wire 2: Brown)
    MOTOR_TOP_LEFT = 2,     // GPIO 18  (ESC wire 3: Orange)
    MOTOR_BOTTOM_LEFT = 3,  // GPIO 17  (ESC wire 4: Yellow)
    MOTOR_ALL = 255         // Special value to control all motors
} motor_id_t;

// Structure to hold motor commands (kept for backward compatibility)
typedef struct
{
    uint16_t motor1; // Motor 1 speed (0-1000)
    uint16_t motor2; // Motor 2 speed (0-1000)
    uint16_t motor3; // Motor 3 speed (0-1000)
    uint16_t motor4; // Motor 4 speed (0-1000)
} motor_command_t;

// Function prototypes - updated for multiple motors
void init_motors(void);
void set_motor_speed(motor_id_t motor, uint16_t speed);
void set_motor_acceleration(motor_id_t motor, uint8_t ramp_rate);

// Stabilization control
void set_stabilization_enabled(bool enabled);

// ESC control functions
void esc_control_task(void *pvParameters);

// New: power adjust entry points used by BLE opcodes
void motor_adjust_power(int16_t delta_step_0_to_1000);
void motor_get_debug_status(char *buffer, size_t buffer_len);

// Per-motor thrust trim (percent, 50-150, 100 = neutral), NVS-persisted.
// In motor_set_trims, 0 or 0xFF entries leave that motor unchanged.
#include "esp_err.h"
void motor_load_trims(void);
esp_err_t motor_set_trims(const uint8_t trims_pct[4]);
void motor_get_trims(uint8_t trims_pct[4]);

// Current per-motor PWM output speeds (0-1000), motor_id_t order.
void motor_get_speeds(uint16_t speeds[4]);

#endif // MOTOR_CONTROL_H
