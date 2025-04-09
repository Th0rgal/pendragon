#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Structure to hold motor commands (kept for backward compatibility)
typedef struct
{
    uint16_t motor1; // Motor 1 speed (0-1000)
    uint16_t motor2; // Motor 2 speed (0-1000)
    uint16_t motor3; // Motor 3 speed (0-1000)
    uint16_t motor4; // Motor 4 speed (0-1000)
} motor_command_t;

// Function prototypes - simplified API for single motor
void init_motors(void);
void set_motor_speed(uint16_t speed);
void set_motor_acceleration(uint8_t ramp_rate);

// ESC control functions
void esc_control_task(void *pvParameters);

#endif // MOTOR_CONTROL_H