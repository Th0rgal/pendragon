#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Structure to hold motor commands
typedef struct
{
    uint16_t motor1; // Motor 1 speed (0-2000)
    uint16_t motor2; // Motor 2 speed (0-2000)
    uint16_t motor3; // Motor 3 speed (0-2000)
    uint16_t motor4; // Motor 4 speed (0-2000)
} motor_command_t;

// Function prototypes
void init_motor_control(void);
void set_motor_speed(uint8_t motor, uint16_t speed);
void apply_motor_command(motor_command_t *motor_command);

#endif // MOTOR_CONTROL_H