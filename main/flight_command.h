#ifndef FLIGHT_COMMAND_H
#define FLIGHT_COMMAND_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Flight command consumed by the ESC control loop. Currently only fed over
// BLE (collective power adjustments); the queue is the insertion point for
// the future stabilization/flight controller.
typedef struct
{
    float throttle;  // 0.0 to 1.0
    float roll;      // -1.0 to 1.0
    float pitch;     // -1.0 to 1.0
    float yaw;       // -1.0 to 1.0
    bool arm_status; // true = armed, false = disarmed
    bool stabilize;  // true = enable simple stabilization mode
} flight_command_t;

// Owned by main.c, consumed by esc_control_task.
extern QueueHandle_t command_queue;

#endif // FLIGHT_COMMAND_H
