#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Structure to hold flight commands
typedef struct
{
    float throttle;  // 0.0 to 1.0
    float roll;      // -1.0 to 1.0
    float pitch;     // -1.0 to 1.0
    float yaw;       // -1.0 to 1.0
    bool arm_status; // true = armed, false = disarmed
    // TODO: Add more command parameters as needed
} flight_command_t;

// External queue declaration
extern QueueHandle_t command_queue;

// Function prototypes
void command_handler_task(void *pvParameters);

#endif // COMMAND_HANDLER_H