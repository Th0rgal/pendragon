#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensor_monitor.h"
#include "command_handler.h"
#include "motor_control.h"

// Function prototypes
void pid_controller_task(void *pvParameters);

#endif // PID_CONTROLLER_H