#ifndef SENSOR_MONITOR_H
#define SENSOR_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Structure to hold sensor data
typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    // TODO: Add more sensor data as needed (battery voltage, current, etc.)
} sensor_data_t;

// External queue declaration
extern QueueHandle_t sensor_data_queue;

// Function prototypes
void init_sensors(void);
void sensor_monitor_task(void *pvParameters);

#endif // SENSOR_MONITOR_H