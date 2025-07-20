/*
 * ICM-42688-P Sensor Driver (header)
 * Minimal SPI driver for ESP32 (ESP-IDF)
 *
 * Pinout used (FSPI bus):
 *  - SCLK (AP_SCL/AP_SCLK) → GPIO21
 *  - MOSI (AP_SDA/AP_SDI)  → GPIO36
 *  - MISO (AP_SDO/AP_AD0)  → GPIO37
 *  - CS   (AP_CS)          → GPIO9
 *
 * Wiring power:
 *  - VDDIO → 3.3 V
 *  - VDD   → 3.3 V
 *  - GND   → GND
 */
#ifndef ICM42688P_SENSOR_H
#define ICM42688P_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // Data structure for returning sensor data (scaled to physical units)
    typedef struct
    {
        float accel_x; // Acceleration in g (gravity)
        float accel_y;
        float accel_z;
        float gyro_x; // Angular rate in °/s
        float gyro_y;
        float gyro_z;
        float temperature;  // Temperature in °C (approx.)
        uint32_t timestamp; // Time of reading (ms since boot)
    } icm42688p_data_t;

    // Driver API
    esp_err_t icm42688p_init(void);
    esp_err_t icm42688p_read_data(icm42688p_data_t *data);
    void icm42688p_sensor_task(void *pvParameters);
    esp_err_t icm42688p_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // ICM42688P_SENSOR_H