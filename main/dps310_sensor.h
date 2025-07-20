#ifndef DPS310_SENSOR_H
#define DPS310_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

// DPS310 Pin Configuration (Adafruit Breakout):
// VIN → 3.3V (ESP32 Right Side Pin 32 - 3V3_1)
// GND → GND (ESP32 Left Side Pin 1 or Right Side Pin 1)
//
// Current SPI wiring:
// SCK (capteur) → GPIO 21 (ESP32 Left Side Pin 21) - SPI Clock
// SDI (capteur) → GPIO 10 (ESP32 Left Side Pin 23) - SPI MOSI (data to sensor)
// SDO (capteur) → GPIO 11 (ESP32 Left Side Pin 22) - SPI MISO (data from sensor)
// CS (capteur) → GPIO 9 (ESP32 Left Side Pin 24) - Chip Select
//
// Note: For I2C mode, SCK becomes SCL and SDI becomes SDA
// I2C Addresses:
// - 0x77 if SDO/ADR is left floating (default)
// - 0x76 if SDO/ADR is connected to GND

// DPS310 I2C addresses
#define DPS310_I2C_ADDR_PRIMARY 0x77   // Default address
#define DPS310_I2C_ADDR_SECONDARY 0x76 // When SDO/ADR is pulled low

// Communication mode selection
typedef enum
{
    DPS310_COMM_I2C,
    DPS310_COMM_SPI
} dps310_comm_mode_t;

// DPS310 sensor data structure
typedef struct
{
    float temperature;  // Temperature in Celsius
    float pressure;     // Pressure in Pa
    float altitude;     // Calculated altitude in meters
    uint32_t timestamp; // Timestamp when data was read
} dps310_data_t;

// DPS310 configuration structure
typedef struct
{
    int sck_pin;  // SPI Clock pin
    int mosi_pin; // SPI MOSI pin (SDI)
    int miso_pin; // SPI MISO pin (SDO)
    int cs_pin;   // Chip Select pin
} dps310_config_t;

// Function prototypes
esp_err_t dps310_init(void);
esp_err_t dps310_read_data(dps310_data_t *data);
void dps310_sensor_task(void *pvParameters);
esp_err_t dps310_deinit(void);

#endif // DPS310_SENSOR_H