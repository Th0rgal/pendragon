#include "icm42688p_sensor.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define ENABLE_ICM42688P_LOGGING 0     // high-level data prints (disabled)
#define ENABLE_ICM42688P_REG_LOGGING 0 // per-register SPI logs (noisy)
#define LOG_THROTTLE_MS 100
static uint32_t last_log_ms = 0;

static const char *TAG = "ICM42688P";

// SPI configuration (HSPI bus - independent from DPS310)
#define ICM42688P_SPI_HOST SPI2_HOST      // HSPI
#define ICM42688P_SPI_CLOCK_SPEED 8000000 // 1 MHz for reliability

// Pin mapping based on user wiring
static const struct
{
    int sck_pin;  // GPIO 21
    int mosi_pin; // GPIO 36
    int miso_pin; // GPIO 37
    int cs_pin;   // GPIO 9
} icm_cfg = {
    .sck_pin = 21,
    .mosi_pin = 36,
    .miso_pin = 37,
    .cs_pin = 9,
};

// Register definitions (Bank 0)
#define ICM42688P_REG_WHO_AM_I 0x75
#define ICM42688P_WHO_AM_I_VALUE 0x47
#define ICM42688P_REG_USER_BANK_SEL 0x76 // Corrected from 0x4F
#define ICM42688P_BANK0 0x00
#define ICM42688P_BANK1 0x10
#define ICM42688P_REG_PWR_MGMT0 0x4E
#define ICM42688P_PWR_MGMT0_ACC_GYRO_LN 0x0F // Enable accel+gyro low-noise mode
#define ICM42688P_REG_GYRO_CONFIG0 0x4F
#define ICM42688P_REG_ACCEL_CONFIG0 0x50
// Data registers (Bank 0)
#define ICM42688P_REG_TEMP_DATA1 0x1D
#define ICM42688P_REG_TEMP_DATA0 0x1E
#define ICM42688P_REG_ACCEL_DATA_X1 0x1F
#define ICM42688P_REG_ACCEL_DATA_X0 0x20
#define ICM42688P_REG_ACCEL_DATA_Y1 0x21
#define ICM42688P_REG_ACCEL_DATA_Y0 0x22
#define ICM42688P_REG_ACCEL_DATA_Z1 0x23
#define ICM42688P_REG_ACCEL_DATA_Z0 0x24
#define ICM42688P_REG_GYRO_DATA_X1 0x25
#define ICM42688P_REG_GYRO_DATA_X0 0x26
#define ICM42688P_REG_GYRO_DATA_Y1 0x27
#define ICM42688P_REG_GYRO_DATA_Y0 0x28
#define ICM42688P_REG_GYRO_DATA_Z1 0x29
#define ICM42688P_REG_GYRO_DATA_Z0 0x2A
#define ICM42688P_REG_INT_STATUS 0x2F

// Sensitivity (LSB/physical unit) - corrected based on actual readings
#define ACCEL_SENS_ACTUAL 2048.0f // LSB/g (corrected - was 4x too high)
#define GYRO_SENS_2000DPS 16.4f   // LSB/deg/s for ±2000dps range

static spi_device_handle_t icm_spi;

// ===== Low-level SPI helpers =====
static esp_err_t icm42688p_spi_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg & 0x7F, data};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    esp_err_t ret = spi_device_polling_transmit(icm_spi, &t);
#if ENABLE_ICM42688P_REG_LOGGING
    ESP_LOGI(TAG, "SPI W reg 0x%02X = 0x%02X (%s)", reg, data, esp_err_to_name(ret));
#endif
    return ret;
}

static esp_err_t icm42688p_spi_read_reg(uint8_t reg, uint8_t *data)
{
    uint8_t tx_rx[2] = {reg | 0x80, 0x00};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_rx,
        .rx_buffer = tx_rx,
    };
    esp_err_t ret = spi_device_polling_transmit(icm_spi, &t);
    if (ret == ESP_OK)
        *data = tx_rx[1];
#if ENABLE_ICM42688P_REG_LOGGING
    ESP_LOGI(TAG, "SPI R reg 0x%02X -> 0x%02X (%s)", reg, *data, esp_err_to_name(ret));
#endif
    return ret;
}

static esp_err_t icm42688p_spi_read_bytes(uint8_t start_reg, uint8_t *data, size_t len)
{
    uint8_t tx_buf[1 + len];
    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = start_reg | 0x80; // Read command

    spi_transaction_t t = {
        .length = sizeof(tx_buf) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = tx_buf, // reuse buffer
    };

    esp_err_t ret = spi_device_polling_transmit(icm_spi, &t);
    if (ret != ESP_OK)
        return ret;

    // First byte in rx is dummy (address phase)
    memcpy(data, &tx_buf[1], len);
    return ESP_OK;
}

// ===== Driver public API =====
esp_err_t icm42688p_init(void)
{
#if ENABLE_ICM42688P_LOGGING
    ESP_LOGI(TAG, "Initializing ICM-42688-P...");
#endif
    // Initialize SPI bus (may already be done by other drivers)
    spi_bus_config_t bus_cfg = {
        .miso_io_num = icm_cfg.miso_pin,
        .mosi_io_num = icm_cfg.mosi_pin,
        .sclk_io_num = icm_cfg.sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t ret = spi_bus_initialize(ICM42688P_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = ICM42688P_SPI_CLOCK_SPEED,
        .mode = 0, // SPI mode 0 for ICM-42688-P
        .spics_io_num = icm_cfg.cs_pin,
        .queue_size = 1,
    };
    ret = spi_bus_add_device(ICM42688P_SPI_HOST, &dev_cfg, &icm_spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // Check WHO_AM_I
    uint8_t who = 0;
    ret = icm42688p_spi_read_reg(ICM42688P_REG_WHO_AM_I, &who);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }
    if (who != ICM42688P_WHO_AM_I_VALUE)
    {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I 0x%02X (expected 0x%02X)", who, ICM42688P_WHO_AM_I_VALUE);
        return ESP_ERR_INVALID_RESPONSE;
    }
#if ENABLE_ICM42688P_LOGGING
    ESP_LOGI(TAG, "WHO_AM_I verified: 0x%02X", who);
#endif

    // Ensure we are in Bank 0
    icm42688p_spi_write_reg(ICM42688P_REG_USER_BANK_SEL, ICM42688P_BANK0);

    // Exit sleep, enable accel+gyro
    icm42688p_spi_write_reg(ICM42688P_REG_PWR_MGMT0, ICM42688P_PWR_MGMT0_ACC_GYRO_LN);

    // Wait for power-up sequence
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure gyroscope (GYRO_CONFIG0) and accelerometer (ACCEL_CONFIG0) for 1 kHz ODR and full scale
    // Gyro: bits[5:4] GFS_SEL = 3 (±2000 dps), bits[3:0] ODR = 0xE (1047 Hz effective)
    icm42688p_spi_write_reg(ICM42688P_REG_GYRO_CONFIG0, 0x36); // GYRO_CONFIG0: ±2000dps, 1kHz ODR

    // Accel: bits[5:4] AFS_SEL = 1 (±4 g), bits[3:0] ODR = 0x6 (1kHz)
    icm42688p_spi_write_reg(ICM42688P_REG_ACCEL_CONFIG0, 0x16); // ACCEL_CONFIG0: ±4g, 1kHz ODR

    // Verify configuration was written
    uint8_t gyro_cfg, accel_cfg, pwr_mgmt;
    icm42688p_spi_read_reg(ICM42688P_REG_GYRO_CONFIG0, &gyro_cfg);
    icm42688p_spi_read_reg(ICM42688P_REG_ACCEL_CONFIG0, &accel_cfg);
    icm42688p_spi_read_reg(0x4E, &pwr_mgmt);
    ESP_LOGI(TAG, "Config verification: PWR_MGMT0=0x%02X, GYRO_CFG=0x%02X, ACCEL_CFG=0x%02X",
             pwr_mgmt, gyro_cfg, accel_cfg);

    // Reset signal paths so new data starts flowing
    icm42688p_spi_write_reg(0x4B, 0x01); // SIGNAL_PATH_RESET = 1
    vTaskDelay(pdMS_TO_TICKS(2));        // > 1 ms per datasheet

    // Enable data ready interrupt and continuous mode
    icm42688p_spi_write_reg(0x14, 0x03); // INT_CONFIG - pulse mode, active high
    icm42688p_spi_write_reg(0x15, 0x18); // INT_CONFIG1 - async reset off, latch mode
    icm42688p_spi_write_reg(0x16, 0x01); // INT_SOURCE0 - enable UI data ready interrupt

    vTaskDelay(pdMS_TO_TICKS(50));

#if ENABLE_ICM42688P_LOGGING
    ESP_LOGI(TAG, "ICM-42688-P initialization complete");
#endif
    return ESP_OK;
}

esp_err_t icm42688p_read_data(icm42688p_data_t *out)
{
    if (!out)
        return ESP_ERR_INVALID_ARG;

    // Check data ready status
    uint8_t status;
    esp_err_t ret = icm42688p_spi_read_reg(ICM42688P_REG_INT_STATUS, &status);
    if (ret != ESP_OK)
        return ret;

    // Bit 0 = UI_DRDY (data ready)
    if (!(status & 0x01))
    {
        // No new data available - return previous values or error
        ESP_LOGD(TAG, "No new data ready (status=0x%02X)", status);
        // Continue anyway to read current registers
    }

#if ENABLE_ICM42688P_REG_LOGGING
    static uint32_t debug_counter = 0;
    if (++debug_counter % 25 == 0)
    { // Every 25th read (every 500ms at 50Hz)
        ESP_LOGI(TAG, "Status=0x%02X, DataReady=%s", status, (status & 0x01) ? "YES" : "NO");
    }
#endif

    uint8_t hi, lo;

    // Temperature (TEMP_DATA1, TEMP_DATA0)
    ret = icm42688p_spi_read_reg(ICM42688P_REG_TEMP_DATA1, &hi);
    if (ret != ESP_OK)
        return ret;
    ret = icm42688p_spi_read_reg(ICM42688P_REG_TEMP_DATA0, &lo);
    if (ret != ESP_OK)
        return ret;
    int16_t temp_raw = (hi << 8) | lo;

    // Accelerometer X (ACCEL_DATA_X1, ACCEL_DATA_X0)
    ret = icm42688p_spi_read_reg(ICM42688P_REG_ACCEL_DATA_X1, &hi);
    if (ret != ESP_OK)
        return ret;
    ret = icm42688p_spi_read_reg(ICM42688P_REG_ACCEL_DATA_X0, &lo);
    if (ret != ESP_OK)
        return ret;
    int16_t accel_x_raw = (hi << 8) | lo;

    // Accelerometer Y (ACCEL_DATA_Y1, ACCEL_DATA_Y0)
    ret = icm42688p_spi_read_reg(ICM42688P_REG_ACCEL_DATA_Y1, &hi);
    if (ret != ESP_OK)
        return ret;
    ret = icm42688p_spi_read_reg(ICM42688P_REG_ACCEL_DATA_Y0, &lo);
    if (ret != ESP_OK)
        return ret;
    int16_t accel_y_raw = (hi << 8) | lo;

    // Accelerometer Z (ACCEL_DATA_Z1, ACCEL_DATA_Z0)
    ret = icm42688p_spi_read_reg(ICM42688P_REG_ACCEL_DATA_Z1, &hi);
    if (ret != ESP_OK)
        return ret;
    ret = icm42688p_spi_read_reg(ICM42688P_REG_ACCEL_DATA_Z0, &lo);
    if (ret != ESP_OK)
        return ret;
    int16_t accel_z_raw = (hi << 8) | lo;

    // Gyroscope X (GYRO_DATA_X1, GYRO_DATA_X0)
    ret = icm42688p_spi_read_reg(ICM42688P_REG_GYRO_DATA_X1, &hi);
    if (ret != ESP_OK)
        return ret;
    ret = icm42688p_spi_read_reg(ICM42688P_REG_GYRO_DATA_X0, &lo);
    if (ret != ESP_OK)
        return ret;
    int16_t gyro_x_raw = (hi << 8) | lo;

    // Gyroscope Y (GYRO_DATA_Y1, GYRO_DATA_Y0)
    ret = icm42688p_spi_read_reg(ICM42688P_REG_GYRO_DATA_Y1, &hi);
    if (ret != ESP_OK)
        return ret;
    ret = icm42688p_spi_read_reg(ICM42688P_REG_GYRO_DATA_Y0, &lo);
    if (ret != ESP_OK)
        return ret;
    int16_t gyro_y_raw = (hi << 8) | lo;

    // Gyroscope Z (GYRO_DATA_Z1, GYRO_DATA_Z0)
    ret = icm42688p_spi_read_reg(ICM42688P_REG_GYRO_DATA_Z1, &hi);
    if (ret != ESP_OK)
        return ret;
    ret = icm42688p_spi_read_reg(ICM42688P_REG_GYRO_DATA_Z0, &lo);
    if (ret != ESP_OK)
        return ret;
    int16_t gyro_z_raw = (hi << 8) | lo;

    // Convert raw values to physical units
    out->temperature = ((float)temp_raw) / 132.48f + 25.0f;

    // Convert accelerometer data using corrected sensitivity
    out->accel_x = ((float)accel_x_raw) / ACCEL_SENS_ACTUAL;
    out->accel_y = ((float)accel_y_raw) / ACCEL_SENS_ACTUAL;
    out->accel_z = ((float)accel_z_raw) / ACCEL_SENS_ACTUAL;

    // Convert gyroscope data using ±2000dps sensitivity
    out->gyro_x = ((float)gyro_x_raw) / GYRO_SENS_2000DPS;
    out->gyro_y = ((float)gyro_y_raw) / GYRO_SENS_2000DPS;
    out->gyro_z = ((float)gyro_z_raw) / GYRO_SENS_2000DPS;
    out->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

#if ENABLE_ICM42688P_LOGGING
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now_ms - last_log_ms >= LOG_THROTTLE_MS)
    {
        last_log_ms = now_ms;
        ESP_LOGI(TAG, "Accel[g] (%.2f, %.2f, %.2f) Gyro[dps] (%.2f, %.2f, %.2f) Temp %.2f C",
                 out->accel_x, out->accel_y, out->accel_z,
                 out->gyro_x, out->gyro_y, out->gyro_z,
                 out->temperature);
    }
#endif

    return ESP_OK;
}

void icm42688p_sensor_task(void *pvParameters)
{
    esp_err_t ret = icm42688p_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ICM initialization failed: %s", esp_err_to_name(ret));
    }

    icm42688p_data_t data;
    while (1)
    {
        if (icm42688p_read_data(&data) != ESP_OK)
        {
            ESP_LOGW(TAG, "Read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
    }
}

esp_err_t icm42688p_deinit(void)
{
    if (icm_spi)
    {
        spi_bus_remove_device(icm_spi);
        icm_spi = NULL;
    }
    spi_bus_free(ICM42688P_SPI_HOST);
    return ESP_OK;
}