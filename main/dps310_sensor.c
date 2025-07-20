#include "dps310_sensor.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

#define ENABLE_DPS310_LOGGING 1 // Set to 0 to disable DPS310 logging

static const char *TAG = "DPS310_SENSOR";

// DPS310 Register addresses
#define DPS310_REG_PSR_B2 0x00
#define DPS310_REG_PSR_B1 0x01
#define DPS310_REG_PSR_B0 0x02
#define DPS310_REG_TMP_B2 0x03
#define DPS310_REG_TMP_B1 0x04
#define DPS310_REG_TMP_B0 0x05
#define DPS310_REG_PRS_CFG 0x06
#define DPS310_REG_TMP_CFG 0x07
#define DPS310_REG_MEAS_CFG 0x08
#define DPS310_REG_CFG_REG 0x09
#define DPS310_REG_INT_STS 0x0A
#define DPS310_REG_FIFO_STS 0x0B
#define DPS310_REG_RESET 0x0C
#define DPS310_REG_ID 0x0D

// DPS310 Commands
#define DPS310_RESET_CMD 0x89
#define DPS310_CHIP_ID 0x10

// SPI Configuration
#define DPS310_SPI_HOST SPI3_HOST      // Use FSPI (SPI3) for ESP32-S3 FSPI pins
#define DPS310_SPI_CLOCK_SPEED 1000000 // 1 MHz

// Pin configuration - Updated to match actual wiring
static dps310_config_t dps310_config = {
    .sck_pin = 21,  // GPIO 21 (SCK - SPI Clock)
    .mosi_pin = 10, // GPIO 10 (SDI - SPI MOSI – data into sensor)
    .miso_pin = 11, // GPIO 11 (SDO - SPI MISO – data from sensor)
    .cs_pin = 9     // GPIO 9 (CS - Chip Select)
};

// SPI device handle
static spi_device_handle_t dps310_spi;

// Calibration coefficients (simplified for minimal implementation)
// Note: Currently unused in this minimal implementation
// static struct
// {
//     int16_t c0, c1, c00, c10, c01, c11, c20, c21, c30;
// } dps310_calib;

// I2C Configuration - Using current wiring
#define I2C_MASTER_SCL_IO 21      // GPIO 21 (SCK pin on breakout = SCL)
#define I2C_MASTER_SDA_IO 10      // GPIO 10 (SDI pin on breakout = SDA)
#define I2C_MASTER_NUM 0          // I2C port number
#define I2C_MASTER_FREQ_HZ 100000 // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// Function to write to DPS310 register
static esp_err_t dps310_write_reg(uint8_t reg, uint8_t data)
{
    spi_transaction_t trans = {
        .cmd = reg & 0x7F, // Clear MSB for write
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {data}};

    esp_err_t ret = spi_device_polling_transmit(dps310_spi, &trans);
#if ENABLE_DPS310_LOGGING
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "SPI Write: reg=0x%02X, data=0x%02X, cmd=0x%02X",
                 reg, data, trans.cmd);
    }
    else
    {
        ESP_LOGE(TAG, "SPI Write failed: reg=0x%02X, data=0x%02X, error=%s",
                 reg, data, esp_err_to_name(ret));
    }
#endif

    return ret;
}

// Function to read from DPS310 register
// Balanced TX/RX lengths are required by ESP-IDF (tx length must be >= rx length).
// We transmit one dummy byte (0x00) while receiving the requested byte.
static esp_err_t dps310_read_reg(uint8_t reg, uint8_t *data)
{
    spi_transaction_t trans = {
        .cmd = reg | 0x80, // Set MSB for read
        .length = 8,       // Transmit 8 bits (dummy)
        .rxlength = 8,     // Receive 8 bits
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_data = {0x00} // Dummy byte to clock data out
    };

    esp_err_t ret = spi_device_polling_transmit(dps310_spi, &trans);
    if (ret == ESP_OK)
    {
        *data = trans.rx_data[0];
#if ENABLE_DPS310_LOGGING
        ESP_LOGI(TAG, "SPI Read: reg=0x%02X, cmd=0x%02X, rx_data=0x%02X",
                 reg, trans.cmd, *data);
#endif
    }
    else
    {
        ESP_LOGE(TAG, "SPI Read failed: reg=0x%02X, error=%s", reg, esp_err_to_name(ret));
    }

    return ret;
}

// Function to read multiple registers
static esp_err_t dps310_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        esp_err_t ret = dps310_read_reg(reg + i, &data[i]);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }
    return ESP_OK;
}

// Test SPI communication with multiple approaches
static esp_err_t dps310_test_spi_communication(void)
{
    ESP_LOGI(TAG, "=== Testing SPI Communication ===");

    // Test 1: Read ID register multiple times
    ESP_LOGI(TAG, "Test 1: Reading ID register multiple times");
    for (int i = 0; i < 3; i++)
    {
        uint8_t id;
        esp_err_t ret = dps310_read_reg(DPS310_REG_ID, &id);
        ESP_LOGI(TAG, "  Attempt %d: ID=0x%02X, result=%s", i + 1, id, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Test 2: Read different registers
    ESP_LOGI(TAG, "Test 2: Reading different registers");
    uint8_t registers[] = {0x00, 0x01, 0x02, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D};
    for (int i = 0; i < sizeof(registers); i++)
    {
        uint8_t value;
        esp_err_t ret = dps310_read_reg(registers[i], &value);
        ESP_LOGI(TAG, "  Reg 0x%02X: 0x%02X (%s)", registers[i], value, esp_err_to_name(ret));
    }

    // Test 3: Test at different clock speeds with mode 3
    ESP_LOGI(TAG, "Test 3: Testing different clock speeds with SPI mode 3");

    // Remove current device
    spi_bus_remove_device(dps310_spi);

    // Test slower speed first
    spi_device_interface_config_t dev_cfg_slow = {
        .clock_speed_hz = 100000, // 100kHz for initial test
        .mode = 3,
        .spics_io_num = dps310_config.cs_pin,
        .queue_size = 1};

    esp_err_t ret = spi_bus_add_device(DPS310_SPI_HOST, &dev_cfg_slow, &dps310_spi);
    if (ret == ESP_OK)
    {
        uint8_t id;
        ret = dps310_read_reg(DPS310_REG_ID, &id);
        ESP_LOGI(TAG, "  SPI Mode 3 (100kHz): ID=0x%02X (%s)", id, esp_err_to_name(ret));
        spi_bus_remove_device(dps310_spi);
    }

    // Restore original configuration with mode 3
    spi_device_interface_config_t dev_cfg_original = {
        .clock_speed_hz = DPS310_SPI_CLOCK_SPEED,
        .mode = 3,
        .spics_io_num = dps310_config.cs_pin,
        .queue_size = 1};

    ret = spi_bus_add_device(DPS310_SPI_HOST, &dev_cfg_original, &dps310_spi);
    ESP_LOGI(TAG, "=== SPI Communication Test Complete ===");

    return ret;
}

// Test I2C communication
static esp_err_t dps310_test_i2c_communication(void)
{
    ESP_LOGI(TAG, "=== Testing I2C Communication ===");

    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");

    // Scan I2C addresses
    ESP_LOGI(TAG, "Scanning I2C bus...");
    int devices_found = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (result == ESP_OK)
        {
            ESP_LOGI(TAG, "  Device found at address 0x%02X", addr);
            devices_found++;

            // Test reading ID register if this is a potential DPS310 address
            if (addr == DPS310_I2C_ADDR_PRIMARY || addr == DPS310_I2C_ADDR_SECONDARY)
            {
                uint8_t id_reg = 0;
                cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
                i2c_master_write_byte(cmd, DPS310_REG_ID, true);
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
                i2c_master_read_byte(cmd, &id_reg, I2C_MASTER_NACK);
                i2c_master_stop(cmd);

                esp_err_t read_result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
                i2c_cmd_link_delete(cmd);

                if (read_result == ESP_OK)
                {
                    ESP_LOGI(TAG, "    ID register: 0x%02X (expected: 0x10)", id_reg);
                    if (id_reg == DPS310_CHIP_ID)
                    {
                        ESP_LOGI(TAG, "    ✓ DPS310 detected!");
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "    Failed to read ID register: %s", esp_err_to_name(read_result));
                }
            }
        }
    }

    ESP_LOGI(TAG, "I2C scan complete. Found %d device(s)", devices_found);

    // Clean up
    i2c_driver_delete(I2C_MASTER_NUM);

    ESP_LOGI(TAG, "=== I2C Communication Test Complete ===");
    return ESP_OK;
}

// Comprehensive wiring diagnostic
static esp_err_t dps310_wiring_diagnostic(void)
{
    ESP_LOGI(TAG, "=== DPS310 Wiring Diagnostic ===");

    // Check all pin configurations
    ESP_LOGI(TAG, "Pin Configuration:");
    ESP_LOGI(TAG, "  SCK (Clock): GPIO %d", dps310_config.sck_pin);
    ESP_LOGI(TAG, "  SDI (MOSI):  GPIO %d", dps310_config.mosi_pin);
    ESP_LOGI(TAG, "  SDO (MISO):  GPIO %d", dps310_config.miso_pin);
    ESP_LOGI(TAG, "  CS (Select): GPIO %d", dps310_config.cs_pin);

    // Test different SPI speeds
    // First test different SPI modes at low speed
    ESP_LOGI(TAG, "Testing different SPI modes at 100kHz...");
    uint8_t test_modes[] = {0, 1, 2, 3};
    for (int m = 0; m < 4; m++)
    {
        spi_bus_remove_device(dps310_spi);
        spi_device_interface_config_t mode_cfg = {
            .clock_speed_hz = 100000,
            .mode = test_modes[m],
            .spics_io_num = dps310_config.cs_pin,
            .queue_size = 1};

        esp_err_t ret = spi_bus_add_device(DPS310_SPI_HOST, &mode_cfg, &dps310_spi);
        if (ret == ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            uint8_t id;
            ret = dps310_read_reg(DPS310_REG_ID, &id);
            ESP_LOGI(TAG, "  Mode %d: ID=0x%02X (%s)", test_modes[m], id, ret == ESP_OK ? "OK" : esp_err_to_name(ret));
        }
    }

    ESP_LOGI(TAG, "Testing different SPI speeds...");

    uint32_t test_speeds[] = {10000, 50000, 100000, 500000, 1000000}; // 10kHz to 1MHz
    int num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);

    for (int i = 0; i < num_speeds; i++)
    {
        ESP_LOGI(TAG, "Testing at %lu Hz...", test_speeds[i]);

        // Remove current device
        spi_bus_remove_device(dps310_spi);

        // Create new device with test speed
        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = test_speeds[i],
            .mode = 3,
            .spics_io_num = dps310_config.cs_pin,
            .queue_size = 1};

        esp_err_t ret = spi_bus_add_device(DPS310_SPI_HOST, &dev_cfg, &dps310_spi);
        if (ret == ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(50)); // Give time to settle

            uint8_t id;
            ret = dps310_read_reg(DPS310_REG_ID, &id);
            ESP_LOGI(TAG, "  Speed %lu Hz: ID=0x%02X (%s)",
                     test_speeds[i], id, ret == ESP_OK ? "OK" : esp_err_to_name(ret));

            // If we got a reasonable value, test a few more registers
            if (ret == ESP_OK && id != 0x00 && id != 0xFF)
            {
                uint8_t reg_vals[4];
                dps310_read_reg(0x06, &reg_vals[0]); // PRS_CFG
                dps310_read_reg(0x07, &reg_vals[1]); // TMP_CFG
                dps310_read_reg(0x08, &reg_vals[2]); // MEAS_CFG
                dps310_read_reg(0x09, &reg_vals[3]); // CFG_REG

                ESP_LOGI(TAG, "    Additional regs: 0x06=0x%02X, 0x07=0x%02X, 0x08=0x%02X, 0x09=0x%02X",
                         reg_vals[0], reg_vals[1], reg_vals[2], reg_vals[3]);
            }
        }
        else
        {
            ESP_LOGE(TAG, "  Failed to add device at %lu Hz: %s", test_speeds[i], esp_err_to_name(ret));
        }
    }

    // Restore original speed
    spi_bus_remove_device(dps310_spi);
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = DPS310_SPI_CLOCK_SPEED,
        .mode = 3,
        .spics_io_num = dps310_config.cs_pin,
        .queue_size = 1};
    spi_bus_add_device(DPS310_SPI_HOST, &dev_cfg, &dps310_spi);

    ESP_LOGI(TAG, "=== Wiring Diagnostic Complete ===");

    // Additional MISO line test
    ESP_LOGI(TAG, "=== MISO Line Test ===");
    ESP_LOGI(TAG, "Testing if MISO (SDO) line is connected...");

    // Test with CS deasserted (high) - sensor should not respond
    ESP_LOGI(TAG, "Testing with CS high (sensor not selected):");
    gpio_set_level(dps310_config.cs_pin, 1); // CS high
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t test_val_cs_high;
    esp_err_t ret = dps310_read_reg(DPS310_REG_ID, &test_val_cs_high);
    ESP_LOGI(TAG, "  CS=HIGH: ID=0x%02X (%s)", test_val_cs_high, ret == ESP_OK ? "OK" : esp_err_to_name(ret));

    // Test with CS asserted (low) - sensor should respond
    ESP_LOGI(TAG, "Testing with CS low (sensor selected):");
    gpio_set_level(dps310_config.cs_pin, 0); // CS low
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t test_val_cs_low;
    ret = dps310_read_reg(DPS310_REG_ID, &test_val_cs_low);
    ESP_LOGI(TAG, "  CS=LOW:  ID=0x%02X (%s)", test_val_cs_low, ret == ESP_OK ? "OK" : esp_err_to_name(ret));

    // Restore CS to high (idle state)
    gpio_set_level(dps310_config.cs_pin, 1);

    // Analysis
    if (test_val_cs_high == test_val_cs_low && test_val_cs_high == 0xFF)
    {
        ESP_LOGE(TAG, "❌ MISO line appears to be floating high or disconnected");
        ESP_LOGE(TAG, "   Check SDO (MISO) connection: DPS310 SDO → ESP32 GPIO %d", dps310_config.miso_pin);
    }
    else if (test_val_cs_high == test_val_cs_low && test_val_cs_high == 0x00)
    {
        ESP_LOGE(TAG, "❌ MISO line appears to be stuck low or short to ground");
        ESP_LOGE(TAG, "   Check SDO (MISO) connection: DPS310 SDO → ESP32 GPIO %d", dps310_config.miso_pin);
    }
    else if (test_val_cs_high == test_val_cs_low)
    {
        ESP_LOGW(TAG, "⚠️  CS pin may not be connected or sensor not responding");
        ESP_LOGW(TAG, "   Check CS connection: DPS310 CS → ESP32 GPIO %d", dps310_config.cs_pin);
        ESP_LOGW(TAG, "   Also check power: DPS310 VIN → ESP32 3.3V, DPS310 GND → ESP32 GND");
    }
    else
    {
        ESP_LOGI(TAG, "✅ MISO and CS lines appear to be connected");
        ESP_LOGI(TAG, "   Issue may be with sensor communication protocol or power");
    }

    ESP_LOGI(TAG, "=== MISO Line Test Complete ===");
    return ESP_OK;
}

// Initialize DPS310 sensor
esp_err_t dps310_init(void)
{
#if ENABLE_DPS310_LOGGING
    ESP_LOGI(TAG, "Initializing DPS310 sensor...");
#endif

    // Test I2C communication FIRST (before SPI setup to avoid pin conflicts)
    ESP_LOGI(TAG, "Testing I2C communication first (using current wiring)...");
    ESP_LOGI(TAG, "I2C pins: SCL=GPIO%d, SDA=GPIO%d", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);

    esp_err_t i2c_result = dps310_test_i2c_communication();
    if (i2c_result != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C test failed: %s", esp_err_to_name(i2c_result));
    }

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .miso_io_num = dps310_config.miso_pin,
        .mosi_io_num = dps310_config.mosi_pin,
        .sclk_io_num = dps310_config.sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32};

    esp_err_t ret = spi_bus_initialize(DPS310_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = DPS310_SPI_CLOCK_SPEED,
        .mode = 3, // SPI mode 3 (CPOL=1, CPHA=1) - recommended for DPS310
        .spics_io_num = dps310_config.cs_pin,
        .queue_size = 1};

    ret = spi_bus_add_device(DPS310_SPI_HOST, &dev_cfg, &dps310_spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for sensor to be ready after SPI initialization
    vTaskDelay(pdMS_TO_TICKS(200));

    // Try to wake up the sensor by toggling CS pin
    ESP_LOGI(TAG, "Attempting to wake up sensor...");
    gpio_set_level(dps310_config.cs_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(dps310_config.cs_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Run comprehensive SPI communication test
    ESP_LOGI(TAG, "Running SPI communication diagnostics...");
    dps310_test_spi_communication();

    // Run comprehensive wiring diagnostic
    ESP_LOGI(TAG, "Running comprehensive wiring diagnostic...");
    dps310_wiring_diagnostic();

    // Read chip ID to verify communication
    uint8_t chip_id;
    ret = dps310_read_reg(DPS310_REG_ID, &chip_id);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }

#if ENABLE_DPS310_LOGGING
    ESP_LOGI(TAG, "Chip ID: 0x%02X (expected: 0x%02X)", chip_id, DPS310_CHIP_ID);
#endif

    // Check for common SPI communication issues
    if (chip_id == 0x00 || chip_id == 0xFF)
    {
        ESP_LOGE(TAG, "Sensor appears disconnected - got invalid chip ID: 0x%02X", chip_id);
        ESP_LOGE(TAG, "Please check wiring:");
        ESP_LOGE(TAG, "  VIN → 3.3V (ESP32 Right Side Pin 32)");
        ESP_LOGE(TAG, "  GND → GND (ESP32 Left/Right Side Pin 1)");
        ESP_LOGE(TAG, "  SCK → GPIO %d (ESP32 Left Side Pin 21)", dps310_config.sck_pin);
        ESP_LOGE(TAG, "  SDI → GPIO %d (ESP32 Left Side Pin 23)", dps310_config.mosi_pin);
        ESP_LOGE(TAG, "  SDO → GPIO %d (ESP32 Left Side Pin 22)", dps310_config.miso_pin);
        ESP_LOGE(TAG, "  CS  → GPIO %d (ESP32 Left Side Pin 24)", dps310_config.cs_pin);
        return ESP_ERR_NOT_FOUND;
    }

    if (chip_id != DPS310_CHIP_ID)
    {
        ESP_LOGW(TAG, "Unexpected chip ID: 0x%02X (expected: 0x%02X)", chip_id, DPS310_CHIP_ID);
        ESP_LOGW(TAG, "Continuing anyway, but sensor may not work correctly...");
    }

    // Reset the sensor with multiple attempts
    ESP_LOGI(TAG, "Attempting to reset sensor...");
    for (int reset_attempt = 0; reset_attempt < 3; reset_attempt++)
    {
        ret = dps310_write_reg(DPS310_REG_RESET, DPS310_RESET_CMD);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Reset command sent successfully (attempt %d)", reset_attempt + 1);
            break;
        }
        else
        {
            ESP_LOGW(TAG, "Reset attempt %d failed: %s", reset_attempt + 1, esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Wait longer for sensor to complete reset
    ESP_LOGI(TAG, "Waiting for sensor reset to complete...");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Configure pressure measurement (1x oversampling, 1 measurement per second)
    ret = dps310_write_reg(DPS310_REG_PRS_CFG, 0x00);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure pressure: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure temperature measurement (1x oversampling, 1 measurement per second)
    ret = dps310_write_reg(DPS310_REG_TMP_CFG, 0x80); // Use external sensor
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure temperature: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure measurement mode (continuous pressure and temperature)
    ret = dps310_write_reg(DPS310_REG_MEAS_CFG, 0x07);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure measurement mode: %s", esp_err_to_name(ret));
        return ret;
    }

#if ENABLE_DPS310_LOGGING
    ESP_LOGI(TAG, "DPS310 sensor initialized successfully");
#endif

    return ESP_OK;
}

// Read sensor data
esp_err_t dps310_read_data(dps310_data_t *data)
{
    if (data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t psr_data[3];
    uint8_t tmp_data[3];

    // Read pressure data
    esp_err_t ret = dps310_read_regs(DPS310_REG_PSR_B2, psr_data, 3);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read pressure data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read temperature data
    ret = dps310_read_regs(DPS310_REG_TMP_B2, tmp_data, 3);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read temperature data: %s", esp_err_to_name(ret));
        return ret;
    }

#if ENABLE_DPS310_LOGGING
    // Log raw data for debugging
    ESP_LOGI(TAG, "Raw pressure data: 0x%02X 0x%02X 0x%02X", psr_data[0], psr_data[1], psr_data[2]);
    ESP_LOGI(TAG, "Raw temperature data: 0x%02X 0x%02X 0x%02X", tmp_data[0], tmp_data[1], tmp_data[2]);
#endif

    // Convert raw data to signed 24-bit values
    int32_t psr_raw = (int32_t)((psr_data[0] << 16) | (psr_data[1] << 8) | psr_data[2]);
    if (psr_raw & 0x800000)
        psr_raw |= 0xFF000000; // Sign extend

    int32_t tmp_raw = (int32_t)((tmp_data[0] << 16) | (tmp_data[1] << 8) | tmp_data[2]);
    if (tmp_raw & 0x800000)
        tmp_raw |= 0xFF000000; // Sign extend

#if ENABLE_DPS310_LOGGING
    ESP_LOGI(TAG, "Raw values - Pressure: %ld, Temperature: %ld", psr_raw, tmp_raw);
#endif

    // Check if we're getting all zeros or all 0xFF (common SPI communication issues)
    if ((psr_raw == 0 && tmp_raw == 0) || (psr_raw == 0xFFFFFF && tmp_raw == 0xFFFFFF))
    {
        ESP_LOGW(TAG, "Sensor appears disconnected - got invalid raw data");
        // Return default values to indicate no sensor
        data->temperature = 0.0f;
        data->pressure = 101325.0f; // Sea level pressure
        data->altitude = 0.0f;
        data->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Simplified conversion (without full calibration for minimal implementation)
    // These are approximate conversions - for production use, implement full calibration
    data->temperature = (float)tmp_raw / 524288.0f;           // Approximate scaling
    data->pressure = (float)psr_raw / 1048576.0f + 101325.0f; // Approximate scaling + sea level pressure

    // Calculate approximate altitude using barometric formula
    data->altitude = 44330.0f * (1.0f - powf(data->pressure / 101325.0f, 0.1903f));

    data->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

#if ENABLE_DPS310_LOGGING
    ESP_LOGI(TAG, "Temperature: %.2f°C, Pressure: %.2f Pa, Altitude: %.2f m",
             data->temperature, data->pressure, data->altitude);
#endif

    return ESP_OK;
}

// DPS310 sensor task
void dps310_sensor_task(void *pvParameters)
{
#if ENABLE_DPS310_LOGGING
    ESP_LOGI(TAG, "DPS310 sensor task started");
#endif

    // Initialize the sensor
    esp_err_t ret = dps310_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize DPS310 sensor: %s", esp_err_to_name(ret));
        if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Sensor not detected - check wiring and connections");
        }
        ESP_LOGE(TAG, "Task will continue but sensor readings will be invalid");
        // Don't exit the task, continue with error handling
    }

    dps310_data_t sensor_data;
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 5;

    while (1)
    {
        // Read sensor data
        ret = dps310_read_data(&sensor_data);
        if (ret != ESP_OK)
        {
            consecutive_errors++;
            ESP_LOGE(TAG, "Failed to read sensor data: %s (error count: %d)",
                     esp_err_to_name(ret), consecutive_errors);

            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS)
            {
                ESP_LOGE(TAG, "Too many consecutive errors - sensor may be disconnected");
                ESP_LOGE(TAG, "Will continue trying but check connections");
                consecutive_errors = 0; // Reset counter to avoid spam
            }
        }
        else
        {
            consecutive_errors = 0; // Reset error counter on successful read
        }

        // Wait before next reading (1 second)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Deinitialize DPS310 sensor
esp_err_t dps310_deinit(void)
{
    if (dps310_spi)
    {
        spi_bus_remove_device(dps310_spi);
        dps310_spi = NULL;
    }

    spi_bus_free(DPS310_SPI_HOST);

#if ENABLE_DPS310_LOGGING
    ESP_LOGI(TAG, "DPS310 sensor deinitialized");
#endif

    return ESP_OK;
}