#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 9       // SCL pin
#define I2C_MASTER_SDA_IO 8       // SDA pin
#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number for master
#define I2C_MASTER_FREQ_HZ 400000 // I2C clock frequency

#define ICM20602_ADDRESS 0x69 // I2C address of the ICM-20602 (found in logs)
#define WHO_AM_I_REG 0x75     // WHO_AM_I register address
#define PWR_MGMT_1 0x6B       // Power Management 1 register
#define ACCEL_XOUT_H 0x3B     // First register of accelerometer data
#define GYRO_XOUT_H 0x43      // First register of gyroscope data

static const char *TAG = "ICM20602";

// Initialize I2C
void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Write a byte to a register
void i2c_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM20602_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// Read multiple bytes from consecutive registers
void i2c_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM20602_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (ICM20602_ADDRESS << 1) | I2C_MASTER_READ, true);

    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// Initialize the ICM-20602
void icm20602_init()
{
    // Reset the device
    i2c_write_byte(PWR_MGMT_1, 0x80);     // Set bit 7 to reset
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for reset to complete

    // Wake up the device
    i2c_write_byte(PWR_MGMT_1, 0x00); // Clear sleep bit
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void app_main()
{
    // Initialize I2C
    i2c_master_init();

    // Initialize ICM-20602
    icm20602_init();

    // Main loop to read sensor data
    while (1)
    {
        // Read accelerometer data (6 bytes: X, Y, Z with 2 bytes each)
        uint8_t accel_data[6] = {0};
        i2c_read_bytes(ACCEL_XOUT_H, accel_data, 6);

        // Convert the data to 16-bit signed values
        int16_t accel_x = (accel_data[0] << 8) | accel_data[1];
        int16_t accel_y = (accel_data[2] << 8) | accel_data[3];
        int16_t accel_z = (accel_data[4] << 8) | accel_data[5];

        // Read gyroscope data (6 bytes: X, Y, Z with 2 bytes each)
        uint8_t gyro_data[6] = {0};
        i2c_read_bytes(GYRO_XOUT_H, gyro_data, 6);

        // Convert the data to 16-bit signed values
        int16_t gyro_x = (gyro_data[0] << 8) | gyro_data[1];
        int16_t gyro_y = (gyro_data[2] << 8) | gyro_data[3];
        int16_t gyro_z = (gyro_data[4] << 8) | gyro_data[5];

        // Log the sensor data
        ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d",
                 accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

        // Delay before next reading
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}