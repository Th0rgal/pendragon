#include "sensor_monitor.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include <math.h>

#define I2C_MASTER_SCL_IO 9       // SCL pin
#define I2C_MASTER_SDA_IO 8       // SDA pin
#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number for master
#define I2C_MASTER_FREQ_HZ 400000 // I2C clock frequency

#define ICM20602_ADDRESS 0x69 // I2C address of the ICM-20602
#define WHO_AM_I_REG 0x75     // WHO_AM_I register address
#define PWR_MGMT_1 0x6B       // Power Management 1 register
#define ACCEL_XOUT_H 0x3B     // First register of accelerometer data
#define GYRO_XOUT_H 0x43      // First register of gyroscope data

static const char *TAG = "SENSOR_MONITOR";

// Flag to enable simulation mode (no physical sensors)
#define SENSOR_SIMULATION_MODE 0 // Set to 0 to read actual sensor data
#define ENABLE_SENSOR_LOGGING 0  // Set to 1 to enable sensor logging

// Global variables
static TimerHandle_t sensor_timer = NULL;
static sensor_data_t sensor_data = {0};

// Function prototypes
static void sensor_timer_callback(TimerHandle_t xTimer);
static void read_sensor_data(sensor_data_t *data);
static esp_err_t i2c_write_byte(uint8_t reg_addr, uint8_t data);

// I2C initialization
static esp_err_t i2c_master_init(void)
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
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Read multiple bytes from the sensor
static esp_err_t i2c_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
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
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Initialize the sensor
void init_sensors(void)
{
    i2c_master_init();

    // Reset the device
    i2c_write_byte(PWR_MGMT_1, 0x80);     // Set bit 7 to reset
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for reset to complete

    // Wake up the device
    i2c_write_byte(PWR_MGMT_1, 0x00); // Clear sleep bit
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Sensors initialized");
}

// Read sensor data (either real or simulated)
static void read_sensor_data(sensor_data_t *data)
{
    // Read accelerometer data (6 bytes: X, Y, Z with 2 bytes each)
    uint8_t accel_data[6] = {0};
    if (i2c_read_bytes(ACCEL_XOUT_H, accel_data, 6) == ESP_OK)
    {
        // Convert the data to 16-bit signed values
        data->accel_x = (accel_data[0] << 8) | accel_data[1];
        data->accel_y = (accel_data[2] << 8) | accel_data[3];
        data->accel_z = (accel_data[4] << 8) | accel_data[5];
    }

    // Read gyroscope data (6 bytes: X, Y, Z with 2 bytes each)
    uint8_t gyro_data[6] = {0};
    if (i2c_read_bytes(GYRO_XOUT_H, gyro_data, 6) == ESP_OK)
    {
        // Convert the data to 16-bit signed values
        data->gyro_x = (gyro_data[0] << 8) | gyro_data[1];
        data->gyro_y = (gyro_data[2] << 8) | gyro_data[3];
        data->gyro_z = (gyro_data[4] << 8) | gyro_data[5];
    }
}

// Timer callback function - called every 100ms (10Hz)
static void sensor_timer_callback(TimerHandle_t xTimer)
{
    static uint32_t backoff_count = 0;
    static TickType_t last_success_time = 0;

    // Read sensor data
    read_sensor_data(&sensor_data);

    // Log the sensor data (less frequently to avoid log spam)
#if ENABLE_SENSOR_LOGGING
    static uint32_t log_counter = 0;
    log_counter++;

    if (log_counter % 25 == 0) // Log every 2.5 seconds (25 * 100ms)
    {
        ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d",
                 sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
                 sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
    }
#endif

    // Send data to queue with a timeout to prevent blocking
    // Use a very short timeout (1ms) to avoid blocking the timer callback
    if (xQueueSend(sensor_data_queue, &sensor_data, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        // Success - reset backoff
        backoff_count = 0;
        last_success_time = xTaskGetTickCount();
    }
    else
    {
        // Skip sensor data if queue is full - don't even log warnings unless it's persisting
        backoff_count++;

        // Only log if we haven't successfully sent data in a while (3 seconds)
        if (backoff_count > 30 && (xTaskGetTickCount() - last_success_time) > pdMS_TO_TICKS(3000))
        {
            ESP_LOGW(TAG, "Queue full for 3 seconds - consider reducing sensor frequency");
            // Reset after logging to avoid spam
            last_success_time = xTaskGetTickCount();
            backoff_count = 0;
        }
    }
}

// Define the function somewhere in your file
static esp_err_t i2c_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM20602_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void sensor_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor monitor task started");

    // Get the synchronization semaphore
    SemaphoreHandle_t startup_sync = (SemaphoreHandle_t)pvParameters;

    // Wait for the synchronization signal from main
    if (startup_sync != NULL)
    {
        ESP_LOGI(TAG, "Waiting for startup synchronization...");
        if (xSemaphoreTake(startup_sync, pdMS_TO_TICKS(5000)) != pdTRUE)
        {
            ESP_LOGW(TAG, "Timeout waiting for startup sync - proceeding anyway");
        }
        else
        {
            ESP_LOGI(TAG, "Received startup synchronization");
        }
        // Return the semaphore
        xSemaphoreGive(startup_sync);
    }

    // Create a timer that triggers every 100ms (10Hz) instead of 50ms (20Hz)
    // This further reduces the frequency of timer callbacks to prevent queue overflow
    sensor_timer = xTimerCreate(
        "SensorTimer",        // Timer name
        pdMS_TO_TICKS(100),   // Timer period in ticks (100ms)
        pdTRUE,               // Auto-reload timer
        (void *)0,            // Timer ID
        sensor_timer_callback // Callback function
    );

    if (sensor_timer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sensor timer");
        vTaskDelete(NULL);
        return;
    }

    // Start the timer with a higher priority
    if (xTimerStart(sensor_timer, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start sensor timer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Sensor timer started successfully");

    // Task remains alive to keep the timer running
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Just keep the task alive
    }
}