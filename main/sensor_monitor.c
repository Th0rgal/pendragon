#include "sensor_monitor.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/timers.h"
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
#define SENSOR_SIMULATION_MODE 1
#define ENABLE_SENSOR_LOGGING 1 // Set to 1 to enable sensor logging

// Global variables
static uint32_t time_ms = 0;
static TimerHandle_t sensor_timer = NULL;
static sensor_data_t sensor_data = {0};

// Function prototypes
static void sensor_timer_callback(TimerHandle_t xTimer);
static void read_sensor_data(sensor_data_t *data);
static void generate_simulated_data(sensor_data_t *data, uint32_t time_ms);

// Initialize I2C
static void i2c_master_init(void)
{
#if !SENSOR_SIMULATION_MODE
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
#else
    ESP_LOGI(TAG, "Sensor simulation mode enabled - I2C not initialized");
#endif
}

// Write a byte to a register
static void i2c_write_byte(uint8_t reg_addr, uint8_t data)
{
#if !SENSOR_SIMULATION_MODE
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM20602_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
#endif
}

// Read multiple bytes from consecutive registers
static void i2c_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
#if !SENSOR_SIMULATION_MODE
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
#else
    // In simulation mode, just fill with test data
    for (int i = 0; i < len; i++)
    {
        data[i] = i * 10; // Simple pattern for test data
    }
#endif
}

// Initialize the ICM-20602
static void icm20602_init(void)
{
#if !SENSOR_SIMULATION_MODE
    // Reset the device
    i2c_write_byte(PWR_MGMT_1, 0x80);     // Set bit 7 to reset
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for reset to complete

    // Wake up the device
    i2c_write_byte(PWR_MGMT_1, 0x00); // Clear sleep bit
    vTaskDelay(100 / portTICK_PERIOD_MS);
#else
    ESP_LOGI(TAG, "Sensor simulation mode enabled - ICM20602 not initialized");
#endif
}

void init_sensors(void)
{
    // Initialize I2C
    i2c_master_init();

    // Initialize ICM-20602
    icm20602_init();

    ESP_LOGI(TAG, "Sensors initialized %s",
             SENSOR_SIMULATION_MODE ? "(SIMULATION MODE)" : "");
}

// Generate simulated sensor data
static void generate_simulated_data(sensor_data_t *data, uint32_t time_ms)
{
    // Create some simple oscillating values for testing
    float t = time_ms / 1000.0f; // Time in seconds

    // Simulated accelerometer data
    data->accel_x = (int16_t)(2000 * sin(t * 0.5f));
    data->accel_y = (int16_t)(2000 * cos(t * 0.5f));
    data->accel_z = (int16_t)(16000 + 500 * sin(t * 0.2f)); // ~1g with small oscillation

    // Simulated gyroscope data
    data->gyro_x = (int16_t)(1000 * sin(t * 2.0f));
    data->gyro_y = (int16_t)(1000 * cos(t * 2.0f));
    data->gyro_z = (int16_t)(500 * sin(t * 1.0f));
}

// Read sensor data (either real or simulated)
static void read_sensor_data(sensor_data_t *data)
{
#if !SENSOR_SIMULATION_MODE
    // Read accelerometer data (6 bytes: X, Y, Z with 2 bytes each)
    uint8_t accel_data[6] = {0};
    i2c_read_bytes(ACCEL_XOUT_H, accel_data, 6);

    // Convert the data to 16-bit signed values
    data->accel_x = (accel_data[0] << 8) | accel_data[1];
    data->accel_y = (accel_data[2] << 8) | accel_data[3];
    data->accel_z = (accel_data[4] << 8) | accel_data[5];

    // Read gyroscope data (6 bytes: X, Y, Z with 2 bytes each)
    uint8_t gyro_data[6] = {0};
    i2c_read_bytes(GYRO_XOUT_H, gyro_data, 6);

    // Convert the data to 16-bit signed values
    data->gyro_x = (gyro_data[0] << 8) | gyro_data[1];
    data->gyro_y = (gyro_data[2] << 8) | gyro_data[3];
    data->gyro_z = (gyro_data[4] << 8) | gyro_data[5];
#else
    // Generate simulated sensor data
    generate_simulated_data(data, time_ms);
    time_ms += 20; // 20ms per iteration (50Hz)
#endif
}

// Timer callback function - called every 20ms (50Hz)
static void sensor_timer_callback(TimerHandle_t xTimer)
{
    // Read sensor data
    read_sensor_data(&sensor_data);

    // Log the sensor data (less frequently to avoid log spam)
#if ENABLE_SENSOR_LOGGING
    static uint32_t log_counter = 0;
    log_counter++;

    if (log_counter % 25 == 0) // Log every 500ms (25 * 20ms)
    {
        ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d",
                 sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
                 sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
    }
#endif

    // Send data to PID controller task
    if (xQueueSend(sensor_data_queue, &sensor_data, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Failed to send sensor data to queue");
    }
}

void sensor_monitor_task(void *pvParameters)
{
    // Always log this message regardless of ENABLE_SENSOR_LOGGING
    ESP_LOGI(TAG, "Sensor monitor task started");

    // Create a timer that triggers every 20ms (50Hz)
    sensor_timer = xTimerCreate(
        "SensorTimer",        // Timer name
        pdMS_TO_TICKS(20),    // Timer period in ticks (20ms)
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

    // Start the timer
    if (xTimerStart(sensor_timer, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start sensor timer");
        vTaskDelete(NULL);
        return;
    }

    // Add a confirmation log that the timer started successfully
    ESP_LOGI(TAG, "Sensor timer started successfully");

    // Task remains alive to keep the timer running
    while (1)
    {
        // Add periodic heartbeat log to confirm task is still running
        ESP_LOGI(TAG, "Sensor monitor heartbeat");
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Log every 5 seconds
    }
}