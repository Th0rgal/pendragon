#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

// #include "sensor_monitor.h" // Removed
#include "command_handler.h"
#include "motor_control.h"
#include "led_handler.h"
#include "ble_handler.h"
#if ENABLE_DPS310
#include "dps310_sensor.h"
#endif
#include "icm42688p_sensor.h" // new include

// Add near the top of the file, after the includes
#define ENABLE_MAIN_LOGGING 0  // Set to 0 to disable main task logging
#define ENABLE_MOTOR_CONTROL 0 // Set to 0 to disable motor control (disconnected)
#define ENABLE_DPS310 0        // Set to 1 to enable DPS310 barometer

static const char *TAG = "MAIN";

// Queues for inter-task communication
// QueueHandle_t sensor_data_queue; // Removed
QueueHandle_t command_queue;

// Task handles for monitoring
// TaskHandle_t sensor_task_handle; // Removed
TaskHandle_t command_task_handle;
TaskHandle_t esc_task_handle;
TaskHandle_t debug_task_handle;
TaskHandle_t led_task_handle;
#if ENABLE_DPS310
TaskHandle_t dps310_task_handle;
#endif
TaskHandle_t icm42688p_task_handle;

// Semaphore for task synchronization
SemaphoreHandle_t startup_sync_semaphore;

// Debug task to monitor system status
void debug_monitor_task(void *pvParameters)
{
#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Debug monitor task started");
#endif

    while (1)
    {
#if ENABLE_MAIN_LOGGING
        // Check if tasks are still running (with null checks)
        if (command_task_handle != NULL)
        {
            eTaskState command_state = eTaskGetState(command_task_handle);
            ESP_LOGI(TAG, "  Command task: %s",
                     (command_state == eReady || command_state == eRunning || command_state == eBlocked)
                         ? "ACTIVE"
                         : "INACTIVE");
        }

#if ENABLE_MOTOR_CONTROL
        if (esc_task_handle != NULL)
        {
            eTaskState esc_state = eTaskGetState(esc_task_handle);
            ESP_LOGI(TAG, "  ESC task: %s",
                     (esc_state == eReady || esc_state == eRunning || esc_state == eBlocked)
                         ? "ACTIVE"
                         : "INACTIVE");
        }
#endif

#if ENABLE_DPS310
        if (dps310_task_handle != NULL)
        {
            eTaskState dps310_state = eTaskGetState(dps310_task_handle);
            ESP_LOGI(TAG, "  DPS310 task: %s",
                     (dps310_state == eReady || dps310_state == eRunning || dps310_state == eBlocked)
                         ? "ACTIVE"
                         : "INACTIVE");
        }
#endif

        if (icm42688p_task_handle != NULL)
        {
            eTaskState icm_state = eTaskGetState(icm42688p_task_handle);
            ESP_LOGI(TAG, "  ICM42688P task: %s",
                     (icm_state == eReady || icm_state == eRunning || icm_state == eBlocked)
                         ? "ACTIVE"
                         : "INACTIVE");
        }

        ESP_LOGI(TAG, "System running - Tasks status checked");
        // Print memory info
        ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
#else
        // When logging is disabled, just check if tasks are still running without storing the state
        if (command_task_handle != NULL)
        {
            eTaskGetState(command_task_handle);
        }
#if ENABLE_MOTOR_CONTROL
        if (esc_task_handle != NULL)
        {
            eTaskGetState(esc_task_handle);
        }
#endif
#if ENABLE_DPS310
        if (dps310_task_handle != NULL)
        {
            eTaskGetState(dps310_task_handle);
        }
#endif
        if (icm42688p_task_handle != NULL)
        {
            eTaskGetState(icm42688p_task_handle);
        }
#endif

        vTaskDelay(3000 / portTICK_PERIOD_MS); // Print status every 3 seconds
    }
}

void app_main(void)
{
    // Set log levels for different components
    // esp_log_level_set("SENSOR_MONITOR", ESP_LOG_INFO); // Removed
    esp_log_level_set("BLE_HANDLER", ESP_LOG_DEBUG);
#if ENABLE_MOTOR_CONTROL
    esp_log_level_set("MOTOR_CONTROL", ESP_LOG_DEBUG);
#else
    esp_log_level_set("MOTOR_CONTROL", ESP_LOG_WARN); // Reduce motor logging when disabled
#endif
    esp_log_level_set("DPS310_SENSOR", ESP_LOG_DEBUG);

#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Flight Controller Starting...");
#endif

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize BLE
    ret = init_ble();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize BLE");
        return;
    }

    // Create semaphore for task synchronization
    startup_sync_semaphore = xSemaphoreCreateBinary();
    if (startup_sync_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create startup sync semaphore");
        return;
    }

    // Create queues for communication - increased to 50 items
    // sensor_data_queue = xQueueCreate(50, sizeof(sensor_data_t)); // Removed
    command_queue = xQueueCreate(50, sizeof(flight_command_t));

    if (/* !sensor_data_queue || */ !command_queue) // Adjusted condition
    {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }

#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Queues created successfully");
    // ESP_LOGI(TAG, "Initializing sensors..."); // Removed
#endif
    // init_sensors(); // Removed

#if ENABLE_MOTOR_CONTROL
#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Initializing motor control...");
#endif
    init_motors();
#else
#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Motor control disabled - skipping initialization");
#endif
#endif

#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Creating tasks...");
#endif

    // Create tasks with proper priorities
    // xTaskCreate(sensor_monitor_task, "sensor_monitor", 8192, (void *)startup_sync_semaphore, 6, &sensor_task_handle); // Removed
    // ESP_LOGI(TAG, "Sensor monitor task created"); // Removed

    xTaskCreate(command_handler_task, "command_handler", 8192, (void *)startup_sync_semaphore, 5, &command_task_handle);
    ESP_LOGI(TAG, "Command handler task created");

#if ENABLE_MOTOR_CONTROL
    xTaskCreate(esc_control_task, "esc_control", 8192, (void *)startup_sync_semaphore, 4, &esc_task_handle);
    ESP_LOGI(TAG, "ESC control task created");
#else
    ESP_LOGI(TAG, "ESC control task disabled");
    esc_task_handle = NULL; // Ensure it's null when disabled
#endif

    // Create LED task
    xTaskCreate(led_handler_task, "led_handler", 4096, NULL, 2, &led_task_handle);

#if ENABLE_DPS310
    // Create DPS310 sensor task
    xTaskCreate(dps310_sensor_task, "dps310_sensor", 8192, NULL, 3, &dps310_task_handle);
#endif

    // Create ICM-42688-P sensor task
    xTaskCreate(icm42688p_sensor_task, "icm42688p_sensor", 8192, NULL, 3, &icm42688p_task_handle);

    // Create debug monitor task (after other tasks are created)
    xTaskCreate(debug_monitor_task, "debug_monitor", 4096, NULL, 1, &debug_task_handle);

    // Give the semaphore to signal that all tasks are created
    // and can begin their normal operation
    xSemaphoreGive(startup_sync_semaphore);

#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "All tasks started");
#endif

    // Keep app_main alive to prevent potential issues
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}