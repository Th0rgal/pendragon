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

// Add near the top of the file, after the includes
#define ENABLE_MAIN_LOGGING 0 // Set to 0 to disable main task logging

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
        // Check if tasks are still running
        // eTaskState sensor_state = eTaskGetState(sensor_task_handle); // Removed
        eTaskState command_state = eTaskGetState(command_task_handle);
        eTaskState esc_state = eTaskGetState(esc_task_handle);

        ESP_LOGI(TAG, "System running - Tasks status:");
        // ESP_LOGI(TAG, "  Sensor task: %s", // Removed
        //          (sensor_state == eReady || sensor_state == eRunning || sensor_state == eBlocked) // Removed
        //              ? "ACTIVE" // Removed
        //              : "INACTIVE"); // Removed
        ESP_LOGI(TAG, "  Command task: %s",
                 (command_state == eReady || command_state == eRunning || command_state == eBlocked)
                     ? "ACTIVE"
                     : "INACTIVE");
        ESP_LOGI(TAG, "  ESC task: %s",
                 (esc_state == eReady || esc_state == eRunning || esc_state == eBlocked)
                     ? "ACTIVE"
                     : "INACTIVE");

        // Print memory info
        ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
#else
        // When logging is disabled, just check if tasks are still running without storing the state
        // eTaskGetState(sensor_task_handle); // Removed
        eTaskGetState(command_task_handle);
        eTaskGetState(esc_task_handle);
#endif

        vTaskDelay(3000 / portTICK_PERIOD_MS); // Print status every 3 seconds
    }
}

void app_main(void)
{
    // Set log levels for different components
    // esp_log_level_set("SENSOR_MONITOR", ESP_LOG_INFO); // Removed
    esp_log_level_set("BLE_HANDLER", ESP_LOG_DEBUG);
    esp_log_level_set("MOTOR_CONTROL", ESP_LOG_DEBUG);

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

#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Initializing motor control...");
#endif
    init_motors();

#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Creating tasks...");
#endif

    // Create tasks with proper priorities
    // xTaskCreate(sensor_monitor_task, "sensor_monitor", 8192, (void *)startup_sync_semaphore, 6, &sensor_task_handle); // Removed
    // ESP_LOGI(TAG, "Sensor monitor task created"); // Removed

    xTaskCreate(command_handler_task, "command_handler", 8192, (void *)startup_sync_semaphore, 5, &command_task_handle);
    ESP_LOGI(TAG, "Command handler task created");

    xTaskCreate(esc_control_task, "esc_control", 8192, (void *)startup_sync_semaphore, 4, &esc_task_handle);
    ESP_LOGI(TAG, "ESC control task created");

    // Create debug monitor task
    xTaskCreate(debug_monitor_task, "debug_monitor", 4096, NULL, 1, &debug_task_handle);

    // Create LED task
    xTaskCreate(led_handler_task, "led_handler", 4096, NULL, 2, &led_task_handle);

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