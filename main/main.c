#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sensor_monitor.h"
#include "command_handler.h"
#include "pid_controller.h"
#include "motor_control.h"

// Add near the top of the file, after the includes
#define ENABLE_MAIN_LOGGING 0 // Set to 0 to disable main task logging

static const char *TAG = "MAIN";

// Queues for inter-task communication
QueueHandle_t sensor_data_queue;
QueueHandle_t command_queue;

// Task handles for monitoring
TaskHandle_t sensor_task_handle;
TaskHandle_t command_task_handle;
TaskHandle_t pid_task_handle;
TaskHandle_t esc_task_handle;

// Debug task to monitor system status
void debug_monitor_task(void *pvParameters)
{
#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Debug monitor task started");
#endif

    while (1)
    {
        // Check if tasks are still running
        eTaskState sensor_state = eTaskGetState(sensor_task_handle);
        eTaskState command_state = eTaskGetState(command_task_handle);
        eTaskState pid_state = eTaskGetState(pid_task_handle);
        eTaskState esc_state = eTaskGetState(esc_task_handle);

#if ENABLE_MAIN_LOGGING
        ESP_LOGI(TAG, "System running - Tasks status:");
        ESP_LOGI(TAG, "  Sensor task: %s",
                 (sensor_state == eReady || sensor_state == eRunning || sensor_state == eBlocked)
                     ? "ACTIVE"
                     : "INACTIVE");
        ESP_LOGI(TAG, "  Command task: %s",
                 (command_state == eReady || command_state == eRunning || command_state == eBlocked)
                     ? "ACTIVE"
                     : "INACTIVE");
        ESP_LOGI(TAG, "  PID task: %s",
                 (pid_state == eReady || pid_state == eRunning || pid_state == eBlocked)
                     ? "ACTIVE"
                     : "INACTIVE");
        ESP_LOGI(TAG, "  ESC task: %s",
                 (esc_state == eReady || esc_state == eRunning || esc_state == eBlocked)
                     ? "ACTIVE"
                     : "INACTIVE");

        // Print memory info
        ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
#endif

        vTaskDelay(3000 / portTICK_PERIOD_MS); // Print status every 3 seconds
    }
}

void app_main(void)
{
#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Flight Controller Starting...");
#endif

    // Initialize queues for inter-task communication
    sensor_data_queue = xQueueCreate(5, sizeof(sensor_data_t));
    command_queue = xQueueCreate(5, sizeof(flight_command_t));

    if (!sensor_data_queue || !command_queue)
    {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }

#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Queues created successfully");
    ESP_LOGI(TAG, "Initializing sensors...");
#endif
    init_sensors();

#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Initializing motor control...");
#endif
    init_motors();

#if ENABLE_MAIN_LOGGING
    ESP_LOGI(TAG, "Creating tasks...");
#endif

    BaseType_t result = xTaskCreate(sensor_monitor_task, "sensor_monitor", 4096, NULL, 5, &sensor_task_handle);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create sensor monitor task: %d", result);
    }
    else
    {
        ESP_LOGI(TAG, "Sensor monitor task created");
    }

    result = xTaskCreate(command_handler_task, "command_handler", 4096, NULL, 4, &command_task_handle);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create command handler task: %d", result);
    }
    else
    {
        ESP_LOGI(TAG, "Command handler task created");
    }

    result = xTaskCreate(pid_controller_task, "pid_controller", 4096, NULL, 6, &pid_task_handle);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create PID controller task: %d", result);
    }
    else
    {
        ESP_LOGI(TAG, "PID controller task created");
    }

    // Create ESC control task
    result = xTaskCreate(esc_control_task, "esc_control", 4096, NULL, 7, &esc_task_handle);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create ESC control task: %d", result);
    }
    else
    {
        ESP_LOGI(TAG, "ESC control task created");
    }

    // Create debug monitor task
    result = xTaskCreate(debug_monitor_task, "debug_monitor", 4096, NULL, 1, NULL);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create debug monitor task: %d", result);
    }
#if ENABLE_MAIN_LOGGING
    else
    {
        ESP_LOGI(TAG, "Debug monitor task created");
    }

    ESP_LOGI(TAG, "All tasks started");
#endif

    // Keep app_main alive to prevent potential issues
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}