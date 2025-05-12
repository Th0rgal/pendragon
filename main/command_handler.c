#include "command_handler.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

#define ENABLE_COMMAND_LOGGING 0 // Set to 0 to disable command logging

static const char *TAG = "COMMAND_HANDLER";

void command_handler_task(void *pvParameters)
{
    flight_command_t command = {
        .throttle = 0.0f,
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 0.0f,
        .arm_status = false};

    uint32_t counter = 0;
    static TickType_t last_success_time = 0;
    static uint32_t backoff_count = 0;

    ESP_LOGI(TAG, "Command handler task started");

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

    while (1)
    {
        // Generate test commands that change over time
        counter++;

        // Every 5 seconds, toggle arm status
        if (counter % 25 == 0) // Changed from 50 to 25 since we now run at 5Hz
        {
            command.arm_status = !command.arm_status;
#if ENABLE_COMMAND_LOGGING
            ESP_LOGI(TAG, "Arm status changed to: %s", command.arm_status ? "ARMED" : "DISARMED");
#endif
        }

        // If armed, create some test throttle values
        if (command.arm_status)
        {
            // Slowly ramp throttle up and down between 0.1 and 0.3
            float t = (counter % 50) / 50.0f; // 0.0 to 1.0 over 10 seconds (at 5Hz)
            command.throttle = 0.1f + 0.2f * sin(t * 3.14159f * 2);

            // Create some gentle roll/pitch/yaw movements
            command.roll = 0.2f * sin(counter / 10.0f);
            command.pitch = 0.2f * cos(counter / 15.0f);
            command.yaw = 0.1f * sin(counter / 25.0f);

#if ENABLE_COMMAND_LOGGING
            if (counter % 5 == 0) // Log every second (at 5Hz)
            {
                ESP_LOGI(TAG, "Command: throttle=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
                         command.throttle, command.roll, command.pitch, command.yaw);
            }
#endif
        }
        else
        {
            // When disarmed, all controls at zero
            command.throttle = 0.0f;
            command.roll = 0.0f;
            command.pitch = 0.0f;
            command.yaw = 0.0f;
        }

        // Send command to queue with more intelligent backoff strategy
        /* // Temporarily disabled to prevent interference with BLE commands
        if (xQueueSend(command_queue, &command, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            // Success - reset backoff
            backoff_count = 0;
            last_success_time = xTaskGetTickCount();
        }
        else
        {
            // Increase backoff counter
            backoff_count++;

            // Only log if we haven't successfully sent data in a while (5 seconds)
            if (backoff_count > 25 && (xTaskGetTickCount() - last_success_time) > pdMS_TO_TICKS(5000))
            {
                ESP_LOGW(TAG, "Queue full for 5 seconds - consider process balance");
                // Reset after logging to avoid spam
                last_success_time = xTaskGetTickCount();
                backoff_count = 0;
            }

            // Add a small additional delay on failure to help system recover
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        */

        // Command processing frequency - reduced from 10Hz to 5Hz
        vTaskDelay(200 / portTICK_PERIOD_MS); // 5Hz
    }
}