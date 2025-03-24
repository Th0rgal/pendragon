#include "command_handler.h"
#include "esp_log.h"
#include "freertos/task.h"
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

    ESP_LOGI(TAG, "Command handler task started");

    while (1)
    {
        // Generate test commands that change over time
        counter++;

        // Every 5 seconds, toggle arm status
        if (counter % 50 == 0)
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
            float t = (counter % 100) / 100.0f; // 0.0 to 1.0 over 10 seconds
            command.throttle = 0.1f + 0.2f * sin(t * 3.14159f * 2);

            // Create some gentle roll/pitch/yaw movements
            command.roll = 0.2f * sin(counter / 20.0f);
            command.pitch = 0.2f * cos(counter / 30.0f);
            command.yaw = 0.1f * sin(counter / 50.0f);

#if ENABLE_COMMAND_LOGGING
            if (counter % 10 == 0)
            { // Log every second
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

        // Send command to PID controller
        if (xQueueSend(command_queue, &command, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "Failed to send command to queue");
        }

        // Command processing frequency
        vTaskDelay(100 / portTICK_PERIOD_MS); // 10Hz
    }
}