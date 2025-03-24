#include "pid_controller.h"
#include "esp_log.h"
#include "freertos/task.h"

#define ENABLE_PID_LOGGING 0 // Set to 0 to disable PID controller logging

static const char *TAG = "PID_CONTROLLER";

// PID controller structure
typedef struct
{
    float kp;         // Proportional gain
    float ki;         // Integral gain
    float kd;         // Derivative gain
    float setpoint;   // Desired value
    float integral;   // Integral term
    float prev_error; // Previous error for derivative
} pid_controller_t;

// Initialize PID controllers for each axis
static pid_controller_t roll_pid = {0};
static pid_controller_t pitch_pid = {0};
static pid_controller_t yaw_pid = {0};

// PID calculation function
static float pid_compute(pid_controller_t *pid, float input, float dt)
{
    // TODO: Implement PID algorithm
    return 0.0f; // Placeholder
}

// Simple mixer function to convert throttle/roll/pitch/yaw to motor outputs
static void calculate_motor_outputs(
    float throttle, float roll, float pitch, float yaw,
    motor_command_t *motor_cmd)
{
    // Basic quadcopter mixing algorithm (X configuration)
    // Motor order: 1=front-right, 2=rear-right, 3=rear-left, 4=front-left

    // Scale throttle from 0-1 to 0-1000 for motor commands
    float base_throttle = throttle * 1000.0f;

    // Apply simple mixing
    float m1 = base_throttle - roll - pitch + yaw; // Front right
    float m2 = base_throttle - roll + pitch - yaw; // Rear right
    float m3 = base_throttle + roll + pitch + yaw; // Rear left
    float m4 = base_throttle + roll - pitch - yaw; // Front left

    // Constrain values and convert to uint16_t
    motor_cmd->motor1 = (uint16_t)(m1 < 0 ? 0 : (m1 > 1000 ? 1000 : m1));
    motor_cmd->motor2 = (uint16_t)(m2 < 0 ? 0 : (m2 > 1000 ? 1000 : m2));
    motor_cmd->motor3 = (uint16_t)(m3 < 0 ? 0 : (m3 > 1000 ? 1000 : m3));
    motor_cmd->motor4 = (uint16_t)(m4 < 0 ? 0 : (m4 > 1000 ? 1000 : m4));
}

void pid_controller_task(void *pvParameters)
{
    sensor_data_t sensor_data;
    flight_command_t command = {0};
    motor_command_t motor_command = {0};
    uint32_t last_log_time = 0;
    uint32_t current_time = 0;
    bool armed = false;

    ESP_LOGI(TAG, "PID controller task started");

    while (1)
    {
        current_time += 10; // 10ms per iteration

        // Get latest sensor data
        if (xQueueReceive(sensor_data_queue, &sensor_data, 0) == pdTRUE)
        {
            // Process sensor data (will be used in PID implementation)
            // For now, just acknowledge receipt
            ESP_LOGD(TAG, "Received sensor data");
        }

        // Get latest command
        if (xQueueReceive(command_queue, &command, 0) == pdTRUE)
        {
            // Check if arm status changed
            if (command.arm_status != armed)
            {
                armed = command.arm_status;
#if ENABLE_PID_LOGGING
                ESP_LOGI(TAG, "Arm status changed to: %s", armed ? "ARMED" : "DISARMED");
#endif
            }
        }

        // Calculate motor outputs based on commands
        if (armed)
        {
            // In a real implementation, this would use PID control
            // For now, just do direct mixing of the commands
            calculate_motor_outputs(
                command.throttle,
                command.roll * 0.3f,  // Scale for less aggressive response
                command.pitch * 0.3f, // Scale for less aggressive response
                command.yaw * 0.3f,   // Scale for less aggressive response
                &motor_command);
        }
        else
        {
            // When disarmed, all motors off
            motor_command.motor1 = 0;
            motor_command.motor2 = 0;
            motor_command.motor3 = 0;
            motor_command.motor4 = 0;
        }

        // Apply motor commands directly
        apply_motor_command(&motor_command);

        // Log motor outputs periodically
#if ENABLE_PID_LOGGING
        if (current_time - last_log_time >= 1000)
        { // Log every second
            ESP_LOGI(TAG, "Motors: %d, %d, %d, %d",
                     motor_command.motor1, motor_command.motor2,
                     motor_command.motor3, motor_command.motor4);
            last_log_time = current_time;
        }
#endif

        // PID control frequency
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz
    }
}