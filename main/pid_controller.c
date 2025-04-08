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
    float m1 = base_throttle - roll - pitch + yaw;  // Front right
    float m2 = base_throttle - roll + pitch - yaw;  // Rear right
    float m3 = base_throttle + roll + pitch + yaw;  // Rear left
    float m4 = base_throttle + roll - pitch - yaw;  // Front left
    
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
    uint32_t current_time = 0;
    bool armed = false;
    uint32_t test_counter = 0;

    ESP_LOGI(TAG, "PID controller task started");

    // Wait a bit before starting motor tests to let the ESC initialize
    vTaskDelay(15000 / portTICK_PERIOD_MS);  // Wait 15 seconds
    ESP_LOGI(TAG, "Starting motor test sequence from PID controller");

    while (1)
    {
        current_time += 10; // 10ms per iteration
        test_counter++;
        
        // Get latest sensor data
        if (xQueueReceive(sensor_data_queue, &sensor_data, 0) == pdTRUE)
        {
            // Process sensor data (will be used in PID implementation)
            ESP_LOGD(TAG, "Received sensor data");
        }

        // Get latest command
        if (xQueueReceive(command_queue, &command, 0) == pdTRUE)
        {
            // Check if arm status changed
            if (command.arm_status != armed) {
                armed = command.arm_status;
                ESP_LOGI(TAG, "Arm status changed to: %s", armed ? "ARMED" : "DISARMED");
            }
        }

        // Test sequence using known working values (200 maps to 1200 on ESC)
        // We use a simple state machine approach with longer periods at each level
        
        // Change state every 10 seconds (1000 * 10ms iterations)
        uint32_t state = (test_counter / 1000) % 5;
        uint16_t test_throttle = 0;
        
        switch(state) {
            case 0:  // Minimum/off state
                test_throttle = 0;
                break;
            case 1:  // Start at known working threshold
                test_throttle = 200;  // Maps to 1200 - confirmed working value
                break;
            case 2:  // Medium-low throttle 
                test_throttle = 220;  // Maps to 1220
                break;
            case 3:  // Medium throttle
                test_throttle = 250;  // Maps to 1250
                break;
            case 4:  // Back to lower throttle
                test_throttle = 200;  // Maps to 1200 - confirmed working value
                break;
        }
        
        // Log at the beginning of each state
        if (test_counter % 1000 == 0) {
            ESP_LOGI(TAG, "PID TEST STATE %lu: Setting throttle to %u", 
                     state, test_throttle);
        }
        
        // Set all motors to the test throttle
        // For a single ESC setup, only motor1 actually matters
        motor_command.motor1 = test_throttle;
        motor_command.motor2 = test_throttle;
        motor_command.motor3 = test_throttle;
        motor_command.motor4 = test_throttle;

        // Apply motor commands directly
        set_all_motors(&motor_command);
        
        // PID control frequency - 100Hz
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}