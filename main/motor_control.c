#include "motor_control.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "sensor_monitor.h"  // For sensor_data_t
#include "command_handler.h" // For flight_command_t

// Queue handles declared in main.c
extern QueueHandle_t sensor_data_queue;
extern QueueHandle_t command_queue;

// Utility macros
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static const char *TAG = "MOTOR_CONTROL";

// Constants for motor control
#define ENABLE_LOGGING 0 // Set to 0 to disable logging

// PWM Configuration
#define PWM_FREQUENCY 50                 // 50Hz for standard ESCs
#define PWM_RESOLUTION LEDC_TIMER_13_BIT // 13-bit resolution (0-8191)
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE

// Motor pin and channel configuration
#define MOTOR_COUNT 4

// Motor pins for each motor
const uint8_t MOTOR_PINS[MOTOR_COUNT] = {
    10, // MOTOR_TOP_RIGHT (GPIO 10)
    9,  // MOTOR_BOTTOM_RIGHT (GPIO 9)
    18, // MOTOR_TOP_LEFT (GPIO 18)
    17  // MOTOR_BOTTOM_LEFT (GPIO 17)
};

// LEDC channels for each motor
const uint8_t MOTOR_CHANNELS[MOTOR_COUNT] = {
    LEDC_CHANNEL_0, // MOTOR_TOP_RIGHT
    LEDC_CHANNEL_1, // MOTOR_BOTTOM_RIGHT
    LEDC_CHANNEL_2, // MOTOR_TOP_LEFT
    LEDC_CHANNEL_3  // MOTOR_BOTTOM_LEFT
};

// ESC signal calibration
#define MIN_THROTTLE_PULSE_US 1000 // 1ms pulse (minimum throttle)
#define MAX_THROTTLE_PULSE_US 2000 // 2ms pulse (maximum throttle)
#define WORKING_THROTTLE_VALUE 200 // Input throttle value where motor starts working (~1200 µs)

// Global state
static uint8_t motor_ramp_rates[MOTOR_COUNT] = {5, 5, 5, 5};      // Default ramp rate (units per 100ms)
static uint16_t current_motor_speeds[MOTOR_COUNT] = {0, 0, 0, 0}; // Current speed for each motor

// Calculate PWM duty cycle from pulse width in microseconds
static uint32_t pulse_us_to_duty(uint32_t pulse_us)
{
    // For 50Hz, period is 20,000 µs. Map pulse_us (1000-2000) to duty cycle
    return (pulse_us * ((1 << PWM_RESOLUTION) - 1)) / 20000;
}

// Set acceleration rate (1-50 units per 100ms)
void set_motor_acceleration(motor_id_t motor, uint8_t ramp_rate)
{
    // Validate input range
    if (ramp_rate < 1)
    {
        ramp_rate = 1; // Minimum allowed (very slow acceleration)
    }
    else if (ramp_rate > 50)
    {
        ramp_rate = 50; // Maximum allowed (rapid acceleration)
    }

    if (motor == MOTOR_ALL)
    {
        // Set ramp rate for all motors
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            motor_ramp_rates[i] = ramp_rate;
        }
#if ENABLE_LOGGING
        ESP_LOGI(TAG, "All motors acceleration set to %d units per 100ms", ramp_rate);
#endif
    }
    else if (motor < MOTOR_COUNT)
    {
        motor_ramp_rates[motor] = ramp_rate;
#if ENABLE_LOGGING
        ESP_LOGI(TAG, "Motor %d acceleration set to %d units per 100ms", motor, ramp_rate);
#endif
    }
    else
    {
#if ENABLE_LOGGING
        ESP_LOGW(TAG, "Invalid motor ID: %d", motor);
#endif
        return;
    }

    // Calculate approximate time for a full 0-1000 ramp
    float full_ramp_time = (1000.0f / ramp_rate) * 0.1f; // in seconds
#if ENABLE_LOGGING
    ESP_LOGI(TAG, "Full throttle ramp will take approximately %.1f seconds", full_ramp_time);
#endif
}

// Initialize motor PWM channels
void init_motors(void)
{
#if ENABLE_LOGGING
    ESP_LOGI(TAG, "Initializing motor control for %d motors", MOTOR_COUNT);
#endif

    // Configure timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    // Configure all motor channels
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        ledc_channel_config_t channel_conf = {
            .channel = MOTOR_CHANNELS[i],
            .duty = pulse_us_to_duty(MIN_THROTTLE_PULSE_US), // Start at minimum throttle
            .gpio_num = MOTOR_PINS[i],
            .speed_mode = PWM_MODE,
            .hpoint = 0,
            .timer_sel = PWM_TIMER};
        ledc_channel_config(&channel_conf);

#if ENABLE_LOGGING
        ESP_LOGI(TAG, "Motor %d initialized on GPIO%d (channel %d)", i, MOTOR_PINS[i], MOTOR_CHANNELS[i]);
#endif
    }

    // Initialize default acceleration for all motors
    set_motor_acceleration(MOTOR_ALL, 5);
}

// Set motor speed (0-1000 input range)
void set_motor_speed(motor_id_t motor, uint16_t speed)
{
    // Limit speed to 0-1000 range
    if (speed > 1000)
    {
        speed = 1000;
    }

    // Map 0-1000 to ESC range (1000-2000µs)
    uint32_t pulse_us;
    if (speed == 0)
    {
        pulse_us = MIN_THROTTLE_PULSE_US; // Min throttle/off
    }
    else
    {
        pulse_us = MIN_THROTTLE_PULSE_US + speed; // 1000 + speed → 1000-2000µs range
    }

    // Convert to duty cycle
    uint32_t duty = pulse_us_to_duty(pulse_us);

    if (motor == MOTOR_ALL)
    {
        // Set all motors to the same speed
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            ledc_set_duty(PWM_MODE, MOTOR_CHANNELS[i], duty);
            ledc_update_duty(PWM_MODE, MOTOR_CHANNELS[i]);
            current_motor_speeds[i] = speed;
        }

#if ENABLE_LOGGING
        ESP_LOGI(TAG, "All motors set to %d (pulse: %lu µs, duty: %lu)",
                 speed, pulse_us, duty);
#endif
    }
    else if (motor < MOTOR_COUNT)
    {
        // Set specific motor
        ledc_set_duty(PWM_MODE, MOTOR_CHANNELS[motor], duty);
        ledc_update_duty(PWM_MODE, MOTOR_CHANNELS[motor]);
        current_motor_speeds[motor] = speed;

#if ENABLE_LOGGING
        // Only log significant changes to reduce log spam
        static uint16_t last_speeds[MOTOR_COUNT] = {0, 0, 0, 0};
        static uint32_t log_counter = 0;
        static bool logged_zero[MOTOR_COUNT] = {false, false, false, false};

        // Only log in these specific cases
        bool should_log = false;

        if (speed == 0 && !logged_zero[motor])
        {
            // First time we detect motor turned off
            should_log = true;
            logged_zero[motor] = true;
        }
        else if (speed > 0 && last_speeds[motor] == 0)
        {
            // Motor turned on from off state
            should_log = true;
            logged_zero[motor] = false;
        }
        else if (abs((int)last_speeds[motor] - (int)speed) > 100)
        {
            // Large speed change
            should_log = true;
        }
        else if (log_counter % 500 == 0)
        {
            // Occasional status update
            should_log = true;
        }

        log_counter++;

        if (should_log)
        {
            ESP_LOGI(TAG, "Motor %d set to %d (pulse: %lu µs, duty: %lu)",
                     motor, speed, pulse_us, duty);
            last_speeds[motor] = speed;
        }
#endif
    }
    else
    {
        ESP_LOGW(TAG, "Invalid motor ID: %d", motor);
    }
}

// Helper function to drain both queues - used during initialization and waiting periods
static void drain_queues(sensor_data_t *sensor_data, flight_command_t *command)
{
    UBaseType_t sensor_queue_items = uxQueueMessagesWaiting(sensor_data_queue);
    UBaseType_t command_queue_items = uxQueueMessagesWaiting(command_queue);

    // If either queue has items, drain them
    if (sensor_queue_items > 0 || command_queue_items > 0)
    {
        // Drain sensor data queue
        int drained_sensor = 0;
        while (xQueueReceive(sensor_data_queue, sensor_data, 0) == pdTRUE)
        {
            drained_sensor++;
            if (drained_sensor >= 50)
                break; // Safety limit
        }

        // Drain command queue
        int drained_command = 0;
        while (xQueueReceive(command_queue, command, 0) == pdTRUE)
        {
            drained_command++;
            if (drained_command >= 10)
                break; // Safety limit
        }

        // Only log if we drained something significant
        if (drained_sensor > 5 || drained_command > 2)
        {
#if ENABLE_LOGGING
            ESP_LOGI(TAG, "Drained %d sensor items and %d command items",
                     drained_sensor, drained_command);
#endif
        }
    }
}

// Task to control the ESCs/motors for testing
void esc_control_task(void *pvParameters)
{
    // Variables declared at the top to avoid redeclaration issues
    sensor_data_t sensor_data;
    flight_command_t command;
    TickType_t drain_start_time, current_time, time_limit;
    uint32_t step_counter, log_counter;
    bool motors_running[MOTOR_COUNT] = {false};
    bool is_now_running;
    int drain_counter = 0;
    UBaseType_t sensor_queue_items, command_queue_items;

#if ENABLE_LOGGING
    ESP_LOGI(TAG, "ESC control task started");
#endif

    // Get the synchronization semaphore
    SemaphoreHandle_t startup_sync = (SemaphoreHandle_t)pvParameters;

    // Wait for the synchronization signal from main - this should be the first task to take it
    // as it is the consumer that needs to be ready before producers start
    if (startup_sync != NULL)
    {
#if ENABLE_LOGGING
        ESP_LOGI(TAG, "Waiting for startup synchronization...");
#endif
        if (xSemaphoreTake(startup_sync, pdMS_TO_TICKS(5000)) != pdTRUE)
        {
#if ENABLE_LOGGING
            ESP_LOGW(TAG, "Timeout waiting for startup sync - proceeding anyway");
#endif
        }
        else
        {
#if ENABLE_LOGGING
            ESP_LOGI(TAG, "Received startup synchronization");
#endif
            // Now we're the first to take the semaphore, so we'll hold it for a moment
            // to ensure we're ready to consume data before other tasks start producing
            vTaskDelay(200 / portTICK_PERIOD_MS);

            // Give back the semaphore to allow other tasks to proceed
            xSemaphoreGive(startup_sync);
        }
    }

    // Initial queue draining
#if ENABLE_LOGGING
    ESP_LOGI(TAG, "Initial queue draining...");
#endif
    drain_queues(&sensor_data, &command);

    // Brief initialization delay
    vTaskDelay(500 / portTICK_PERIOD_MS);

#if ENABLE_LOGGING
    ESP_LOGI(TAG, "Starting ESC initialization sequence");
#endif

    // Step 1: Set all motors to min throttle
#if ENABLE_LOGGING
    ESP_LOGI(TAG, "Step 1: Setting minimum throttle for all motors");
#endif
    set_motor_speed(MOTOR_ALL, 0); // Set all to minimum

    // During the 3-second wait, periodically drain queues to prevent overflow
    for (int i = 0; i < 30; i++)
    { // 30 * 100ms = 3000ms
        drain_queues(&sensor_data, &command);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Step 2: Test with a working throttle value for all motors
#if ENABLE_LOGGING
    ESP_LOGI(TAG, "Step 2: Testing all motors with working throttle value (200)");
#endif
    set_motor_speed(MOTOR_ALL, 200); // 20% throttle - should start spinning

    // During the 5-second wait, periodically drain queues to prevent overflow
    for (int i = 0; i < 50; i++)
    { // 50 * 100ms = 5000ms
        drain_queues(&sensor_data, &command);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

#if ENABLE_LOGGING
    ESP_LOGI(TAG, "Starting test cycle");
#endif

    // Define test steps with different accelerations
    const struct
    {
        const char *description;
        uint16_t throttle_value;
        uint16_t duration_ds; // Duration in deciseconds (10 = 1 second)
        uint8_t accel_rate;   // Acceleration rate for this step
    } test_steps[] = {
        {"Test 1: Off (value: 0)", 0, 30, 5},               // 3 seconds - standard accel
        {"Test 2: Low speed (value: 250)", 250, 50, 5},     // 5 seconds - standard accel
        {"Test 3: Medium speed (value: 500)", 500, 50, 10}, // 5 seconds - medium accel
        {"Test 4: High speed (value: 750)", 750, 50, 15},   // 5 seconds - fast accel
        {"Test 5: Max speed (value: 1000)", 1000, 30, 20},  // 3 seconds - very fast accel
        {"Test 6: Medium-high (value: 750)", 750, 30, 15},  // 3 seconds - fast decel
        {"Test 7: Medium (value: 500)", 500, 30, 10},       // 3 seconds - medium decel
        {"Test 8: Low (value: 250)", 250, 30, 5},           // 3 seconds - standard decel
        {"Test 9: Back to off", 0, 30, 5}                   // 3 seconds - standard decel
    };

    // Initialize variables for each motor
    int current_step = 0;
    int num_steps = sizeof(test_steps) / sizeof(test_steps[0]);
    int last_logged_step = -1;
    uint16_t target_throttles[MOTOR_COUNT];

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        target_throttles[i] = test_steps[0].throttle_value;
    }

    step_counter = 0;
    log_counter = 0;

    // Start with standard acceleration for all motors
    set_motor_acceleration(MOTOR_ALL, 5);

    // Main control loop
    while (1)
    {
        // Drain any accumulated queue data before processing
        drain_queues(&sensor_data, &command);

        // Increment counters
        log_counter++;
        step_counter++;

        // Check if it's time to move to the next step
        if (step_counter >= test_steps[current_step].duration_ds)
        {
            current_step = (current_step + 1) % num_steps;
            step_counter = 0;

            // Set the target throttle for all motors
            for (int i = 0; i < MOTOR_COUNT; i++)
            {
                target_throttles[i] = test_steps[current_step].throttle_value;
            }

            // Set the acceleration rate for this step for all motors
            set_motor_acceleration(MOTOR_ALL, test_steps[current_step].accel_rate);

            // Log when changing steps
#if ENABLE_LOGGING
            ESP_LOGI(TAG, "Moving to %s (accel: %d)",
                     test_steps[current_step].description,
                     test_steps[current_step].accel_rate);
#endif

            last_logged_step = current_step;
        }

        // Smoothly approach the target throttle for each motor
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (current_motor_speeds[i] < target_throttles[i])
            {
                current_motor_speeds[i] = MIN(current_motor_speeds[i] + motor_ramp_rates[i], target_throttles[i]);
            }
            else if (current_motor_speeds[i] > target_throttles[i])
            {
                current_motor_speeds[i] = MAX(current_motor_speeds[i] - motor_ramp_rates[i], target_throttles[i]);
            }

            // Apply current throttle value to each motor
            set_motor_speed(i, current_motor_speeds[i]);

            // Track motor state for potential state change logging
            is_now_running = (current_motor_speeds[i] > 0);
            if (is_now_running != motors_running[i])
            {
#if ENABLE_LOGGING
                ESP_LOGI(TAG, "Motor %d %s", i, is_now_running ? "ON" : "OFF");
#endif
                motors_running[i] = is_now_running;
            }
        }

        // Log status only when step changes or every 50 iterations
        if (current_step != last_logged_step || log_counter % 50 == 0)
        {
#if ENABLE_LOGGING
            ESP_LOGI(TAG, "%s (Acceleration: %d)",
                     test_steps[current_step].description,
                     test_steps[current_step].accel_rate);
#endif

            // Log throttle for each motor
            for (int i = 0; i < MOTOR_COUNT; i++)
            {
                // Add visual indicator for throttle percentage
                char throttle_bar[21] = "[                    ]";
                int bar_length = 20;
                int fill_amount = (current_motor_speeds[i] * bar_length) / 1000;

                for (int j = 0; j < fill_amount; j++)
                {
                    throttle_bar[j + 1] = '=';
                }

                // Add a position marker
                if (fill_amount < bar_length)
                {
                    throttle_bar[fill_amount + 1] = '>';
                }

                // Show throttle percentage with visual indicator for each motor
#if ENABLE_LOGGING
                ESP_LOGI(TAG, "Motor %d: %d%% %s",
                         i, current_motor_speeds[i] / 10,
                         throttle_bar);
#endif
            }

            last_logged_step = current_step;
        }

        // Brief delay before next iteration
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}