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

// Motor pin and channel (simplified for single motor)
#define MOTOR_PIN 10 // Motor 1 - GPIO10
#define MOTOR_CHANNEL LEDC_CHANNEL_0

// ESC signal calibration
#define MIN_THROTTLE_PULSE_US 1000 // 1ms pulse (minimum throttle)
#define MAX_THROTTLE_PULSE_US 2000 // 2ms pulse (maximum throttle)
#define WORKING_THROTTLE_VALUE 200 // Input throttle value where motor starts working (~1200 µs)

// Global state
static uint8_t motor_ramp_rate = 5; // Default ramp rate (units per 100ms)

// Calculate PWM duty cycle from pulse width in microseconds
static uint32_t pulse_us_to_duty(uint32_t pulse_us)
{
    // For 50Hz, period is 20,000 µs. Map pulse_us (1000-2000) to duty cycle
    return (pulse_us * ((1 << PWM_RESOLUTION) - 1)) / 20000;
}

// Set acceleration rate (1-50 units per 100ms)
void set_motor_acceleration(uint8_t ramp_rate)
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

    motor_ramp_rate = ramp_rate;
    ESP_LOGI(TAG, "Motor acceleration set to %d units per 100ms", motor_ramp_rate);

    // Calculate approximate time for a full 0-1000 ramp
    float full_ramp_time = (1000.0f / motor_ramp_rate) * 0.1f; // in seconds
    ESP_LOGI(TAG, "Full throttle ramp will take approximately %.1f seconds", full_ramp_time);
}

// Initialize motor PWM channels
void init_motors(void)
{
    ESP_LOGI(TAG, "Initializing motor control");

    // Configure timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    // Configure single motor channel
    ledc_channel_config_t channel_conf = {
        .channel = MOTOR_CHANNEL,
        .duty = pulse_us_to_duty(MIN_THROTTLE_PULSE_US), // Start at minimum throttle
        .gpio_num = MOTOR_PIN,
        .speed_mode = PWM_MODE,
        .hpoint = 0,
        .timer_sel = PWM_TIMER};
    ledc_channel_config(&channel_conf);

    ESP_LOGI(TAG, "Motor initialized on GPIO%d (channel %d)", MOTOR_PIN, MOTOR_CHANNEL);

    // Initialize default acceleration
    set_motor_acceleration(motor_ramp_rate);
}

// Set motor speed (0-1000 input range)
void set_motor_speed(uint16_t speed)
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

    // Convert to duty cycle and apply
    uint32_t duty = pulse_us_to_duty(pulse_us);
    ledc_set_duty(PWM_MODE, MOTOR_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, MOTOR_CHANNEL);

#if ENABLE_LOGGING
    // Only log significant changes to reduce log spam
    static uint16_t last_speed = 0;
    static uint32_t log_counter = 0;
    static bool logged_zero = false;

    // Only log in these specific cases
    bool should_log = false;

    if (speed == 0 && !logged_zero)
    {
        // First time we detect motor turned off
        should_log = true;
        logged_zero = true;
    }
    else if (speed > 0 && last_speed == 0)
    {
        // Motor turned on from off state
        should_log = true;
        logged_zero = false;
    }
    else if (abs((int)last_speed - (int)speed) > 100)
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
        ESP_LOGI(TAG, "Motor set to %d (pulse: %lu µs, duty: %lu)",
                 speed, pulse_us, duty);
        last_speed = speed;
    }
#endif
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
            ESP_LOGI(TAG, "Drained %d sensor items and %d command items",
                     drained_sensor, drained_command);
        }
    }
}

// Task to control the ESC/motor for testing
void esc_control_task(void *pvParameters)
{
    // Variables declared at the top to avoid redeclaration issues
    sensor_data_t sensor_data;
    flight_command_t command;
    int messages_processed;
    TickType_t drain_start_time, current_time, time_limit;
    int current_step, num_steps, last_logged_step;
    uint16_t current_throttle, target_throttle;
    uint32_t step_counter, log_counter;
    bool motor_running, is_now_running;
    int drain_counter = 0;
    UBaseType_t sensor_queue_items, command_queue_items;

    ESP_LOGI(TAG, "ESC control task started");

    // Get the synchronization semaphore
    SemaphoreHandle_t startup_sync = (SemaphoreHandle_t)pvParameters;

    // Wait for the synchronization signal from main - this should be the first task to take it
    // as it is the consumer that needs to be ready before producers start
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
            // Now we're the first to take the semaphore, so we'll hold it for a moment
            // to ensure we're ready to consume data before other tasks start producing
            vTaskDelay(200 / portTICK_PERIOD_MS);

            // Give back the semaphore to allow other tasks to proceed
            xSemaphoreGive(startup_sync);
        }
    }

    // Initial queue draining
    ESP_LOGI(TAG, "Initial queue draining...");
    drain_queues(&sensor_data, &command);

    // Brief initialization delay
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Starting ESC initialization sequence");

    // Step 1: Set all motors to min throttle
    ESP_LOGI(TAG, "Step 1: Setting minimum throttle");
    set_motor_speed(0); // Set to minimum

    // During the 3-second wait, periodically drain queues to prevent overflow
    for (int i = 0; i < 30; i++)
    { // 30 * 100ms = 3000ms
        drain_queues(&sensor_data, &command);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Step 2: Test with a working throttle value
    ESP_LOGI(TAG, "Step 2: Testing with working throttle value (200)");
    set_motor_speed(200); // 20% throttle - should start spinning

    // During the 5-second wait, periodically drain queues to prevent overflow
    for (int i = 0; i < 50; i++)
    { // 50 * 100ms = 5000ms
        drain_queues(&sensor_data, &command);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Starting test cycle");

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

    // Initialize variables
    current_step = 0;
    num_steps = sizeof(test_steps) / sizeof(test_steps[0]);
    last_logged_step = -1;
    current_throttle = 0;
    target_throttle = test_steps[0].throttle_value;
    step_counter = 0;
    log_counter = 0;
    motor_running = false;

    // Start with standard acceleration
    set_motor_acceleration(5);

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
            target_throttle = test_steps[current_step].throttle_value;
            step_counter = 0;

            // Set the acceleration rate for this step
            set_motor_acceleration(test_steps[current_step].accel_rate);

            // Log when changing steps
            ESP_LOGI(TAG, "Moving to %s (accel: %d)",
                     test_steps[current_step].description,
                     test_steps[current_step].accel_rate);

            last_logged_step = current_step;
        }

        // Smoothly approach the target throttle (change by at most the set ramp rate per iteration)
        if (current_throttle < target_throttle)
        {
            current_throttle = MIN(current_throttle + motor_ramp_rate, target_throttle);
        }
        else if (current_throttle > target_throttle)
        {
            current_throttle = MAX(current_throttle - motor_ramp_rate, target_throttle);
        }

        // Apply current throttle value
        set_motor_speed(current_throttle);

        // Log status only when step changes or every 50 iterations
        if (current_step != last_logged_step || log_counter % 50 == 0)
        {
            // Add visual indicator for throttle percentage
            char throttle_bar[21] = "[                    ]";
            int bar_length = 20;
            int fill_amount = (current_throttle * bar_length) / 1000;

            for (int i = 0; i < fill_amount; i++)
            {
                throttle_bar[i + 1] = '=';
            }

            // Add a position marker
            if (fill_amount < bar_length)
            {
                throttle_bar[fill_amount + 1] = '>';
            }

            ESP_LOGI(TAG, "%s (Speed: %d/1000, Target: %d, Accel: %d)",
                     test_steps[current_step].description,
                     current_throttle,
                     target_throttle,
                     test_steps[current_step].accel_rate);

            // Show throttle percentage with visual indicator
            ESP_LOGI(TAG, "Throttle: %d%% %s",
                     current_throttle / 10,
                     throttle_bar);

            last_logged_step = current_step;
        }

        // Track motor state for potential state change logging
        is_now_running = (current_throttle > 0);
        if (is_now_running != motor_running)
        {
            ESP_LOGI(TAG, "Motor %s", is_now_running ? "ON" : "OFF");
            motor_running = is_now_running;
        }

        // Brief delay before next iteration
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}