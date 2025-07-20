#include "motor_control.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "command_handler.h" // For flight_command_t

// Queue handles declared in main.c
extern QueueHandle_t command_queue;

// Utility macros
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

// Define a default acceleration rate
#define DEFAULT_MOTOR_ACCELERATION 25 // Units (0-1000 range) per 100ms. 25 gives a 4-second 0-1000 ramp.
#define MOTOR_CONTROL_PERIOD_MS 100   // Loop period for motor control logic

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
    5,  // MOTOR_TOP_RIGHT (GPIO 10)
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

// Task to control the ESCs/motors based on commands
void esc_control_task(void *pvParameters)
{
    flight_command_t received_command;
    uint16_t target_throttles[MOTOR_COUNT];
    static bool system_armed = false;

#if ENABLE_LOGGING
    ESP_LOGI(TAG, "ESC control task started");
#endif

    // Initialize target throttles to 0
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        target_throttles[i] = 0;
        current_motor_speeds[i] = 0; // Ensure current speeds also start at 0
    }

    // Get the synchronization semaphore
    SemaphoreHandle_t startup_sync = (SemaphoreHandle_t)pvParameters;

    // Wait for the synchronization signal from main
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
            vTaskDelay(pdMS_TO_TICKS(200)); // Brief delay
            xSemaphoreGive(startup_sync);
        }
    }

    // Initialize motors
    init_motors(); // This sets PWM to min throttle (0 speed)

    // Set all motors to 0 speed explicitly after init, just in case.
    set_motor_speed(MOTOR_ALL, 0);

    // Set default acceleration for all motors
    set_motor_acceleration(MOTOR_ALL, DEFAULT_MOTOR_ACCELERATION);

#if ENABLE_LOGGING
    ESP_LOGI(TAG, "Motor control initialized. Listening for commands.");
#endif

    // Main control loop
    while (1)
    {
        // Check for new commands from the command_queue (non-blocking)
        if (xQueueReceive(command_queue, &received_command, 0) == pdTRUE)
        {
            system_armed = received_command.arm_status;
            uint16_t new_target_throttle_val;

            if (system_armed)
            {
                new_target_throttle_val = (uint16_t)(received_command.throttle * 1000.0f);
            }
            else
            {
                new_target_throttle_val = 0;
            }

            new_target_throttle_val = MIN(MAX(new_target_throttle_val, 0), 1000); // Clamp to 0-1000

            for (int i = 0; i < MOTOR_COUNT; i++)
            {
                target_throttles[i] = new_target_throttle_val;
            }

#if ENABLE_LOGGING
            ESP_LOGD(TAG, "Command Rx: Throttle=%.2f, Arm=%d -> Target Speed=%u, System Armed=%d",
                     received_command.throttle, received_command.arm_status, new_target_throttle_val, system_armed);
#endif
        }

        // Ramping logic: Smoothly approach the target throttle for each motor
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            uint16_t ramp_change_this_tick = motor_ramp_rates[i]; // motor_ramp_rates is units per 100ms

            if (current_motor_speeds[i] < target_throttles[i])
            {
                current_motor_speeds[i] = MIN(current_motor_speeds[i] + ramp_change_this_tick, target_throttles[i]);
            }
            else if (current_motor_speeds[i] > target_throttles[i])
            {
                current_motor_speeds[i] = MAX(current_motor_speeds[i] - ramp_change_this_tick, target_throttles[i]);
            }
            set_motor_speed(i, current_motor_speeds[i]); // Apply the new speed to the motor
        }

        // Drain sensor data queue (as we are not using sensor data in this task currently)
        /* // Removed sensor data queue drain
        while (xQueueReceive(sensor_data_queue, &temp_sensor_data, 0) == pdTRUE)
        {
            // Discard data
        }
        */

        // Logging (throttled)
        static int log_counter = 0;
        if (++log_counter % (1000 / MOTOR_CONTROL_PERIOD_MS) == 0) // Log every 1 second
        {
#if ENABLE_LOGGING
            ESP_LOGI(TAG, "Motor Speeds: TR:%3u, BR:%3u, TL:%3u, BL:%3u (Targets: All %3u)",
                     current_motor_speeds[MOTOR_TOP_RIGHT], current_motor_speeds[MOTOR_BOTTOM_RIGHT],
                     current_motor_speeds[MOTOR_TOP_LEFT], current_motor_speeds[MOTOR_BOTTOM_LEFT],
                     target_throttles[0]); // Assuming collective throttle
#endif
        }

        // Delay for the motor control loop period
        vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
    }
}