#include "motor_control.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include "command_handler.h"  // For flight_command_t
#include "icm42688p_sensor.h" // For IMU data
#include "motor_mapping.h"    // For readable influence mapping

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

// Motor pins for each motor (PWM outputs)
// ESC harness (colors):
//   - GND:   Black  → ESP32 GND (common ground)
//   - V:     Red    → 15V from ESC (DO NOT FEED ESP32 DIRECTLY). Use a buck to 5V → ESP32 5V0.
//   - 1:     White  → Motor TOP_RIGHT signal → GPIO 5
//   - 2:     Brown  → Motor BOTTOM_RIGHT signal → GPIO 13
//   - 3:     Orange → Motor TOP_LEFT signal → GPIO 18
//   - 4:     Yellow → Motor BOTTOM_LEFT signal → GPIO 17
//   - C/NC:  Not connected
const uint8_t MOTOR_PINS[MOTOR_COUNT] = {
    5,  // MOTOR_TOP_RIGHT (GPIO 5)  - ESC wire 1 (White)
    13, // MOTOR_BOTTOM_RIGHT (GPIO 13) - ESC wire 2 (Brown)
    18, // MOTOR_TOP_LEFT (GPIO 18) - ESC wire 3 (Orange)
    17  // MOTOR_BOTTOM_LEFT (GPIO 17) - ESC wire 4 (Yellow)
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
static bool stabilization_enabled = true;
static uint16_t commanded_collective = 0; // 0..1000 collective power target
static portMUX_TYPE motor_state_mux = portMUX_INITIALIZER_UNLOCKED;

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
#if ENABLE_LOGGING
    float full_ramp_time = (1000.0f / ramp_rate) * 0.1f; // in seconds
    ESP_LOGI(TAG, "Full throttle ramp will take approximately %.1f seconds", full_ramp_time);
#endif
}

void set_stabilization_enabled(bool enabled)
{
    stabilization_enabled = enabled;
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
        .clk_cfg = LEDC_USE_APB_CLK};
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
            set_stabilization_enabled(received_command.stabilize);
            uint16_t new_collective = system_armed ? (uint16_t)(received_command.throttle * 1000.0f) : 0;
            portENTER_CRITICAL(&motor_state_mux);
            commanded_collective = MIN(MAX(new_collective, 0), 1000);
            portEXIT_CRITICAL(&motor_state_mux);

#if ENABLE_LOGGING
            ESP_LOGD(TAG, "Command Rx: Throttle=%.2f, Arm=%d -> Collective=%u, Stabilize=%d",
                     received_command.throttle, received_command.arm_status, commanded_collective, stabilization_enabled);
#endif
        }

        // Read IMU once per loop if needed
        icm42688p_data_t imu;
        bool imu_ok = false;
        portENTER_CRITICAL(&motor_state_mux);
        uint16_t collective_snapshot = commanded_collective;
        portEXIT_CRITICAL(&motor_state_mux);
        bool active = (collective_snapshot > 0);
        if (stabilization_enabled && active)
        {
            if (icm42688p_read_data(&imu) == ESP_OK)
            {
                imu_ok = true;
            }
        }

        // Build per-motor target from collective and stabilization
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            target_throttles[i] = collective_snapshot;
        }

        if (imu_ok)
        {
            // Desired level: accel_x≈0, accel_y≈0. Error = measured.
            float roll_error_g = imu.accel_y;  // crude roll proxy
            float pitch_error_g = imu.accel_x; // crude pitch proxy
            float yaw_error = 0.0f;            // not used without gyro yaw control

            const float kP_roll = 50.0f;  // PWM units per g
            const float kP_pitch = 50.0f; // PWM units per g
            const float kP_yaw = 0.0f;    // disabled for now

            for (int m = 0; m < MOTOR_COUNT; m++)
            {
                float corr = 0.0f;
                corr += -kP_roll * roll_error_g * DEFAULT_MOTOR_INFLUENCE[m].roll;
                corr += -kP_pitch * pitch_error_g * DEFAULT_MOTOR_INFLUENCE[m].pitch;
                corr += -kP_yaw * yaw_error * DEFAULT_MOTOR_INFLUENCE[m].yaw;
                int32_t tgt = (int32_t)target_throttles[m] + (int32_t)corr;
                target_throttles[m] = (uint16_t)MIN(MAX(tgt, 0), 1000);
            }
        }

        // Ramp current speeds toward targets
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            uint16_t ramp_change_this_tick = motor_ramp_rates[i];
            if (current_motor_speeds[i] < target_throttles[i])
            {
                current_motor_speeds[i] = MIN(current_motor_speeds[i] + ramp_change_this_tick, target_throttles[i]);
            }
            else if (current_motor_speeds[i] > target_throttles[i])
            {
                current_motor_speeds[i] = MAX(current_motor_speeds[i] - ramp_change_this_tick, target_throttles[i]);
            }
        }

        // Write PWM
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            set_motor_speed(i, current_motor_speeds[i]);
        }

        // Delay for the motor control loop period
        vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
    }
}

// ===== New public entry points used by BLE opcodes =====
void motor_adjust_power(int16_t delta_step_0_to_1000)
{
    portENTER_CRITICAL(&motor_state_mux);
    int32_t updated = (int32_t)commanded_collective + (int32_t)delta_step_0_to_1000;
    if (updated < 0)
        updated = 0;
    if (updated > 1000)
        updated = 1000;
    commanded_collective = (uint16_t)updated;
#if ENABLE_LOGGING
    uint16_t collective_snapshot = commanded_collective;
#endif
    portEXIT_CRITICAL(&motor_state_mux);
#if ENABLE_LOGGING
    ESP_LOGI(TAG, "collective updated to %u via BLE adjust", collective_snapshot);
#endif
}
