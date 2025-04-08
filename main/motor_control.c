#include "motor_control.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/task.h"

static const char *TAG = "MOTOR_CONTROL";

// Constants for motor control
#define ENABLE_LOGGING 1       // Set to 0 to disable logging

// PWM Configuration
#define PWM_FREQUENCY 50       // 50Hz for standard ESCs
#define PWM_RESOLUTION LEDC_TIMER_13_BIT  // 13-bit resolution (0-8191)
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE

// Motor pins and channels
#define MAX_MOTORS 4
static const uint8_t MOTOR_PINS[MAX_MOTORS] = {
    10,  // Motor 1 - GPIO10
    18,  // Motor 2 - GPIO18
    19,  // Motor 3 - GPIO19 
    20   // Motor 4 - GPIO20
};
static const uint8_t MOTOR_CHANNELS[MAX_MOTORS] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3
};

// ESC signal calibration
#define MIN_THROTTLE_PULSE_US 1000  // 1ms pulse (minimum throttle)
#define MAX_THROTTLE_PULSE_US 2000  // 2ms pulse (maximum throttle)
#define NEUTRAL_THROTTLE_VALUE 0    // Input throttle value for neutral/off (maps to MIN_THROTTLE_PULSE_US)
#define WORKING_THROTTLE_VALUE 200  // Input throttle value where motor starts working (maps to ~1200 µs)

// Calculate PWM duty cycle from pulse width in microseconds
static uint32_t pulse_us_to_duty(uint32_t pulse_us) {
    // For 50Hz, period is 20,000 µs. Map pulse_us (1000-2000) to duty cycle
    return (pulse_us * ((1 << PWM_RESOLUTION) - 1)) / 20000;
}

// Initialize motor PWM channels
void init_motors(void) {
    ESP_LOGI(TAG, "Initializing motor control");
    
    // Configure timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    // Configure channels for each motor
    for (int i = 0; i < MAX_MOTORS; i++) {
        ledc_channel_config_t channel_conf = {
            .channel = MOTOR_CHANNELS[i],
            .duty = pulse_us_to_duty(MIN_THROTTLE_PULSE_US),  // Start at minimum throttle
            .gpio_num = MOTOR_PINS[i],
            .speed_mode = PWM_MODE,
            .hpoint = 0,
            .timer_sel = PWM_TIMER
        };
        ledc_channel_config(&channel_conf);
        
        ESP_LOGI(TAG, "Motor %d initialized on GPIO%d (channel %d)", 
                i+1, MOTOR_PINS[i], MOTOR_CHANNELS[i]);
    }
}

// Set speed for a specific motor (0-1000 input range)
void set_motor_speed(uint8_t motor_num, uint16_t speed) {
    // Check if motor number is valid
    if (motor_num < 1 || motor_num > MAX_MOTORS) {
        ESP_LOGE(TAG, "Invalid motor number: %d", motor_num);
        return;
    }
    
    // Limit speed to 0-1000 range
    if (speed > 1000) {
        speed = 1000;
    }
    
    // Map 0-1000 to ESC range (1000-2000µs)
    uint32_t pulse_us;
    if (speed == 0) {
        pulse_us = MIN_THROTTLE_PULSE_US;  // Min throttle/off
    } else {
        pulse_us = MIN_THROTTLE_PULSE_US + speed;  // 1000 + speed → 1000-2000µs range
    }
    
    // Convert to duty cycle and apply
    uint32_t duty = pulse_us_to_duty(pulse_us);
    ledc_set_duty(PWM_MODE, MOTOR_CHANNELS[motor_num-1], duty);
    ledc_update_duty(PWM_MODE, MOTOR_CHANNELS[motor_num-1]);
    
    if (ENABLE_LOGGING) {
        ESP_LOGI(TAG, "Motor %d set to %d (pulse: %lu µs, duty: %lu)", 
                motor_num, speed, pulse_us, duty);
    }
}

// Set all motor speeds at once
void set_all_motors(motor_command_t *cmd) {
    set_motor_speed(1, cmd->motor1);
    set_motor_speed(2, cmd->motor2);
    set_motor_speed(3, cmd->motor3);
    set_motor_speed(4, cmd->motor4);
}

// Task to control the ESC/motor for testing
void esc_control_task(void *pvParameters) {
    ESP_LOGI(TAG, "ESC control task started");
    
    // Wait for everything to initialize
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // ESC initialization sequence
    ESP_LOGI(TAG, "Starting ESC initialization sequence");
    
    // Step 1: Send min throttle (required for ESC arming)
    ESP_LOGI(TAG, "Step 1: Setting minimum throttle");
    set_motor_speed(1, 0);  // Maps to 1000µs
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // Step 2: Test with known working value
    ESP_LOGI(TAG, "Step 2: Testing with working throttle value (%d)", WORKING_THROTTLE_VALUE);
    set_motor_speed(1, WORKING_THROTTLE_VALUE);  // Maps to 1200µs
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    // Step 3: Run a simple test cycle
    ESP_LOGI(TAG, "Starting test cycle");
    
    // Test values to try
    const uint16_t test_values[] = {0, 200, 250, 300, 200, 0};
    const char* descriptions[] = {
        "Off",
        "Threshold", 
        "Low speed", 
        "Medium speed",
        "Back to threshold",
        "Off again"
    };
    
    // Run through test values
    for (int i = 0; i < sizeof(test_values)/sizeof(test_values[0]); i++) {
        ESP_LOGI(TAG, "Test %d: %s (value: %u)", i+1, descriptions[i], test_values[i]);
        set_motor_speed(1, test_values[i]);
        vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
    
    // After tests, enter main control loop
    ESP_LOGI(TAG, "Tests completed, entering main control loop");
    
    // Two states: on and off, alternating
    bool motor_on = false;
    
    while (1) {
        motor_on = !motor_on;
        
        if (motor_on) {
            ESP_LOGI(TAG, "Motor ON (value: %d)", WORKING_THROTTLE_VALUE);
            set_motor_speed(1, WORKING_THROTTLE_VALUE);
        } else {
            ESP_LOGI(TAG, "Motor OFF");
            set_motor_speed(1, 0);
        }
        
        // Wait before switching state
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}