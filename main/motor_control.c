#include "motor_control.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "driver/ledc.h"

static const char *TAG = "MOTOR_CONTROL";

// Add near the top of the file, after the TAG definition
#define ENABLE_MOTOR_LOGGING 0 // Set to 0 to disable motor logging

// PWM configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // 13-bit resolution, 0-8191
#define LEDC_FREQUENCY 50               // 50Hz for standard servo/ESC

// Motor output pins
#define MOTOR1_GPIO 18
#define MOTOR2_GPIO 19
#define MOTOR3_GPIO 20
#define MOTOR4_GPIO 21

// LEDC channels for each motor
#define MOTOR1_CHANNEL LEDC_CHANNEL_0
#define MOTOR2_CHANNEL LEDC_CHANNEL_1
#define MOTOR3_CHANNEL LEDC_CHANNEL_2
#define MOTOR4_CHANNEL LEDC_CHANNEL_3

// ESC calibration values
#define ESC_MIN_DUTY 410 // 1ms pulse (min throttle)
#define ESC_MAX_DUTY 820 // 2ms pulse (max throttle)

// Initialize LEDC for PWM control
static void ledc_init(void)
{
    // Timer configuration
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Channel configuration for Motor 1
    ledc_channel_config_t ledc_channel1 = {
        .channel = MOTOR1_CHANNEL,
        .duty = 0,
        .gpio_num = MOTOR1_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER};
    ledc_channel_config(&ledc_channel1);

    // Channel configuration for Motor 2
    ledc_channel_config_t ledc_channel2 = {
        .channel = MOTOR2_CHANNEL,
        .duty = 0,
        .gpio_num = MOTOR2_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER};
    ledc_channel_config(&ledc_channel2);

    // Channel configuration for Motor 3
    ledc_channel_config_t ledc_channel3 = {
        .channel = MOTOR3_CHANNEL,
        .duty = 0,
        .gpio_num = MOTOR3_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER};
    ledc_channel_config(&ledc_channel3);

    // Channel configuration for Motor 4
    ledc_channel_config_t ledc_channel4 = {
        .channel = MOTOR4_CHANNEL,
        .duty = 0,
        .gpio_num = MOTOR4_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER};
    ledc_channel_config(&ledc_channel4);
}

// Set motor speed (0-2000 range)
void set_motor_speed(uint8_t motor, uint16_t speed)
{
    // Limit speed to valid range
    if (speed > 2000)
    {
        speed = 2000;
    }

    // Map 0-2000 to ESC_MIN_DUTY-ESC_MAX_DUTY
    uint32_t duty = ESC_MIN_DUTY + (speed * (ESC_MAX_DUTY - ESC_MIN_DUTY)) / 2000;

    // Set the duty cycle
    uint8_t channel;
    switch (motor)
    {
    case 1:
        channel = MOTOR1_CHANNEL;
        break;
    case 2:
        channel = MOTOR2_CHANNEL;
        break;
    case 3:
        channel = MOTOR3_CHANNEL;
        break;
    case 4:
        channel = MOTOR4_CHANNEL;
        break;
    default:
        ESP_LOGE(TAG, "Invalid motor number: %d", motor);
        return;
    }

    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

void init_motor_control(void)
{
    // Initialize LEDC for PWM
    ledc_init();

    // Set all motors to minimum throttle (arm position)
    set_motor_speed(1, 0);
    set_motor_speed(2, 0);
    set_motor_speed(3, 0);
    set_motor_speed(4, 0);

// Then modify the logging sections
#if ENABLE_MOTOR_LOGGING
    ESP_LOGI(TAG, "Motor control initialized");
#endif
}

// Apply motor commands directly
void apply_motor_command(motor_command_t *motor_command)
{
    // Set motor speeds
    set_motor_speed(1, motor_command->motor1);
    set_motor_speed(2, motor_command->motor2);
    set_motor_speed(3, motor_command->motor3);
    set_motor_speed(4, motor_command->motor4);

// And at the end of apply_motor_command:
#if ENABLE_MOTOR_LOGGING
    ESP_LOGD(TAG, "Motors: %d, %d, %d, %d",
             motor_command->motor1, motor_command->motor2,
             motor_command->motor3, motor_command->motor4);
#endif
}