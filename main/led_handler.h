#ifndef LED_HANDLER_H
#define LED_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "led_strip.h"

/**
 * @brief Initialize the addressable RGB LED on GPIO38
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t init_led(void);

/**
 * @brief Set the LED to a specific color and brightness
 *
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @param brightness Brightness percentage (0-100)
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t set_led_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);

/**
 * @brief Task to handle LED effects
 *
 * @param pvParameters Task parameters (not used)
 */
void led_handler_task(void *pvParameters);

#endif // LED_HANDLER_H