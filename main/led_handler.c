#include "led_handler.h"
#include "esp_log.h"
#include "freertos/semphr.h"

static const char *TAG = "LED_HANDLER";

// Configuration for WS2812/NeoPixel style addressable LED
#define LED_GPIO_PIN 48 // GPIO for the integrated LED
#define LED_NUM_PIXELS 1

// Set to 1 to enable detailed LED debugging, 0 to disable
#define LED_DEBUG_LOGGING 0

// Lighter blue color (more white)
#define DEFAULT_RED 10   // Add some red to make it more white
#define DEFAULT_GREEN 15 // More green
#define DEFAULT_BLUE 100 // Increased blue

// LED strip control object
static led_strip_handle_t led_strip = NULL;

// Semaphore for thread-safe access
static SemaphoreHandle_t led_semaphore = NULL;

esp_err_t init_led(void)
{
#if LED_DEBUG_LOGGING
    ESP_LOGI(TAG, "Initializing addressable LED on GPIO%d", LED_GPIO_PIN);
#endif

    // Create semaphore for thread safety
    led_semaphore = xSemaphoreCreateMutex();
    if (led_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create LED semaphore");
        return ESP_FAIL;
    }

    // Configure LED strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_PIN,
        .max_leds = LED_NUM_PIXELS,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // Set the light blue color
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, DEFAULT_RED, DEFAULT_GREEN, DEFAULT_BLUE));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

#if LED_DEBUG_LOGGING
    ESP_LOGI(TAG, "LED initialized with lighter blue color (R:%d, G:%d, B:%d)",
             DEFAULT_RED, DEFAULT_GREEN, DEFAULT_BLUE);
#endif
    return ESP_OK;
}

esp_err_t set_led_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
{
    if (led_strip == NULL)
    {
        ESP_LOGE(TAG, "LED strip not initialized");
        return ESP_FAIL;
    }

    // Calculate brightness-adjusted values
    uint8_t r = (red * brightness) / 100;
    uint8_t g = (green * brightness) / 100;
    uint8_t b = (blue * brightness) / 100;

#if LED_DEBUG_LOGGING
    ESP_LOGI(TAG, "Setting LED color: RGB(%u,%u,%u) at %u%% brightness", red, green, blue, brightness);
#endif

    if (xSemaphoreTake(led_semaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Failed to acquire LED semaphore");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = led_strip_set_pixel(led_strip, 0, r, g, b);
    if (ret == ESP_OK)
    {
        ret = led_strip_refresh(led_strip);
    }
#if LED_DEBUG_LOGGING
    else
    {
        ESP_LOGW(TAG, "Failed to set LED color: %s", esp_err_to_name(ret));
    }
#endif

    xSemaphoreGive(led_semaphore);
    return ret;
}

void led_handler_task(void *pvParameters)
{
#if LED_DEBUG_LOGGING
    ESP_LOGI(TAG, "LED handler task started");
#endif

    // Initialize the LED
    if (init_led() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LED");
        vTaskDelete(NULL);
        return;
    }

    // Task loop - periodically refresh the LED to maintain color
    while (1)
    {
        // Refresh the light blue color every 5 seconds
        set_led_color(DEFAULT_RED, DEFAULT_GREEN, DEFAULT_BLUE, 25);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}