#include "evlog.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

static evlog_entry_t entries[EVLOG_CAPACITY];
static uint32_t total_events = 0;
static portMUX_TYPE evlog_mux = portMUX_INITIALIZER_UNLOCKED;

void evlog(const char *fmt, ...)
{
    char text[EVLOG_LINE];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    portENTER_CRITICAL(&evlog_mux);
    evlog_entry_t *entry = &entries[total_events % EVLOG_CAPACITY];
    entry->ms = now_ms;
    memcpy(entry->text, text, EVLOG_LINE);
    entry->text[EVLOG_LINE - 1] = '\0';
    total_events++;
    portEXIT_CRITICAL(&evlog_mux);
}

uint32_t evlog_snapshot(evlog_entry_t *out, uint32_t max)
{
    portENTER_CRITICAL(&evlog_mux);
    uint32_t available = total_events < EVLOG_CAPACITY ? total_events : EVLOG_CAPACITY;
    uint32_t count = available < max ? available : max;
    uint32_t first = total_events - available;
    for (uint32_t i = 0; i < count; i++)
    {
        out[i] = entries[(first + i) % EVLOG_CAPACITY];
    }
    portEXIT_CRITICAL(&evlog_mux);
    return count;
}
