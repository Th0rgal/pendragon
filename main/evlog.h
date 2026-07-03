#ifndef EVLOG_H
#define EVLOG_H

#include <stdint.h>

// Tiny RAM ring buffer of recent firmware events (boot, BLE, motor commands,
// failsafes) for remote debugging over BLE. Survives nothing — RAM only —
// but the boot entry records the reset reason, so crash/brownout loops are
// visible after the fact.

#define EVLOG_CAPACITY 32
#define EVLOG_LINE 72

typedef struct
{
    uint32_t ms; // uptime when logged
    char text[EVLOG_LINE];
} evlog_entry_t;

void evlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Copy up to `max` entries, oldest first. Returns number copied.
uint32_t evlog_snapshot(evlog_entry_t *out, uint32_t max);

#endif // EVLOG_H
