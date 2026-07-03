#ifndef BLE_PROTOCOL_H
#define BLE_PROTOCOL_H

#include <stdint.h>

#define PENDRAGON_BLE_DEVICE_NAME "Pendragon"

// 128-bit UUID byte order used by NimBLE's ble_uuid128_t.value field.
#define PENDRAGON_SERVICE_UUID_BYTES \
    { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, \
      0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff }

#define PENDRAGON_COMMAND_CHARACTERISTIC_UUID_BYTES \
    { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, \
      0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00 }

#define PENDRAGON_TELEMETRY_CHARACTERISTIC_UUID_BYTES \
    { 0x00, 0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, \
      0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11 }

typedef enum
{
    PENDRAGON_BLE_CMD_POWER_UP = 0xA0,
    PENDRAGON_BLE_CMD_POWER_DOWN = 0xA1,
    PENDRAGON_BLE_CMD_PING = 0xB0,
    PENDRAGON_BLE_CMD_INFO = 0xB1,
    PENDRAGON_BLE_CMD_SENSOR_SNAPSHOT = 0xB2,
    PENDRAGON_BLE_CMD_MOTOR_STATUS = 0xB3,
    // Continuous IMU + motor-output telemetry: [hz (1-20), seconds (1-60)].
    // hz=0 stops an active stream.
    PENDRAGON_BLE_CMD_TELEMETRY_STREAM = 0xB4,
    // Dump the in-RAM event log (boot/BLE/motor commands/failsafes).
    PENDRAGON_BLE_CMD_EVENT_LOG = 0xB5,
    PENDRAGON_BLE_CMD_OTA_BEGIN = 0xC0,
    PENDRAGON_BLE_CMD_OTA_DATA = 0xC1,
    PENDRAGON_BLE_CMD_OTA_END = 0xC2,
    PENDRAGON_BLE_CMD_OTA_ABORT = 0xC3,
    // Report OTA session state so a client can resume after a disconnect
    // (session survives disconnects; a reboot loses it).
    PENDRAGON_BLE_CMD_OTA_STATUS = 0xC4,
    // ESC configuration (DShot). MOTOR_MODE persists to NVS and reboots;
    // the other two only work when booted in DShot config mode.
    PENDRAGON_BLE_CMD_MOTOR_MODE = 0xD0,        // [mode: 0=pwm, 1=dshot-config]
    PENDRAGON_BLE_CMD_ESC_DIRECTION = 0xD1,      // [motor mask, 0=normal 1=reversed]
    PENDRAGON_BLE_CMD_ESC_TEST_THROTTLE = 0xD2,  // [lo, hi] raw dshot 0|48..2047
    // Pulse one motor and integrate gyro Z reaction torque to detect its true
    // spin direction (chip +Z is down: CCW seen from above => positive gz).
    PENDRAGON_BLE_CMD_ESC_DIRECTION_PROBE = 0xD3, // [motor, thr_lo, thr_hi]
    // Per-motor thrust trim percent (50-150, 0/0xFF = keep). Empty payload
    // reports current trims. Persisted in NVS, applied in PWM flight mode.
    PENDRAGON_BLE_CMD_MOTOR_TRIM = 0xD4, // [tr, br, tl, bl]
    // Send a raw DShot command (1-47, e.g. 1-5 = beacon beeps) to one motor.
    PENDRAGON_BLE_CMD_ESC_RAW_CMD = 0xD5,      // [motor, cmd]
    // Set one motor's raw DShot throttle (0 stop, 48..cap). DShot mode only.
    PENDRAGON_BLE_CMD_ESC_MOTOR_THROTTLE = 0xD6 // [motor, lo, hi]
} pendragon_ble_opcode_t;

#define PENDRAGON_BLE_OPCODE_MIN 0xA0
#define PENDRAGON_BLE_DEFAULT_POWER_STEP 50
#define PENDRAGON_BLE_OTA_DATA_HEADER_LEN 3
#define PENDRAGON_BLE_OTA_BEGIN_LEN 5

#endif // BLE_PROTOCOL_H
