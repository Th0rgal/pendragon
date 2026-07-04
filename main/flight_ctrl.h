#ifndef FLIGHT_CTRL_H
#define FLIGHT_CTRL_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Minimal hover stabilization for DShot config mode: complementary-filter
// attitude estimation + angle-P / rate-D control on roll and pitch, mixed
// into per-motor DShot throttle with trims. Yaw is left uncontrolled (the
// 2+2 prop set torque-balances). Designed for the "barely levitate" test.
//
// IMU->frame mapping (calibrated 2026-07-04, see HARDWARE.md):
//   chip X ~ forward, chip Y ~ right, chip Z = down (~10deg yaw skew ignored)
//   +gx = roll right-down rate, +gy = nose-up rate
//   level reference = resting accel bias, captured at arm time.

// Start the 100Hz flight control task (created in DShot mode; idles when
// disarmed). Motors move only when armed AND dshot output is enabled.
esp_err_t flight_ctrl_start(void);

// Arm: requires stillness (gyro quiet) to capture bias; starts at collective
// 0. Disarm: cuts motors to 0 immediately.
esp_err_t flight_ctrl_arm(bool armed);
bool flight_ctrl_armed(void);

// Slew-limited base collective target (raw dshot 0..FLIGHT_MAX_COLLECTIVE).
esp_err_t flight_ctrl_set_collective(uint16_t target);

// Gains scaled: kp = kp_x10/10 (dshot units per degree),
// kd = kd_x100/100 (dshot units per deg/s).
void flight_ctrl_set_gains(uint8_t kp_x10, uint8_t kd_x100);

// Current estimate for telemetry (degrees).
void flight_ctrl_get_attitude(float *roll, float *pitch);

void flight_ctrl_get_debug(char *buffer, int buffer_len);

#endif // FLIGHT_CTRL_H
