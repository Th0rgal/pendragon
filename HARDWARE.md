# Pendragon — Hardware & Wiring Reference

Living document for the physical build. Keep this in sync when wiring changes.
Last updated: 2026-07-03 (bench findings from BLE motor/ESC test sessions).

## Current status (2026-07-03, evening)

- Drone runs firmware `c9aa032` in **PWM flight mode**, verified end-to-end:
  trims applied in the mix, slow spin + hard stop clean, event log and
  telemetry streaming working. Legacy accel-P stabilization is active and
  visibly correcting (±15 units at collective 310).
- ESC spin directions configured for quad-X (TR+BL CCW, TL+BR CW), saved in
  the ESCs.
- **Thrust trims calibrated and saved**: `[TR=150, BR=50, TL=79, BL=110]`.
  They SATURATED both clamps — measured thrust spread was 4.4:1
  (TR 8.4mg vs BR 37.3mg median tilt), beyond what trim can equalize.
  TOP RIGHT inspection is mandatory before hover; recalibrate after any
  prop/motor work (`uv run tools/esc_tool.py calibrate` in DShot mode).
- Bench caution: with these trims, collective ≥400 kicks briefly rocked the
  frame off the ground (gyro >100dps transient). Keep bench tests ≤350.
- **Blocked on hardware**: (1) undersized 5V buck causes brownout resets under
  BLE+flash load on battery-only power — must be fixed before flight (USB
  power avoids it on the bench); (2) props need rearranging and the TOP RIGHT
  corner needs inspection; (3) no battery voltage sensing yet.
- Next software milestone: IMU→frame axis mapping (needs a cleaner dedicated
  probe run — calibration dax/day was too noisy except BR), then stabilization
  (attitude estimation → rate/angle PID) for a stable hover just above ground.

## Core components

| Component | Part | Notes |
|---|---|---|
| MCU | ESP32-S3 (N18R8 module) | 4 RMT TX channels total — see constraints below |
| IMU | ICM-42688-P (SPI) | accel + gyro + temp |
| ESC | UAngel 4-in-1, 45A (60A burst), BLHeli_S, 3-6S | supports DShot150/300; direction configurable via DShot commands |
| Status LED | WS2812 (onboard, GPIO 48) | driven by `led_strip` RMT backend |
| Battery | ~15V on ESC "V" wire → likely 4S LiPo | exact pack spec TODO; no voltage sensing wired (see Battery section) |
| Power | ESC "V" (15V) → buck converter → 5V → ESP32 5V0 | do NOT feed 15V to the ESP32 directly |

## ESC harness ↔ ESP32 wiring

| ESC wire | Color | Function | ESP32 GPIO |
|---|---|---|---|
| GND | Black | Common ground | GND |
| V | Red | ~15V from ESC (battery voltage) | → buck → 5V0 (not a signal) |
| 1 | White | Motor TOP RIGHT signal | GPIO 5 |
| 2 | Brown | Motor BOTTOM RIGHT signal | GPIO 13 |
| 3 | Orange | Motor TOP LEFT signal | GPIO 18 |
| 4 | Yellow | Motor BOTTOM LEFT signal | GPIO 17 |
| C | — | Not connected (likely current-sense output — worth wiring to an ADC pin for coulomb counting) | — |

Motor order in firmware (`motor_id_t`): TR=0, BR=1, TL=2, BL=3.

## IMU (ICM-42688-P) wiring — SPI2/FSPI

| Signal | ESP32 GPIO |
|---|---|
| SCLK | 21 |
| MOSI | 36 |
| MISO | 37 |
| CS | 9 |

**Orientation finding (2026-07-03):** at rest on a flat surface the IMU reads
`az ≈ -0.97g` (ax ≈ -0.06, ay ≈ +0.11) — the Z axis points *down* relative to
the frame, and there is a small constant mounting tilt / accel bias.
Stabilization code must account for both.

## Motor signal protocols

- **Normal flight mode:** 50Hz servo PWM via LEDC, 13-bit, 1000-2000µs pulse
  (0 collective = no pulse at all, which hard-silences the ESCs).
- **ESC config mode:** DShot150 via the RMT peripheral (infinite hardware loop,
  ~1kHz frame rate, zero CPU). Selected by the NVS `motor_mode` flag
  (namespace `pendragon`), toggled over BLE opcode `0xD0` + reboot.
- The ESC detects its input protocol **only at ESC power-up** — after switching
  modes the battery must be unplugged/replugged, an ESP32 reboot is not enough.

### Hardware constraint: RMT channels

The ESP32-S3 has exactly **4 RMT TX channels**. DShot mode uses all four for
the motors, so the WS2812 status LED (also RMT-driven) cannot be initialized in
that mode — the firmware skips the LED task there (`ESP_ERROR_CHECK` inside
`led_strip` would otherwise abort → boot loop; this happened, fixed 2026-07-03).
**A dark status LED = DShot config mode.**

## Bench findings (2026-07-03)

- **ESC direction mapping (measured, all four channels): `normal` = CW,
  `reversed` = CCW** (viewed from above). Set via BLHeli_S DShot commands
  (`0xD1` spin-direction + save; persists in the ESC across power cycles).
- **Current ESC configuration** (standard quad-X): TR + BL `reversed` (CCW),
  TL + BR `normal` (CW).
- **Prop types as mounted (measured via differential-thrust probe)**:
  BR = CCW prop (strong signal, 40mg tilt), TL = CW, BL = CW (clear, ~30mg),
  **TR = indeterminate** — its thrust response is weak and erratic in both
  directions (13-17mg), suggesting a loose prop nut, damaged blade, or mount
  issue on that corner. Inspect physically.
- **Props need rearranging for flight**: diagonals must match. If TR's prop
  is CCW, swap the BR and BL props → TR+BL carry CCW props, TL+BR carry CW,
  matching the ESC configuration above. If TR's prop turns out to be CW, the
  set is 3×CW + 1×CCW and a second CCW prop is needed.
- **Direction probing method** (opcode `0xD3`): pulses one motor while
  integrating gyro Z (reaction/drag torque; chip +Z down so CCW motor => +gz)
  and averaging accel deltas (upward thrust unloads the corner => tilt;
  wrong-direction thrust is blocked by the ground). Yaw signal is weak under
  ground friction (~±0.3-0.6deg); thrust tilt is the stronger discriminator
  (up to 4x ratio between settings). Repeat probes and use medians.
- **Auto-power safeguard**: the auto-generated test command task was removed
  from firmware — motors can only move on explicit BLE commands. ESC startup/
  arming beeps still twitch the props at battery-on; that is the ESC itself,
  not throttle.
- **Brownout under load (confirmed, NOT battery charge)**: even on a full pack
  (>4.0V/cell), BLE TX + flash writes (OTA) trip the ESP32 brownout reset
  (`reset=9` in info telemetry). Every USB-powered flash succeeded; battery-only
  OTAs mostly brown out, and pacing the transfer doesn't help → the 15V→5V
  buck is undersized for peak loads. Symptoms: random BLE disconnects, boot
  loops at battery-on with repeated ESC beeps (props twitch). **Fix needed
  before flight**: bigger buck (≥2A) and/or 470-1000µF electrolytic on the 5V
  output. Until then: plug USB for firmware updates. Check `reset=` (0xB1)
  whenever behavior is strange.
- **PWM spin-up threshold:** from standstill, motors need collective ~350-400
  (of 1000) to start; once spinning they sustain down to ~310. At ≤280 they do
  not start at all. (The `WORKING_THROTTLE_VALUE 200` comment in
  `motor_control.c` is optimistic.)
- **DShot test throttle** is firmware-capped at raw 500/2047 (~23%) for bench
  safety; direction checks are done around raw 200 (~8%).
- **BLE OTA:** works end-to-end (~605KB in ~3min, 509-byte chunks with
  write-with-response, ~3.4 KB/s). Version/partition verifiable via `0xB1` info.
- **BLE disconnect failsafe:** collective → 0 (PWM mode) / throttle → 0 (DShot
  mode). Verified in code, not yet exercised deliberately.

## BLE protocol quick reference

Service `ffeeddcc-bbaa-9988-7766-554433221100`, command char (write)
`00ffeedd-ccbb-aa99-8877-665544332211`, telemetry char (notify)
`11223344-5566-7788-99aa-bbccddeeff00`. Opcodes in `main/ble_protocol.h`:

| Opcode | Payload | Action |
|---|---|---|
| `0xA0`/`0xA1` | `[step]` | collective power up/down (PWM mode only) |
| `0xB0` | — | ping → "pong" |
| `0xB1` | — | firmware/partition/heap info |
| `0xB2` | — | IMU snapshot (accel g, gyro °/s, temp) |
| `0xB3` | — | motor driver status (per-mode) |
| `0xC0..0xC3` | see code | OTA begin/data/end/abort |
| `0xD0` | `[0\|1]` | motor mode pwm/dshot-config → NVS + reboot |
| `0xD1` | `[mask, 0\|1]` | ESC spin direction normal/reversed + save (DShot mode) |
| `0xD2` | `[lo, hi]` | raw DShot test throttle 0 / 48-2047 (DShot mode) |
| `0xD3` | `[motor, lo, hi]` | direction probe: pulse motor, report gz/accel deltas |
| `0xD4` | `[tr, br, tl, bl]` | per-motor thrust trim % (50-150, 0/0xFF keep; empty = report) |

Client tooling: `tools/esc_tool.py` (Python/bleak, run with `uv run`) — OTA,
motor tests with IMU monitoring, direction config/probes, trims, telemetry
streaming, event log. See its docstring for all commands.

## Battery monitoring — current status & plan

**There is currently no way for the firmware to read battery voltage.** No ADC
pin is wired to the pack, the BLHeli_S ESC has no telemetry output wire
(that's a BLHeli_32 feature), and the harness "C" wire (probably current sense)
is unconnected.

Until sensing exists, protect the pack externally: a balance-lead LiPo alarm
(buzzer at 3.5V/cell) is the simplest guard against over-discharge.

Planned mod (small):
1. Voltage divider from the ESC "V" wire (= battery voltage, ~15-16.8V max on
   4S) to a free ADC1 GPIO (GPIO 1-10 are ADC1 on the S3; 5/9 are taken —
   suggest GPIO 4). Example: 68kΩ : 10kΩ → 16.8V ÷ 7.8 ≈ 2.15V, safe for
   ADC with 12dB attenuation. Add a 100nF cap across the bottom resistor.
2. Firmware: calibrated ADC sampling, exponential smoothing, telemetry opcode +
   periodic notify, low-voltage warning and auto-power-down failsafe.
3. SoC estimate from the LiPo discharge curve (rough guide, per cell, under
   light load): 4.20V=100%, 3.85V≈60%, 3.75V≈40%, 3.65V≈20%, 3.50V≈5%,
   **3.30V = land now**. Voltage sag under load skews this — with the "C"
   current-sense wire on a second ADC pin, coulomb counting would be far more
   accurate.

## Flight status / next steps

- [x] ESC direction mapping measured; channels set to quad-X (TR+BL CCW, TL+BR CW)
- [ ] Inspect TOP RIGHT corner (weak/erratic thrust response — check prop nut,
      blade, mount) and identify its prop type (compare helix against BR's).
      Owner reports one motor is indeed weaker (possibly damaged) — per-motor
      thrust trim (opcode 0xD4, NVS-persisted, applied in the PWM flight mix)
      exists to compensate; calibrate it from thrust-probe data once props
      are finalized.
- [ ] Rearrange props to match ESC config (likely: swap BR and BL props)
- [ ] Re-run per-motor thrust probe after prop work: every motor should show
      a strong tilt (~30-40mg) at its configured direction
- [ ] Switch `motor_mode` back to PWM (0xD0) + battery cycle for flight mode
      (or better: adopt DShot as the flight protocol — faster, digital, no
      calibration; needs the DShot driver promoted out of config-only mode)
- [ ] Battery voltage sensing mod (see above)
- [ ] Stabilization: attitude estimation (gyro+accel fusion) + angle/rate PID +
      motor mixing — goal: stable hover just above ground. NOT flight-ready yet;
      current `esc_control_task` only has a crude accel-only P correction.
