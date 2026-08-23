# Pendragon — Hardware & Wiring Reference

Living document for the physical build. Keep this in sync when wiring changes.
Last updated: 2026-08-23 (motor map + spin directions confirmed).

## Current status (2026-08-23): four corners mapped; X-quad spins set

Viewed from above, XT60 end = nose, lettering A at top-right then clockwise.

| Letter | Corner | Firmware | GPIO | Signal | Spin (from above) | How |
|---|---|---|---|---|---|---|
| A | Top right (front-right) | 0 TR | 5 | White | CW | ESC as-is |
| B | Bottom right (rear-right) | 1 BR | 13 | Brown | CCW | Two motor phase wires swapped (left↔right, middle kept) |
| C | Bottom left (rear-left) | 3 BL | 17 | Yellow | CW | ESC as-is |
| D | Top left (front-left) | 2 TL | 18 | Orange | CCW | ESC as-is |

Diagonals: A+C CW, B+D CCW. That is the target X-quad. Isolated DShot pulses (raw 400, unused lines silent) confirmed each corner; IMU moved; other `m[]` channels stayed 0.

### What was wrong, and what we do not trust

- **Software DShot reverse does not flip this UAngel BLHeli_S 4-in-1.** Isolated cmd 8, isolated cmd 7, and all-four DShot-0 + cmd 8 all left B clockwise. Reverse a stubborn channel by swapping any two of its three **motor phase wires**, not the signal jumper.
- **`0xD1` used to keep-alive all four with DShot-0.** Unused DShot-0 is mis-read as analog/oneshot on this ESC and neighbouring props creep. Firmware now drives only motors in the mask (optional `flags` bit0 restores the all-four zeros programming window).
- **DShot-0 on unused lines after prime** caused the same analog creep. Target-only output: a motor is silent until commanded; a newly live line gets 2s of DShot-0 (`DSHOT_REARM_MS`) before throttle ≥ 48.
- **Brown GPIO 13 (B) was cut** inside the Dupont; silence with `m=[0,400,0,0]` and a dead IMU was the open signal wire, not a bricked ESC save. White G5 and yellow G17 also needed reseats earlier in the same session.
- **Do not use `0xD2` (all-motor test throttle) on the bench with props on.** That was the wall incident. Identify motors with `0xD6` one-at-a-time, abort if any other `m=` goes live, output off after each pulse. Cap 700.

### Mixer / props (required before any hover)

- `flight_ctrl.c` `YAW_MIX` and `motor_mapping.h` now match A+C CW, B+D CCW. **OTA that firmware before arming flight** — the board still has the old mixer until flashed.
- Each prop must match its motor: CW prop on A and C, CCW prop on B and D, all thrusting **up**. B spun CW until the phase swap; if its prop was a CW blade it will now push down until replaced/flipped to a CCW prop.
- July 2026 ESC/prop notes (TR+BL CCW, TL+BR CW) are **superseded**.

### Not flight-ready yet

Frame was taped to a support for bench work. Trims `[71,71,92,122]` are from the old spin/prop map. Attitude bias from the 2026-07-04 hover is still open. No battery-voltage ADC. Confirm props, OTA mixer, untape, battery in reach, then a low collective hover — not an unattended fly.

## Previous status (2026-08-02): 5V RAIL FIXED; BR+TL channels dead (open)

### 5V rail rebuild (done, verified)

- **Buck replaced**: the undersized 15V→5V buck is gone. New power chain:
  - Battery XT60 → factory low-ESR electrolytic cap module (kept, on VBAT)
    + 4-in-1 ESC.
  - ESC harness "V" (red, ~15V VBAT) + GND (black) → **Mini560 buck module,
    5V fixed-output version** → ESP32 **5V0 pin**.
  - **2200µF 10V electrolytic across the Mini560 5V output** (+ on 5V,
    white-stripe leg on GND) to absorb BLE-TX/flash-write current peaks.
  - All buck splices are **soldered** (no Dupont anywhere between the ESC
    harness and the buck output; the old inline module used crimped joints).
- **Verified on battery only (USB unplugged)**: no LED crackle at plug-in,
  ESC boot melody plays, `reset=1` stays clean across an 11+ min session,
  BLE rock-solid, and the ESP32 survives the USB unplug without rebooting.
- The old buck's failure mode had been masking as a protocol bug: an ESC
  browning out during its own boot never completes protocol detection.
  If detection ever regresses, suspect power first (`reset=` in 0xB1 info).
- Battery-only OTA (the old buck's guaranteed-brownout crash test): not yet
  re-run — do it to formally close the brownout issue.

### Open: BR + TL motors don't respond (found same afternoon)

- IMU-measured probes at throttle 500: **TR and BL spin; BR and TL are
  flat** (noise-level deltas). Group `spin` confirms: only the TR+BL
  diagonal turns, net yaw torque visible on gz. The silent ESCs do their
  periodic "no signal" beeps.
- Two candidate causes, in test order:
  1. **ESC protocol detection never happened on those channels**: the
     battery has been plugged since morning, and at plug-in the DShot lines
     were actually silent (esc_tool one-shot commands cut output on BLE
     disconnect — the `output 1` run had already disconnected). Fix/test:
     hold a BLE connection with output armed (e.g. `spin 0 30`) while
     power-cycling the battery, then re-probe motors 1 and 2.
  2. **Signal wiring**: the brown (BR → GPIO 13) and orange (TL → GPIO 18)
     Dupont jumpers at the harness hub were disturbed during the buck
     soldering. Inspect/re-seat if the power-cycle test doesn't revive them.
- Note: firmware/BLE path was audited during diagnosis — per-motor values
  are correct end-to-end (`run_direction_probe` writes only its motor's
  channel; each motor has its own RMT channel and GPIO).
- **External deep-dive (GPT, 2026-08-02) conclusions applied:**
  - LED/RMT conflict (5 TX clients for 4 S3 channels) — already handled:
    the LED task is skipped in DShot mode (`main.c`), motors own all 4.
  - Shared copy encoder was a real design error but likely dormant (17
    symbols fit one 48-word block; loop_count replays in hardware). Fixed
    anyway: one encoder per channel.
  - **Leading suspect for the shifting dead channels: no zero-throttle
    acquisition window.** BLHeli_S re-scans protocols (~100ms each, DShot150
    mid-list) after every signal loss, then needs ~300ms of zero throttle to
    arm — and our output is cut on every BLE disconnect, i.e. between every
    esc_tool command. Fix implemented: 2s DShot0 prime on every output
    enable (values stored, emitted after the prime), `trans_queue_depth=1`
    so `rmt_disable()` leaves no stale driver-queued transaction, esc_tool
    waits out the prime. ESC signal-loss timeout is ~320ms (~160ms armed):
    silence is NOT an instant stop — always command zero first.
  - Still to verify with props off: whether the 2s prime revives all four
    channels across battery-only boots; scope the four lines if not.

### SAFETY INCIDENT (2026-08-02): runaway motors during PWM bench test

- During a `motor_test.py` collective ramp (340/1000, props ON), the BLE
  link died mid-test and the device stopped advertising entirely. The
  disconnect-event failsafe (`ble_handler.c`) never executed — when the MCU
  or BLE stack is dead, no callback runs — and the LEDC peripheral kept
  driving the last PWM value. Motors ran uncommanded until the operator
  pulled the battery by hand. Root cause of the hang itself: unknown (evlog
  to be read next session; suspect crash/hang while PWM active).
- **Fix (implemented same day)**: firmware-side inactivity failsafe in BOTH
  motor paths, independent of BLE events. Any received BLE command refreshes
  a deadline; the 100Hz control loops cut motors themselves after **3s**
  without traffic (`motor_control.c` PWM loop zeroes collective;
  `flight_ctrl.c` calls `flight_cut("link inactivity")`). Bench clients
  (`motor_test.py`, `flight_test.py`) now send ~1Hz keepalives during holds.
- **Bench rules going forward**: props OFF for any test that doesn't need
  thrust measurement; battery connector physically within arm's reach
  during every powered test; never rely on a software stop as the only stop.

### Still TODO from the 07-04 list

- Attitude bias tuning; battery voltage sensing mod; TR motor mechanical
  check (stalls/desyncs above ~650 solo).

## Previous status (2026-07-04): FIRST FLIGHT

- **The drone flew**: ~6s of stabilized hover at collective 780 (~38%),
  ground-effect altitude, commanded landing, no aborts. Yaw damped to
  +/-10dps (was -150dps runaway before the yaw-D term). Attitude held at
  roll ~+5deg / pitch ~-5deg (stable but biased - see tuning list).
- **Flight controller** (`main/flight_ctrl.c`, DShot mode, 100Hz):
  complementary-filter attitude + angle-PI / rate-D roll+pitch + yaw rate
  damping, trim-mixed outputs, slew-limited collective (buck-friendly),
  auto-cut at 25deg/400dps/IMU-fail/disconnect. BLE: 0xE0 arm, 0xE1
  collective, 0xE2 gains, 0xE3 status. Gains: kp=3.0 kd=0.6 ki=4.0 kdyaw=0.5.
- Lift threshold ~750-800 raw collective with trims [71,71,92,122].
- **Tuning next**: attitude bias (integrator clamp 70 insufficient or level
  reference off), altitude is open-loop (collective), and the 5V buck fix
  remains mandatory before flying without USB assist.
- Bench flow: `uv run tools/flight_test.py 0:2 600:3 700:4 780:10` (arms,
  steps collective, client aborts >12deg, always lands/disarms/silences).

## Previous status (2026-07-03, night)

- **Props are in the final flight arrangement**: owner swapped BR<->BL
  (identified by measurement — config-flatness test, tools/swap_id logic:
  the BR<->BL hypothesis ran 2.7-3.7x flatter than alternatives). Prop map:
  TR+BL = CCW, TL+BR = CW. ESCs set to match (TR+BL reversed, TL+BR normal);
  every prop thrusts up, torques cancel.
- Drone runs firmware `379f8fb` in **DShot config mode** with the new
  **silent-by-default output**: motor lines emit nothing until armed via
  `0xD7 0x01`, and output is cut on BLE disconnect (see safety incident in
  the protocols section). At battery-on the ESC will beep "no signal"
  periodically — normal and harmless; motors cannot spin.
- Trims recalibrated on the final props and deliberately SOFTENED to
  `[TR=71, BR=71, TL=92, BL=122]` (raw calibration said [50,50,85,150] but
  tilt magnitudes are stance-noisy; keep trims a soft prior for the PID).
- Remaining before hover: (1) IMU→frame axis mapping (`tools/axis_map.py`,
  ~2min, needs still drone + battery); (2) stabilization implementation;
  (3) hardware: 5V buck fix (hard blocker — brownouts on battery-only load),
  battery voltage sensing mod.

## Core components

| Component | Part | Notes |
|---|---|---|
| MCU | ESP32-S3 (N18R8 module) | 4 RMT TX channels total — see constraints below |
| IMU | ICM-42688-P (SPI) | accel + gyro + temp |
| ESC | UAngel 4-in-1, 45A (60A burst), BLHeli_S, 3-6S | supports DShot150/300; direction configurable via DShot commands |
| Status LED | WS2812 (onboard, GPIO 48) | driven by `led_strip` RMT backend |
| Battery | ~15V on ESC "V" wire → likely 4S LiPo | exact pack spec TODO; no voltage sensing wired (see Battery section) |
| Power | ESC "V" (15V) → Mini560 buck (5V) + 2200µF cap → ESP32 5V0 | replaced 2026-08-02; do NOT feed 15V to the ESP32 directly |

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
Owner lettering (XT60 = nose, A at top-right clockwise): A=TR, B=BR, C=BL, D=TL.

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
- **SAFETY INCIDENT (2026-07-03) + fix**: with the firmware streaming DShot
  zero-frames continuously from boot, the ESC once mis-detected the pulse
  train (likely as Multishot - DShot's 2.5-5us pulses overlap its range) and
  slow-spun all motors despite commanded zero. Fix: **motor lines are silent
  by default**; DShot output must be armed via opcode `0xD7 0x01` and is cut
  on BLE disconnect. A signal-less ESC disarms and cannot creep. Tools arm
  automatically; the boot state is always OFF.
- **After any ESP32 reboot (e.g. OTA) the ESC may silently stop responding**
  to DShot until a battery power-cycle: re-detection on signal resume is
  unreliable (observed both working and failing on the same day). Before any
  measurement run, do a liveness check — brief all-motor pulse, confirm az
  vibration on the IMU (`tools/axis_map.py` does this automatically).

### Hardware constraint: RMT channels

The ESP32-S3 has exactly **4 RMT TX channels**. DShot mode uses all four for
the motors, so the WS2812 status LED (also RMT-driven) cannot be initialized in
that mode — the firmware skips the LED task there (`ESP_ERROR_CHECK` inside
`led_strip` would otherwise abort → boot loop; this happened, fixed 2026-07-03).
**A dark status LED = DShot config mode.**

## Bench findings (2026-08-23 supersedes 2026-07-03 spin/prop map)

- **Confirmed X-quad spins (from above, XT60 = nose):** A/TR CW, B/BR CCW,
  C/BL CW, D/TL CCW. B was reversed by swapping two motor phase wires after
  DShot cmd 7/8/21 (isolated and all-four-live) failed to change it.
- **ESC `normal`/`reversed` DShot commands** on this 4-in-1 are not a reliable
  per-motor reverse. Do not depend on `0xD1`/`0xD5` save to flip a channel.
- **Keep unused DShot lines silent.** DShot-0 on idle channels analog-creeps
  neighbours. Isolated `0xD6` + 2s re-arm is the bench identification path.
- **DShot test throttle** is firmware-capped at raw 700. Identification pulses
  used 400.

### Historical (2026-07-03) — spin/prop map no longer current

- ESC `normal` = CW, `reversed` = CCW was measured on all four channels then.
- Then-current layout (TR+BL CCW, TL+BR CW, later a BR↔BL prop swap) is
  superseded by the 2026-08-23 table at the top of this file.
- **Measurement lesson**: single-motor tilt-magnitude probes are
  stance-dependent and unreliable across sessions (they flip-flopped on
  BR/BL/TR); the all-motor yaw-balance comparison at fixed throttle is the
  decisive instrument. Trim calibration inherits the same stance noise —
  recalibrate on the final prop config, same resting stance, and treat trims
  as a soft prior for the PID rather than truth.
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
- **Brownout under load — FIXED 2026-08-02** (Mini560 5V + 2200µF on the 5V
  rail; battery-only OTA still to be re-verified). Original findings kept for
  reference: even on a full pack
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
- **DShot test throttle** is firmware-capped at raw 700/2047 for bench
  safety; identification pulses use 400.
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
| `0xD1` | `[mask, 0\|1, flags?]` | ESC spin direction (isolated to mask). flags: bit0=keep-all DShot-0, bit1=skip save, bit2=cmd 20/21. Unreliable reverse on this 4-in-1. |
| `0xD2` | `[lo, hi]` | raw DShot test throttle 0 / 48-2047 (DShot mode) |
| `0xD3` | `[motor, lo, hi]` | direction probe: pulse motor, report gz/accel deltas |
| `0xD4` | `[tr, br, tl, bl]` | per-motor thrust trim % (50-150, 0/0xFF keep; empty = report) |
| `0xD7` | `[0\|1]` | DShot output off/on (boot=OFF, cut on BLE disconnect) |
| `0xD5` | `[motor, cmd]` | raw DShot command to one motor (1-5 = beeps) |

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

- [x] Four isolated corners mapped (A=TR, B=BR, C=BL, D=TL)
- [x] Spins set for X-quad: A+C CW, B+D CCW (B via phase-wire swap)
- [x] Yaw mixer updated in source to match those spins (needs OTA)
- [ ] Verify each prop matches its motor (CW prop on A,C; CCW on B,D; thrust up)
- [ ] OTA mixer firmware; untape from the bench support
- [ ] Recalibrate trims on the final props (old `[71,71,92,122]` is stale)
- [ ] Low collective hover with battery in reach — not an unattended flight
- [ ] Battery voltage sensing mod (see above)
- [ ] Attitude bias from the 2026-07-04 hover still open
