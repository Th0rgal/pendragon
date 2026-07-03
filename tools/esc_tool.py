# /// script
# requires-python = ">=3.10"
# dependencies = ["bleak==0.22.3"]
# ///
"""Pendragon BLE tool: OTA updates, ESC config, debugging.

Run with `uv run tools/esc_tool.py <command>` (deps resolve automatically).

Usage:
  esc_tool.py ota <firmware.bin>     flash over BLE (resumable, batched)
  esc_tool.py info                   ping + firmware info + motor status
  esc_tool.py sensors [secs]         poll IMU snapshots
  esc_tool.py stream [hz] [secs]     continuous IMU+motor telemetry stream
  esc_tool.py evlog                  dump the firmware event log
  esc_tool.py mode <0|1>             0=pwm flight, 1=dshot config (reboots;
                                     power-cycle battery so ESC re-detects)
  esc_tool.py reverse <mask>         DShot mode: set CCW for motor mask (0x0F=all)
  esc_tool.py normal <mask>          DShot mode: set CW for motor mask
  esc_tool.py throttle <value>       DShot mode: raw throttle (0 stop, 48-2047)
  esc_tool.py spin <value> <secs>    DShot mode: spin with IMU monitoring
  esc_tool.py probe <motor> <thr>    DShot mode: direction probe (gyro+accel)
  esc_tool.py auto                   DShot mode: differential-thrust direction
                                     detection + auto-fix on all motors
  esc_tool.py calibrate              DShot mode: measure per-motor thrust and
                                     write compensating trims (for PWM mode)
  esc_tool.py trim [tr br tl bl]     get/set per-motor thrust trim % (50-150)
  esc_tool.py rawcmd <motor> <cmd>   DShot mode: raw ESC command (1-5=beeps)

Motor order: 0=TOP RIGHT, 1=BOTTOM RIGHT, 2=TOP LEFT, 3=BOTTOM LEFT.
See HARDWARE.md for the full protocol and bench findings.
"""

import asyncio
import struct
import sys
import time

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "Pendragon"
COMMAND_CHAR = "00ffeedd-ccbb-aa99-8877-665544332211"
TELEMETRY_CHAR = "11223344-5566-7788-99aa-bbccddeeff00"

OP_PING = 0xB0
OP_INFO = 0xB1
OP_SENSOR_SNAPSHOT = 0xB2
OP_MOTOR_STATUS = 0xB3
OP_OTA_BEGIN = 0xC0
OP_OTA_DATA = 0xC1
OP_OTA_END = 0xC2
OP_OTA_ABORT = 0xC3
OP_OTA_STATUS = 0xC4
OP_MOTOR_MODE = 0xD0
OP_ESC_DIRECTION = 0xD1
OP_ESC_TEST_THROTTLE = 0xD2
OP_ESC_DIRECTION_PROBE = 0xD3
OP_TELEMETRY_STREAM = 0xB4
OP_EVENT_LOG = 0xB5
OP_MOTOR_TRIM = 0xD4
OP_ESC_RAW_CMD = 0xD5

MOTOR_NAMES = ["TOP RIGHT", "BOTTOM RIGHT", "TOP LEFT", "BOTTOM LEFT"]

START = time.monotonic()
TELEMETRY_QUEUE: "asyncio.Queue[str]" = None


def log(msg):
    print(f"[{time.monotonic() - START:7.2f}s] {msg}", flush=True)


def on_telemetry(_char, data: bytearray):
    text = data.decode("utf-8", errors="replace")
    log(f"TELEMETRY <- {text}")
    if TELEMETRY_QUEUE is not None:
        TELEMETRY_QUEUE.put_nowait(text)


async def wait_telemetry(substr, timeout):
    """Wait for a telemetry line containing substr; return it or None."""
    end = time.monotonic() + timeout
    while (remaining := end - time.monotonic()) > 0:
        try:
            line = await asyncio.wait_for(TELEMETRY_QUEUE.get(), remaining)
        except asyncio.TimeoutError:
            return None
        if substr in line:
            return line
    return None


async def connect():
    log(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=20.0)
    if device is None:
        log("Device not found.")
        sys.exit(2)
    log(f"Found {device.name} [{device.address}], connecting...")
    client = BleakClient(device, timeout=20.0)
    await client.connect()
    log("Connected.")
    await client.start_notify(TELEMETRY_CHAR, on_telemetry)
    await asyncio.sleep(0.3)
    return client


async def write(client, payload, response=True):
    await client.write_gatt_char(COMMAND_CHAR, bytes(payload), response=response)


async def cmd_info(client):
    await write(client, [OP_PING])
    await asyncio.sleep(0.4)
    await write(client, [OP_INFO])
    await asyncio.sleep(0.4)
    await write(client, [OP_MOTOR_STATUS])
    await asyncio.sleep(0.6)


async def ota_query_status(client):
    """Ask the firmware for OTA session state; returns dict or None.
    Older firmware rejects the opcode entirely - treat as no session."""
    try:
        await write(client, [OP_OTA_STATUS])
    except Exception:
        return None
    line = await wait_telemetry("status in_progress", timeout=5.0)
    if line is None:
        return None
    fields = dict(re.findall(r"(\w+)=(\d+)", line))
    return {k: int(v) for k, v in fields.items()}


async def ota_session(client, image, start_offset, start_seq):
    """Send image data from an offset; returns True when END was accepted."""
    mtu = client.mtu_size or 185
    chunk_size = min(mtu - 3, 512) - 3
    log(f"MTU={mtu}, chunk={chunk_size}B, resuming at {start_offset}/{len(image)} seq={start_seq}")

    seq = start_seq
    sent = start_offset
    t0 = time.monotonic()
    for offset in range(start_offset, len(image), chunk_size):
        chunk = image[offset : offset + chunk_size]
        await write(client, struct.pack("<BH", OP_OTA_DATA, seq) + chunk)
        seq += 1
        sent += len(chunk)
        # Batch pacing: pause regularly so the 5V rail recovers between
        # radio/flash bursts (marginal buck converter brownouts otherwise).
        if seq % 64 == 0:
            await asyncio.sleep(1.0)
        if seq % 320 == 0:
            rate = (sent - start_offset) / max(time.monotonic() - t0, 0.001)
            eta = (len(image) - sent) / max(rate, 1)
            log(f"progress {sent}/{len(image)} ({100*sent//len(image)}%) "
                f"{rate/1024:.1f} KB/s eta {eta:.0f}s")
    log(f"All data sent (seq={seq}), finalizing...")
    await write(client, [OP_OTA_END])
    await asyncio.sleep(2.0)
    return True


async def cmd_ota(path):
    """Resumable OTA: reconnect and continue from the firmware-reported
    offset after brownout/BLE drops instead of restarting."""
    with open(path, "rb") as f:
        image = f.read()
    log(f"Firmware {path}: {len(image)} bytes")

    for attempt in range(1, 11):
        client = None
        try:
            client = await connect()
            status = await ota_query_status(client)
            if (status and status.get("in_progress") == 1
                    and status.get("expected") == len(image)):
                start_offset = status["received"]
                start_seq = status["seq"]
                log(f"Resuming previous session at {start_offset} bytes")
            else:
                await write(client, struct.pack("<BI", OP_OTA_BEGIN, len(image)))
                await asyncio.sleep(0.5)
                start_offset, start_seq = 0, 0

            if await ota_session(client, image, start_offset, start_seq):
                log("OTA complete; device is rebooting into the new firmware.")
                return
        except Exception as exc:
            log(f"attempt {attempt} interrupted: {exc}; waiting for rail recovery...")
            await asyncio.sleep(4.0)
        finally:
            if client is not None:
                try:
                    await client.disconnect()
                except Exception:
                    pass
    log("OTA FAILED after 10 attempts")
    sys.exit(3)


async def cmd_mode(client, mode):
    await write(client, [OP_MOTOR_MODE, mode])
    await asyncio.sleep(1.5)
    log("Mode saved; device rebooting. Power-cycle the battery so the ESC "
        "re-detects the signal protocol.")


async def cmd_direction(client, mask, reversed_):
    await write(client, [OP_ESC_DIRECTION, mask, 1 if reversed_ else 0])
    # Worker takes ~0.9s per motor; wait for acks.
    await asyncio.sleep(1.2 * bin(mask).count("1") + 1.5)


async def cmd_throttle(client, value):
    await write(client, [OP_ESC_TEST_THROTTLE, value & 0xFF, (value >> 8) & 0xFF])
    await asyncio.sleep(0.6)


async def poll_sensors(client, secs, period=0.5):
    """Stream IMU snapshots for `secs`; telemetry callback prints them."""
    for _ in range(max(1, int(secs / period))):
        await write(client, [OP_SENSOR_SNAPSHOT])
        await asyncio.sleep(period)


async def cmd_spin(client, value, secs):
    log("--- IMU baseline (motors stopped) ---")
    await poll_sensors(client, 1.5)
    log(f"--- throttle {value} for {secs}s, monitoring IMU ---")
    await cmd_throttle(client, value)
    await poll_sensors(client, secs)
    await cmd_throttle(client, 0)
    log("--- IMU after stop ---")
    await poll_sensors(client, 1.5)
    await write(client, [OP_MOTOR_STATUS])
    await asyncio.sleep(0.6)


async def cmd_sensors(client, secs):
    await poll_sensors(client, secs)


import math
import re


async def probe_motor(client, motor, throttle):
    """Run one firmware probe; return dict with gz, dax, day, mag, verdict."""
    await write(client, [OP_ESC_DIRECTION_PROBE, motor,
                         throttle & 0xFF, (throttle >> 8) & 0xFF])
    line = await wait_telemetry(f"probe motor={motor}", timeout=20.0)
    if line is None:
        return None
    numbers = dict(re.findall(r"(\w+)=([+-]?\d+(?:\.\d+)?)", line))
    dax = float(numbers.get("dax", 0))
    day = float(numbers.get("day", 0))
    verdict = re.search(r"verdict=(\w+)", line)
    return {
        "gz": float(numbers.get("gz", 0)),
        "dax": dax,
        "day": day,
        "mag": math.hypot(dax, day),
        "verdict": verdict.group(1) if verdict else "UNCLEAR",
    }


async def set_direction(client, motor, reversed_):
    await write(client, [OP_ESC_DIRECTION, 1 << motor, 1 if reversed_ else 0])
    await wait_telemetry("sequence complete", timeout=10.0)


async def cmd_calibrate(client):
    """Measure per-motor thrust (probe tilt) and write compensating trims.

    Trims only apply in PWM flight mode; run this in DShot mode where
    per-motor throttle exists, then switch modes. Re-run after any prop
    change - trim compensates the motor+prop combination.
    """
    THROTTLE = 500
    ROUNDS = 3
    medians = {}
    for motor in range(4):
        values = []
        for round_no in range(ROUNDS):
            log(f"=== Motor {motor} ({MOTOR_NAMES[motor]}): thrust probe "
                f"{round_no + 1}/{ROUNDS} ===")
            probe = await probe_motor(client, motor, THROTTLE)
            if probe:
                values.append(probe["mag"])
        values.sort()
        medians[motor] = values[len(values) // 2] if values else 0.0
        log(f"  median tilt {medians[motor]*1000:.1f}mg")

    if any(m <= 0.002 for m in medians.values()):
        weak = [MOTOR_NAMES[m] for m, v in medians.items() if v <= 0.002]
        log(f"CALIBRATION ABORTED: no measurable thrust from {weak} - "
            "inspect hardware before trimming")
        return

    target = max(medians.values())
    raw = {m: 100.0 * target / medians[m] for m in medians}
    scale = min(1.0, 150.0 / max(raw.values()))
    trims = [max(50, min(150, round(raw[m] * scale))) for m in range(4)]
    log(f"computed trims (tr,br,tl,bl): {trims}")
    await write(client, [OP_MOTOR_TRIM] + trims)
    await wait_telemetry("trim", timeout=5.0)
    log("Trims saved to NVS; they apply in PWM flight mode.")


async def cmd_thrustmap(client, rounds=4):
    """Definitive per-prop thrust-direction map.

    For each motor and each ESC direction setting, run several probes and
    take medians. The setting with the larger sustained tilt is the one where
    the prop blows air DOWN (thrust up); the gyro sign at that setting names
    the rotation (chip +Z down: CCW seen from above => +gz). Leaves every ESC
    on its thrust-up setting. Single motor at <=23% throttle - cannot lift.
    """
    THROTTLE = 650
    summary = {}
    for motor in range(4):
        name = MOTOR_NAMES[motor]
        samples = {False: {"mags": [], "gzs": []}, True: {"mags": [], "gzs": []}}
        # Interleave settings (A,B,A,B...) so both are measured under the
        # same resting stance - tilt sensitivity drifts when the frame shifts.
        for round_no in range(rounds):
            for setting_reversed in (False, True):
                label = "reversed" if setting_reversed else "normal"
                log(f"=== {name}: '{label}' probe {round_no + 1}/{rounds} ===")
                await set_direction(client, motor, setting_reversed)
                probe = await probe_motor(client, motor, THROTTLE)
                if probe:
                    samples[setting_reversed]["mags"].append(probe["mag"])
                    samples[setting_reversed]["gzs"].append(probe["gz"])

        stats = {}
        for setting_reversed in (False, True):
            mags = sorted(samples[setting_reversed]["mags"])
            gzs = samples[setting_reversed]["gzs"]
            median_mag = mags[len(mags) // 2] if mags else 0.0
            mean_gz = sum(gzs) / len(gzs) if gzs else 0.0
            stats[setting_reversed] = (median_mag, mean_gz)
            label = "reversed" if setting_reversed else "normal"
            log(f"  {name} '{label}': median tilt {median_mag*1000:.1f}mg, "
                f"mean gz {mean_gz:+.2f}deg")

        mag_n, gz_n = stats[False]
        mag_r, gz_r = stats[True]
        best = mag_r > mag_n
        ratio = max(mag_n, mag_r) / max(min(mag_n, mag_r), 1e-6)
        gz_best = stats[best][1]
        spin = "CCW" if gz_best > 0 else "CW"
        await set_direction(client, motor, best)
        summary[name] = {
            "esc_setting": "reversed" if best else "normal",
            "prop_thrust_direction": spin,
            "tilt_ratio": ratio,
            "confidence": "OK" if ratio > 1.4 else "LOW",
        }

    log("=== THRUST MAP (ESCs left on thrust-up settings) ===")
    for name, r in summary.items():
        log(f"  {name}: prop wants {r['prop_thrust_direction']}, "
            f"esc={r['esc_setting']}, ratio x{r['tilt_ratio']:.1f} "
            f"[{r['confidence']}]")


async def cmd_auto(client):
    """Differential-thrust direction finder.

    For each motor, probe under both ESC direction settings. The setting
    where the prop produces real (upward) thrust tilts the frame and gives a
    larger sustained accel delta; spinning backwards pushes into the ground
    and barely registers. Keep the higher-thrust setting.
    """
    THROTTLE = 500
    results = {}
    for motor in range(4):
        name = MOTOR_NAMES[motor]
        measurements = {}
        for setting_reversed in (False, True):
            label = "reversed" if setting_reversed else "normal"
            log(f"=== Motor {motor} ({name}): setting '{label}', probing ===")
            await set_direction(client, motor, setting_reversed)
            probe = await probe_motor(client, motor, THROTTLE)
            if probe is None:
                log("probe timed out")
                continue
            measurements[setting_reversed] = probe
            log(f"    tilt mag={probe['mag']*1000:.1f}mg gz={probe['gz']:+.2f}deg")

        if len(measurements) < 2:
            results[name] = "PROBE FAILED - manual check needed"
            continue

        mag_n = measurements[False]["mag"]
        mag_r = measurements[True]["mag"]
        best = mag_r > mag_n
        ratio = max(mag_n, mag_r) / max(min(mag_n, mag_r), 1e-6)
        gz_best = measurements[best]["gz"]
        spin = "CCW" if gz_best > 0 else "CW"

        # Leave the ESC on the winning setting.
        await set_direction(client, motor, best)
        confidence = "confident" if ratio > 1.5 else "LOW CONFIDENCE"
        results[name] = (
            f"kept '{'reversed' if best else 'normal'}' "
            f"(thrust {max(mag_n, mag_r)*1000:.1f}mg vs {min(mag_n, mag_r)*1000:.1f}mg, "
            f"x{ratio:.1f}, {confidence}); gyro says {spin}"
        )

    log("=== SUMMARY ===")
    for name, outcome in results.items():
        log(f"  {name}: {outcome}")


async def main():
    global TELEMETRY_QUEUE
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    action = sys.argv[1]

    TELEMETRY_QUEUE = asyncio.Queue()

    if action == "ota":
        await cmd_ota(sys.argv[2])  # manages its own connection (resume)
        log("Done.")
        return

    client = await connect()
    try:
        if action == "info":
            await cmd_info(client)
        elif action == "mode":
            await cmd_mode(client, int(sys.argv[2], 0))
        elif action in ("reverse", "normal"):
            await cmd_direction(client, int(sys.argv[2], 0), action == "reverse")
        elif action == "throttle":
            await cmd_throttle(client, int(sys.argv[2], 0))
        elif action == "spin":
            await cmd_spin(client, int(sys.argv[2], 0), float(sys.argv[3]))
        elif action == "sensors":
            await cmd_sensors(client, float(sys.argv[2]) if len(sys.argv) > 2 else 3.0)
        elif action == "probe":
            verdict = await probe_motor(client, int(sys.argv[2]), int(sys.argv[3], 0))
            log(f"verdict: {verdict}")
        elif action == "auto":
            await cmd_auto(client)
        elif action == "calibrate":
            await cmd_calibrate(client)
        elif action == "thrustmap":
            await cmd_thrustmap(client, int(sys.argv[2]) if len(sys.argv) > 2 else 4)
        elif action == "evlog":
            await write(client, [OP_EVENT_LOG])
            await wait_telemetry("end of event log", timeout=15.0)
        elif action == "stream":
            hz = int(sys.argv[2]) if len(sys.argv) > 2 else 10
            secs = int(sys.argv[3]) if len(sys.argv) > 3 else 5
            await write(client, [OP_TELEMETRY_STREAM, hz, secs])
            await wait_telemetry("stream end", timeout=secs + 10.0)
        elif action == "trim":
            if len(sys.argv) >= 6:
                await write(client, [OP_MOTOR_TRIM] + [int(a) for a in sys.argv[2:6]])
            else:
                await write(client, [OP_MOTOR_TRIM])
            await wait_telemetry("trim", timeout=5.0)
        elif action == "rawcmd":
            await write(client, [OP_ESC_RAW_CMD, int(sys.argv[2]), int(sys.argv[3])])
            await wait_telemetry("raw cmd", timeout=10.0)
        else:
            print(__doc__)
            sys.exit(1)
    finally:
        try:
            await client.disconnect()
        except Exception:
            pass
    log("Done.")


if __name__ == "__main__":
    asyncio.run(main())
