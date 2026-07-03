# /// script
# requires-python = ">=3.10"
# dependencies = ["bleak==0.22.3"]
# ///
"""IMU-to-frame axis mapping via per-corner thrust pulses.

Each motor pulse unloads its corner: front motors tilt the frame nose-up,
back motors nose-down; right motors tilt right-side-up, left motors
left-side-up. Contrasting accel deltas across the 4 corners yields which
chip axis (and sign) is frame-forward and frame-right. Chip Z is known
(az=-1g at rest => chip +Z points down).
"""

import asyncio
import re
import statistics
import time
from bleak import BleakClient, BleakScanner

COMMAND_CHAR = "00ffeedd-ccbb-aa99-8877-665544332211"
TELEMETRY_CHAR = "11223344-5566-7788-99aa-bbccddeeff00"
MOTOR_NAMES = ["TR", "BR", "TL", "BL"]
THROTTLE = 650
ROUNDS = 3
START = time.monotonic()
SAMPLES = []


def log(msg):
    print(f"[{time.monotonic() - START:6.2f}s] {msg}", flush=True)


def on_telemetry(_c, data):
    text = data.decode("utf-8", errors="replace")
    m = re.search(r"a=([+-][\d.]+),([+-][\d.]+),([+-][\d.]+) "
                  r"g=([+-][\d.]+),([+-][\d.]+),([+-][\d.]+)", text)
    if m:
        SAMPLES.append(tuple(map(float, m.groups())))
    elif "STR" not in text:
        log(text)


def mean_axes(samples):
    if not samples:
        return None
    return [statistics.mean(s[i] for s in samples) for i in range(6)]


async def main():
    device = await BleakScanner.find_device_by_name("Pendragon", timeout=20.0)
    assert device, "drone not found"
    async with BleakClient(device, timeout=20.0) as client:
        await client.start_notify(TELEMETRY_CHAR, on_telemetry)
        await asyncio.sleep(0.3)

        async def cmd(*payload):
            await client.write_gatt_char(COMMAND_CHAR, bytes(payload), response=True)

        # Liveness gate: motors must actually spin (ESC may silently ignore
        # DShot after an ESP32 reboot until a battery power-cycle).
        await cmd(0xB4, 20, 5)
        await asyncio.sleep(0.3)
        SAMPLES.clear()
        await asyncio.sleep(1.0)
        quiet = [s for s in SAMPLES]
        await cmd(0xD2, 300 & 0xFF, 300 >> 8)
        SAMPLES.clear()
        await asyncio.sleep(1.0)
        spinning = [s for s in SAMPLES]
        await cmd(0xD2, 0, 0)
        await cmd(0xB4, 0)
        await asyncio.sleep(1.0)

        def vib(samples):
            if len(samples) < 4:
                return 0.0
            azs = [s[2] for s in samples]
            return statistics.pstdev(azs)

        vib_quiet, vib_spin = vib(quiet), vib(spinning)
        log(f"liveness: az stdev quiet={vib_quiet*1000:.1f}mg spin={vib_spin*1000:.1f}mg")
        if vib_spin < vib_quiet * 2 + 0.002:
            log("MOTORS NOT SPINNING - power-cycle the battery and rerun")
            return

        deltas = {}
        for motor in range(4):
            dax_list, day_list = [], []
            for round_no in range(ROUNDS):
                await cmd(0xB4, 20, 8)  # stream 20Hz 8s
                await asyncio.sleep(0.3)
                SAMPLES.clear()
                await asyncio.sleep(1.2)
                baseline = mean_axes(list(SAMPLES))
                await cmd(0xD6, motor, THROTTLE & 0xFF, THROTTLE >> 8)
                await asyncio.sleep(0.4)  # skip spin-up
                SAMPLES.clear()
                await asyncio.sleep(1.8)
                during = mean_axes(list(SAMPLES))
                await cmd(0xD6, motor, 0, 0)
                await cmd(0xB4, 0)
                await asyncio.sleep(1.2)
                if baseline and during:
                    dax_list.append(during[0] - baseline[0])
                    day_list.append(during[1] - baseline[1])
                    log(f"{MOTOR_NAMES[motor]} r{round_no+1}: "
                        f"dax={during[0]-baseline[0]:+.4f} "
                        f"day={during[1]-baseline[1]:+.4f}")
            dax_list.sort(); day_list.sort()
            deltas[motor] = (dax_list[len(dax_list)//2] if dax_list else 0.0,
                             day_list[len(day_list)//2] if day_list else 0.0)
            log(f"== {MOTOR_NAMES[motor]} median: dax={deltas[motor][0]:+.4f} "
                f"day={deltas[motor][1]:+.4f}")

        # Contrasts: front(TR=0,TL=2) - back(BR=1,BL=3) => frame +X (forward)
        #            right(TR=0,BR=1) - left(TL=2,BL=3) => frame +Y (right)
        fx = ((deltas[0][0] + deltas[2][0]) - (deltas[1][0] + deltas[3][0])) / 2
        fy = ((deltas[0][1] + deltas[2][1]) - (deltas[1][1] + deltas[3][1])) / 2
        rx = ((deltas[0][0] + deltas[1][0]) - (deltas[2][0] + deltas[3][0])) / 2
        ry = ((deltas[0][1] + deltas[1][1]) - (deltas[2][1] + deltas[3][1])) / 2
        log(f"RESULT front-back contrast in chip coords: ({fx*1000:+.1f}, {fy*1000:+.1f}) mg")
        log(f"RESULT right-left contrast in chip coords: ({rx*1000:+.1f}, {ry*1000:+.1f}) mg")

asyncio.run(main())
