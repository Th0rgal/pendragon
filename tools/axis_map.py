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
THROTTLE = 500
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


CLIENT = None


async def new_client():
    """(Re)connect, subscribe, arm output. Retries; brownouts drop the link."""
    global CLIENT
    if CLIENT is not None:
        try:
            await CLIENT.disconnect()
        except Exception:
            pass
        CLIENT = None
    for attempt in range(8):
        try:
            device = await BleakScanner.find_device_by_name("Pendragon", timeout=20.0)
            if device is None:
                continue
            client = BleakClient(device, timeout=20.0)
            await client.connect()
            await client.start_notify(TELEMETRY_CHAR, on_telemetry)
            await asyncio.sleep(0.3)
            await client.write_gatt_char(COMMAND_CHAR, bytes([0xD7, 1]), response=True)
            await asyncio.sleep(0.5)
            CLIENT = client
            return
        except Exception as exc:
            log(f"reconnect attempt {attempt+1}: {exc}")
            await asyncio.sleep(3.0)
    raise RuntimeError("cannot reach drone")


async def main():
    await new_client()
    if True:
        async def cmd(*payload):
            await CLIENT.write_gatt_char(COMMAND_CHAR, bytes(payload), response=True)

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

        # Gyro rate at pulse ONSET (first ~0.5s): the frame rotates as the
        # corner unloads, before slide/settle contaminates the signal.
        deltas = {}
        for motor in range(4):
            gx_list, gy_list = [], []
            round_no = 0
            failures = 0
            while round_no < ROUNDS and failures < 6:
                try:
                    await cmd(0xB4, 20, 6)  # stream 20Hz 6s
                    await asyncio.sleep(0.3)
                    SAMPLES.clear()
                    await asyncio.sleep(1.0)
                    baseline = mean_axes(list(SAMPLES))
                    SAMPLES.clear()
                    await cmd(0xD6, motor, THROTTLE & 0xFF, THROTTLE >> 8)
                    await asyncio.sleep(0.55)  # onset window
                    onset = mean_axes(list(SAMPLES))
                    await cmd(0xD6, motor, 0, 0)
                    await cmd(0xB4, 0)
                    await asyncio.sleep(1.3)
                except Exception as exc:
                    failures += 1
                    log(f"round dropped ({exc}); reconnecting...")
                    await asyncio.sleep(4.0)
                    await new_client()
                    continue
                round_no += 1
                if baseline and onset:
                    dgx = onset[3] - baseline[3]
                    dgy = onset[4] - baseline[4]
                    gx_list.append(dgx)
                    gy_list.append(dgy)
                    log(f"{MOTOR_NAMES[motor]} r{round_no}: "
                        f"dgx={dgx:+.2f} dgy={dgy:+.2f} dps")
            gx_list.sort(); gy_list.sort()
            deltas[motor] = (gx_list[len(gx_list)//2] if gx_list else 0.0,
                             gy_list[len(gy_list)//2] if gy_list else 0.0)
            log(f"== {MOTOR_NAMES[motor]} median onset: dgx={deltas[motor][0]:+.2f} "
                f"dgy={deltas[motor][1]:+.2f} dps")

        # Contrasts: front(TR=0,TL=2) - back(BR=1,BL=3) => frame +X (forward)
        #            right(TR=0,BR=1) - left(TL=2,BL=3) => frame +Y (right)
        fx = ((deltas[0][0] + deltas[2][0]) - (deltas[1][0] + deltas[3][0])) / 2
        fy = ((deltas[0][1] + deltas[2][1]) - (deltas[1][1] + deltas[3][1])) / 2
        rx = ((deltas[0][0] + deltas[1][0]) - (deltas[2][0] + deltas[3][0])) / 2
        ry = ((deltas[0][1] + deltas[1][1]) - (deltas[2][1] + deltas[3][1])) / 2
        log(f"RESULT front-back gyro contrast (chip gx,gy): ({fx:+.2f}, {fy:+.2f}) dps")
        log(f"RESULT right-left gyro contrast (chip gx,gy): ({rx:+.2f}, {ry:+.2f}) dps")

asyncio.run(main())
