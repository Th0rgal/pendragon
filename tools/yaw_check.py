# /// script
# requires-python = ">=3.10"
# dependencies = ["bleak==0.22.3"]
# ///
"""Validate the 2+2 prop hypothesis via net yaw torque.

Config A (hypothesis, thrust-up everywhere): TR+BR reversed (CCW), TL+BL
normal (CW) -> reaction torques cancel -> minimal yaw drift.
Config C (all normal = all CW) -> torques add -> maximal yaw drift.
All-four spin at 450 (~21%), 4s, client-side abort on big motion.
"""

import asyncio
import re
import statistics
import time
from bleak import BleakClient, BleakScanner

COMMAND_CHAR = "00ffeedd-ccbb-aa99-8877-665544332211"
TELEMETRY_CHAR = "11223344-5566-7788-99aa-bbccddeeff00"
START = time.monotonic()
SAMPLES = []
ABORT = asyncio.Event()


def log(msg):
    print(f"[{time.monotonic() - START:6.2f}s] {msg}", flush=True)


def on_telemetry(_c, data):
    text = data.decode("utf-8", errors="replace")
    log(text)
    m = re.search(r"a=([+-][\d.]+),([+-][\d.]+),([+-][\d.]+) "
                  r"g=([+-][\d.]+),([+-][\d.]+),([+-][\d.]+)", text)
    if m:
        ax, ay, az, gx, gy, gz = map(float, m.groups())
        SAMPLES.append((ax, ay, az, gx, gy, gz))
        # Abort on strong rotation or big accel deviation (tip/lift onset).
        if max(abs(gx), abs(gy)) > 60 or abs(az + 1.0) > 0.5:
            ABORT.set()


async def main():
    device = await BleakScanner.find_device_by_name("Pendragon", timeout=20.0)
    assert device, "drone not found"
    async with BleakClient(device, timeout=20.0) as client:
        await client.start_notify(TELEMETRY_CHAR, on_telemetry)
        await asyncio.sleep(0.3)

        async def cmd(*payload):
            await client.write_gatt_char(COMMAND_CHAR, bytes(payload), response=True)

        async def set_dirs(rev_mask):
            if rev_mask:
                await cmd(0xD1, rev_mask, 1)
                await asyncio.sleep(1.2 * bin(rev_mask).count("1") + 1.0)
            norm_mask = 0x0F & ~rev_mask
            if norm_mask:
                await cmd(0xD1, norm_mask, 0)
                await asyncio.sleep(1.2 * bin(norm_mask).count("1") + 1.0)

        async def measure(label, rev_mask):
            log(f"##### config {label}: rev_mask=0x{rev_mask:02x}")
            await set_dirs(rev_mask)
            SAMPLES.clear()
            ABORT.clear()
            await cmd(0xB4, 10, 12)          # stream 10Hz 12s
            await asyncio.sleep(1.0)
            baseline = list(SAMPLES)
            await cmd(0xD2, 450 & 0xFF, 450 >> 8)
            spin_start = len(SAMPLES)
            for _ in range(40):              # up to 4s
                if ABORT.is_set():
                    log("!!! ABORT threshold hit")
                    break
                await asyncio.sleep(0.1)
            await cmd(0xD2, 0, 0)
            spin = SAMPLES[spin_start + 3:]  # skip spin-up
            await asyncio.sleep(1.0)
            await cmd(0xB4, 0)
            await asyncio.sleep(0.5)
            if not spin:
                log(f"config {label}: no samples")
                return None
            mean_gz = statistics.mean(s[5] for s in spin)
            mean_gx = statistics.mean(s[3] for s in spin)
            mean_gy = statistics.mean(s[4] for s in spin)
            bias_gz = statistics.mean(s[5] for s in baseline) if baseline else 0.0
            log(f">>> config {label}: mean gz {mean_gz:+.2f} (bias {bias_gz:+.2f}) "
                f"gx {mean_gx:+.2f} gy {mean_gy:+.2f} n={len(spin)}")
            return mean_gz - bias_gz

        # A: hypothesis 2+2 balanced (TR,BR reversed=CCW; TL,BL normal=CW)
        a = await measure("A-balanced", 0x03)
        await asyncio.sleep(2.0)
        # C: all normal = all CW (maximum net torque)
        c = await measure("C-all-CW", 0x00)
        # Restore hypothesis config (thrust-up everywhere) before leaving.
        await set_dirs(0x03)

        log(f"RESULT: yaw drift A(balanced)={a and f'{a:+.2f}'} "
            f"C(all-CW)={c and f'{c:+.2f}'} dps")

asyncio.run(main())
