# /// script
# requires-python = ">=3.10"
# dependencies = ["bleak==0.22.3"]
# ///
"""Flight controller bench test: arm, hold collective steps, monitor, disarm.

Usage: flight_test.py <step:secs> [step:secs ...]
e.g.:  flight_test.py 0:8 300:8        estimator check then ground spin
Client-side abort on |attitude| > 12deg on top of firmware's 25deg cut.
"""

import asyncio
import re
import sys
import time
from bleak import BleakClient, BleakScanner

COMMAND_CHAR = "00ffeedd-ccbb-aa99-8877-665544332211"
TELEMETRY_CHAR = "11223344-5566-7788-99aa-bbccddeeff00"
START = time.monotonic()
ABORT = asyncio.Event()


def log(msg):
    print(f"[{time.monotonic() - START:6.2f}s] {msg}", flush=True)


def on_telemetry(_c, data):
    text = data.decode("utf-8", errors="replace")
    log(text)
    m = re.search(r"att=([+-][\d.]+),([+-][\d.]+)", text)
    if m:
        roll, pitch = float(m.group(1)), float(m.group(2))
        if max(abs(roll), abs(pitch)) > 12.0:
            ABORT.set()


async def main():
    steps = []
    for arg in sys.argv[1:]:
        c, s = arg.split(":")
        steps.append((int(c), float(s)))

    device = await BleakScanner.find_device_by_name("Pendragon", timeout=20.0)
    assert device, "drone not found"
    async with BleakClient(device, timeout=20.0) as client:
        await client.start_notify(TELEMETRY_CHAR, on_telemetry)
        await asyncio.sleep(0.3)

        async def cmd(*payload):
            await client.write_gatt_char(COMMAND_CHAR, bytes(payload), response=True)

        try:
            await cmd(0xE0, 1)  # arm (enables output, captures bias)
            await asyncio.sleep(1.5)
            await cmd(0xB4, 10, 60)  # stream 10Hz 60s

            for collective, secs in steps:
                if ABORT.is_set():
                    break
                log(f">>> collective {collective} for {secs}s")
                await cmd(0xE1, collective & 0xFF, collective >> 8)
                end = time.monotonic() + secs
                while time.monotonic() < end and not ABORT.is_set():
                    await asyncio.sleep(0.1)
                if ABORT.is_set():
                    log("!!! client abort: attitude > 12deg")
        finally:
            await cmd(0xE1, 0, 0)
            await asyncio.sleep(1.5)
            await cmd(0xE0, 0)  # disarm
            await cmd(0xE3)     # status
            await asyncio.sleep(1.0)
            await cmd(0xB4, 0)
            await asyncio.sleep(0.5)
            await cmd(0xD7, 0)  # silence lines
            await asyncio.sleep(0.5)
    log("test complete")

asyncio.run(main())
