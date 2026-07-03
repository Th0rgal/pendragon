# /// script
# requires-python = ">=3.10"
# dependencies = ["bleak==0.22.3"]
# ///
"""Pendragon BLE motor test: ramp collective low -> high, then hard stop."""

import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "Pendragon"
SERVICE_UUID = "ffeeddcc-bbaa-9988-7766-554433221100"
COMMAND_CHAR = "00ffeedd-ccbb-aa99-8877-665544332211"
TELEMETRY_CHAR = "11223344-5566-7788-99aa-bbccddeeff00"

OP_POWER_UP = 0xA0
OP_POWER_DOWN = 0xA1
OP_PING = 0xB0
OP_INFO = 0xB1
OP_MOTOR_STATUS = 0xB3

START = time.monotonic()


def log(msg):
    print(f"[{time.monotonic() - START:7.2f}s] {msg}", flush=True)


def on_telemetry(_char, data: bytearray):
    try:
        text = data.decode("utf-8", errors="replace")
    except Exception:
        text = data.hex()
    log(f"TELEMETRY <- {text}")


async def write(client, payload, label):
    log(f"CMD -> {label} {bytes(payload).hex()}")
    await client.write_gatt_char(COMMAND_CHAR, bytes(payload), response=True)


async def adjust_collective(client, delta):
    """Send POWER_UP/POWER_DOWN steps (1-byte step, 0 means default so avoid it)."""
    op = OP_POWER_UP if delta > 0 else OP_POWER_DOWN
    remaining = abs(delta)
    while remaining > 0:
        step = min(remaining, 250)
        await write(client, [op, step], "POWER_UP" if delta > 0 else "POWER_DOWN")
        remaining -= step
        await asyncio.sleep(0.15)


async def main():
    log(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=20.0)
    if device is None:
        log("Device not found. Is the drone powered on and advertising?")
        sys.exit(2)
    log(f"Found {device.name} [{device.address}], connecting...")

    async with BleakClient(device, timeout=20.0) as client:
        log("Connected.")
        await client.start_notify(TELEMETRY_CHAR, on_telemetry)
        await asyncio.sleep(0.3)

        await write(client, [OP_PING], "PING")
        await asyncio.sleep(0.5)
        await write(client, [OP_INFO], "INFO")
        await asyncio.sleep(0.5)
        await write(client, [OP_MOTOR_STATUS], "MOTOR_STATUS (baseline)")
        await asyncio.sleep(0.5)

        collective = 0
        try:
            # Kick above the spin-up threshold (motors spun at 400 previously),
            # then settle back down to a slow observable speed.
            for target, hold in ((400, 2.0), (310, 15.0), (340, 10.0)):
                log(f"=== Target collective {target}/1000 ===")
                await adjust_collective(client, target - collective)
                collective = target
                await asyncio.sleep(hold)
                await write(client, [OP_MOTOR_STATUS], f"MOTOR_STATUS @ {target}")
                await asyncio.sleep(0.5)
        finally:
            # Hard stop: drive collective well past zero (firmware clamps and
            # zeroes PWM output when a POWER_DOWN lands on 0).
            log("=== Stopping motors ===")
            for _ in range(5):
                await write(client, [OP_POWER_DOWN, 250], "POWER_DOWN")
                await asyncio.sleep(0.15)
            await asyncio.sleep(0.5)
            await write(client, [OP_MOTOR_STATUS], "MOTOR_STATUS (after stop)")
            await asyncio.sleep(1.0)
            await client.stop_notify(TELEMETRY_CHAR)

    log("Disconnected. Test complete.")


if __name__ == "__main__":
    asyncio.run(main())
