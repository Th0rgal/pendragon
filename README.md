# Pendragon

This project is firmware for a custom-built drone, "Pendragon". It's developed using the ESP-IDF framework for an ESP32-S3 microcontroller module (specifically an N18R8 variant). The drone utilizes various readily available electronic components for its flight systems.

## Overview

The primary goal of this firmware is to enable flight control and telemetry via a Bluetooth Low Energy (BLE) interface. The software is custom-written to manage sensors, motor control, and communication.

## Hardware

*   **Microcontroller:** ESP32-S3 (N18R8 variant)
*   **Sensors:** ICM-42688-P IMU (SPI)
*   **Motors & ESCs:** UAngel 4-in-1 45A BLHeli_S (3-6S)
*   **Frame & Power:** ~4S LiPo → ESC "V" 15V → buck → 5V

See [HARDWARE.md](HARDWARE.md) for the full wiring map, protocols, bench
findings, and battery-monitoring plan.

## Bluetooth LE (BLE) Interface

The drone communicates with a companion app (e.g., on a smartphone) using BLE.

*   **Device Name:** `Pendragon`
*   **Service UUID:** `ffeeddcc-bbaa-9988-7766-554433221100`
*   **Command Characteristic UUID (write):** `00ffeedd-ccbb-aa99-8877-665544332211`
*   **Telemetry Characteristic UUID (notify/read):** `11223344-5566-7788-99aa-bbccddeeff00`

*   **App to Drone (Write):** write `[opcode, ...args]` to the command
    characteristic. Opcode families: `0xA0-0xA1` collective power, `0xB0-0xB5`
    ping/info/telemetry/debug, `0xC0-0xC4` resumable OTA updates, `0xD0-0xD5`
    ESC configuration (DShot direction, per-motor trim, probes). Full table in
    [HARDWARE.md](HARDWARE.md), definitions in `main/ble_protocol.h`.
*   **Drone to App (Notify):** telemetry/log lines are sent as UTF-8 notifications on the telemetry characteristic.

## Tooling

`tools/esc_tool.py` (run via `uv run tools/esc_tool.py`) drives everything over
BLE from a laptop: firmware OTA, motor tests with IMU monitoring, ESC direction
configuration/probing, thrust trims, telemetry streaming and the firmware event
log. `tools/motor_test.py` is a simpler PWM-mode collective ramp test.

## Building and Flashing

This project uses the ESP-IDF build system. Standard ESP-IDF commands can be used:

1.  **Configure:** `idf.py menuconfig` (to set serial port, SDK options, etc.)
2.  **Build:** `idf.py build`
3.  **Flash:** `idf.py -p /dev/ttyUSB0 flash` (replace `/dev/ttyUSB0` with your port)
4.  **Monitor:** `idf.py -p /dev/ttyUSB0 monitor`
