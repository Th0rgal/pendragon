# Pendragon

This project is firmware for a custom-built drone, "Pendragon". It's developed using the ESP-IDF framework for an ESP32-S3 microcontroller module (specifically an N18R8 variant). The drone utilizes various readily available electronic components for its flight systems.

## Overview

The primary goal of this firmware is to enable flight control and telemetry via a Bluetooth Low Energy (BLE) interface. The software is custom-written to manage sensors, motor control, and communication.

## Hardware

*   **Microcontroller:** ESP32-S3 (N18R8 variant)
*   **Sensors:** todo
*   **Motors & ESCs:** todo
*   **Frame & Power:** todo

## Bluetooth LE (BLE) Interface

The drone communicates with a companion app (e.g., on a smartphone) using BLE.

*   **Device Name:** `Pendragon`
*   **Service UUID:** `ffeeddcc-bbaa-9988-7766-554433221100`
*   **Command Characteristic UUID (write):** `00ffeedd-ccbb-aa99-8877-665544332211`
*   **Telemetry Characteristic UUID (notify/read):** `11223344-5566-7788-99aa-bbccddeeff00`

The BLE protocol is intentionally small while the motor mapping is being tuned:
*   **App to Drone (Write):** write `[opcode]` or `[opcode, step]` to the command characteristic.
    *   `0xA0`: power up by `step` units.
    *   `0xA1`: power down by `step` units.
    *   `step` defaults to `50` when omitted or zero and is clamped in firmware to the 0-1000 motor power domain.
*   **Drone to App (Notify):** telemetry/log lines are sent as UTF-8 notifications on the telemetry characteristic.

## Building and Flashing

This project uses the ESP-IDF build system. Standard ESP-IDF commands can be used:

1.  **Configure:** `idf.py menuconfig` (to set serial port, SDK options, etc.)
2.  **Build:** `idf.py build`
3.  **Flash:** `idf.py -p /dev/ttyUSB0 flash` (replace `/dev/ttyUSB0` with your port)
4.  **Monitor:** `idf.py -p /dev/ttyUSB0 monitor`
