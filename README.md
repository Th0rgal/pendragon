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
*   **Characteristic UUID (for commands and telemetry):** `00ffeedd-ccbbaa-9988-7766-5544332211`

This characteristic is used for bidirectional communication:
*   **App to Drone (Write):** The app sends commands to the drone by writing to this characteristic. The command structure is as follows:
    ```c
    typedef struct
    {
        uint8_t throttle; // 0-255 (0% to 100% power)
        uint8_t pitch;    // 0-255 (128 is neutral, <128 backward, >128 forward)
        uint8_t roll;     // 0-255 (128 is neutral, <128 left, >128 right)
        uint8_t yaw;      // 0-255 (128 is neutral, <128 rotate left, >128 rotate right)
        uint8_t mode;     // Flight mode or other custom flags
    } ble_command_t;
    ```
*   **Drone to App (Notify):** The drone can send telemetry data (sensor readings, status, etc.) to the app via notifications on this same characteristic. The app must subscribe to these notifications to receive the data. (The exact structure of telemetry data can be defined here as it's implemented).

## Building and Flashing

This project uses the ESP-IDF build system. Standard ESP-IDF commands can be used:

1.  **Configure:** `idf.py menuconfig` (to set serial port, SDK options, etc.)
2.  **Build:** `idf.py build`
3.  **Flash:** `idf.py -p /dev/ttyUSB0 flash` (replace `/dev/ttyUSB0` with your port)
4.  **Monitor:** `idf.py -p /dev/ttyUSB0 monitor`
