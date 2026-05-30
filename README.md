# FIRMWARE - ESP32 Water Level Monitoring

This repository contains the firmware source code for an ESP32-based IoT device designed to monitor water level, store measurement data, and send telemetry data to an IoT platform via MQTT.

## 1. Project Overview

The main purpose of this project is to build a low-power water level monitoring device using ESP32. The device collects sensor data, stores records locally, and uploads data to ThingsBoard Cloud through MQTT.

The firmware is designed for field monitoring applications such as rice field water level tracking and other remote water measurement use cases.

## 2. Main Features

* Read water level data from sensor
* Store measurement records locally
* Send telemetry data to ThingsBoard via MQTT
* Retry sending unsent records when network connection is available
* Support ESP-IDF project structure
* Support low-power operation using sleep/deep sleep
* Provide a basic state-machine-based firmware flow

## 3. Technologies Used

* ESP32 / ESP32-C3
* ESP-IDF v5.3.1
* FreeRTOS
* MQTT
* ThingsBoard Cloud
* SD Card / FATFS
* VS Code
* Git / GitHub

## 4. Project Structure

```text
water_level_project/
├── main/
│   ├── CMakeLists.txt
│   └── source files
├── CMakeLists.txt
├── sdkconfig
├── .gitignore
└── README.md
```

## 5. Firmware Workflow

The general firmware flow is:

```text
Boot
→ Initialize system
→ Read water level sensor
→ Save data locally
→ Connect to network
→ Connect to MQTT broker
→ Send telemetry data
→ Send pending unsent records
→ Enter sleep/deep sleep mode
```

## 6. ThingsBoard MQTT Configuration

The device sends telemetry data to ThingsBoard Cloud using MQTT.

Default MQTT information:

```text
Host: mqtt.thingsboard.cloud
Port: 1883
Topic: v1/devices/me/telemetry
Username: Device Access Token
Password: Empty
```

Example telemetry payload:

```json
{
  "water_level": 120,
  "temperature": 25
}
```

## 7. Build Instructions

Open ESP-IDF terminal or VS Code terminal and run:

```bat
cd /d "C:\lab project 1\water_level_project"
idf.py set-target esp32c3
idf.py build
```

If you are using a standard ESP32 board instead of ESP32-C3, use:

```bat
idf.py set-target esp32
idf.py build
```

## 8. Flash Instructions

Connect the ESP32 board to your computer and run:

```bat
idf.py -p COM4 flash monitor
```

Replace `COM4` with the correct COM port of your board.

Example:

```bat
idf.py -p COM5 flash monitor
```

## 9. Git Notes

The `build/` directory should not be pushed to GitHub because it contains generated build files.

Recommended `.gitignore` content:

```gitignore
build/
managed_components/
sdkconfig.old
.vscode/
```

## 10. Project Status

This project is currently under development and testing. The firmware will continue to be improved for stability, power efficiency, MQTT communication, and field deployment.
