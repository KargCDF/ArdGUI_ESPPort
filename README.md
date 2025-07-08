# ArdGUI_ESPPort

## Overview
This project implements a Stepper‑Servo controller for the ESP32. It drives a standard stepper driver (such as a TB6600) together with an SG90/MG90S style servo motor. A small web UI is served from the ESP32 over Wi‑Fi and communicates via WebSocket using the same binary protocol as the serial port.

## Prerequisites
- ESP32 development board (`esp32dev` in `platformio.ini`)
- Stepper driver with **STEP**, **DIR** and **EN** pins (e.g. TB6600) and compatible stepper motor
- SG90/MG90S servo connected to pin 13
- Normally closed limit switch on pin 33
- A recent Node.js & npm installation
- [PlatformIO](https://platformio.org) command‑line tools

## Building the Web Assets
The web interface is written in TypeScript under `websrc/` and compiled to JavaScript in `data/js/`:
```bash
npm install        # only once
npm run build-web  # produces files in data/js
```
These files are uploaded to SPIFFS when flashing.

## Flashing the Firmware
Use PlatformIO to build and upload both the firmware and the SPIFFS data:
```bash
# build and flash the application
pio run -t upload

# upload the contents of the data/ folder to SPIFFS
pio run -t uploadfs
```
After flashing, the board creates an open access point `ArdGUI_ESP32`. The web interface is available at `http://am.local` or via the IP address shown in the serial monitor.
