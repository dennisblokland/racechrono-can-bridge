# ESP32 RaceChrono BLE Gateway

ESP32 device that reads CAN bus data from your car and sends it wirelessly to the RaceChrono app for telemetry logging.

## Hardware

Uses the [LILYGO T-CAN485](https://lilygo.cc/products/t-can485) board with built-in CAN transceiver.

## Setup

1. Install [PlatformIO](https://platformio.org/)
2. Open this project in PlatformIO
3. Edit `src/config.h` to set your CAN bus speed
4. Upload to the T-CAN485 board

## Usage

1. Connect T-CAN485 to your car's CAN bus (CAN_H, CAN_L, GND)
2. Power the device from 12V
3. In RaceChrono app: Settings → OBD-II & CAN → Add DIY BLE device
4. Connect to "BLE CAN device demo"