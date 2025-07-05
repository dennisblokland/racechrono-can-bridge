# ESP32 RaceChrono BLE Gateway

ESP32 device that reads CAN bus data from your car and sends it wirelessly to the RaceChrono app for telemetry logging.

## Features

- **CAN Bus to BLE Bridge**: Reads CAN bus data and transmits it to RaceChrono app via Bluetooth Low Energy
- **Over-the-Air (OTA) Updates**: Update firmware wirelessly via WiFi using a web interface
- **Configurable Update Rates**: Control how frequently different CAN messages are sent to reduce bandwidth usage
- **Hardware Abstraction**: Works with LILYGO T-CAN485 board with built-in CAN transceiver

## Hardware

Uses the [LILYGO T-CAN485](https://lilygo.cc/products/t-can485) board with built-in CAN transceiver.

## Setup

1. Install [PlatformIO](https://platformio.org/)
2. Clone this repository
3. Open this project in PlatformIO
4. Configure your WiFi credentials (see WiFi Configuration section below)
5. Edit `src/config.h` to set your CAN bus speed and configuration
6. Upload to the T-CAN485 board

### WiFi Configuration (for OTA Updates)

To enable Over-the-Air updates:

1. Copy `src/wifi_config.h.template` to `src/wifi_config.h`
2. Edit `src/wifi_config.h` and set your WiFi credentials:
   ```cpp
   #define WIFI_SSID "YourWiFiName"
   #define WIFI_PASSWORD "YourWiFiPassword"
   ```
3. Optionally, set OTA authentication (recommended for security):
   ```cpp
   #define OTA_USERNAME "admin"
   #define OTA_PASSWORD "your_ota_password"
   ```

**Security Note**: The `wifi_config.h` file is automatically git-ignored to protect your credentials. Never commit WiFi passwords to version control.

## Usage

1. Connect T-CAN485 to your car's CAN bus (CAN_H, CAN_L, GND)
2. Power the device from 12V
3. In RaceChrono app: Settings → OBD-II & CAN → Add DIY BLE device
4. Connect to "BLE CAN device demo"

## OTA Updates

Once WiFi is configured and the device is connected to your network:

1. Find the device's IP address in the serial output
2. Open a web browser and navigate to `http://<device-ip-address>`
3. Upload new firmware (.bin file) through the web interface
4. The device will automatically reboot with the new firmware

**Note**: OTA updates require the device to be connected to WiFi. If WiFi is not configured, the device will work normally but OTA updates will be disabled.