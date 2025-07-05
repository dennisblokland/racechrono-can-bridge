# Over-The-Air (OTA) Updates Setup Guide

This document provides detailed instructions for setting up and using the OTA update functionality in the ESP32 RaceChrono CAN Bridge.

## Overview

The OTA (Over-The-Air) update feature allows you to update the firmware on your ESP32 device wirelessly through a web interface. This eliminates the need to physically connect your device to a computer for firmware updates.

**Important**: OTA updates are available even when the device is waiting for a Bluetooth connection. This allows for remote troubleshooting and updates without requiring a device connection first.

## Requirements

- ESP32 device with WiFi capability (LILYGO T-CAN485 board)
- WiFi network with internet access
- Web browser on the same network as the device

## Setup Instructions

### 1. WiFi Configuration

1. **Copy the template file:**
   ```bash
   cp src/wifi_config.h.template src/wifi_config.h
   ```

2. **Edit the configuration:**
   Open `src/wifi_config.h` and modify the following settings:
   
   ```cpp
   // Your WiFi network credentials
   #define WIFI_SSID "YourWiFiNetworkName"
   #define WIFI_PASSWORD "YourWiFiPassword"
   
   // OTA Web Server Configuration
   #define OTA_PORT 80                    // Port for the web interface
   #define OTA_USERNAME "admin"           // Optional: Username for authentication
   #define OTA_PASSWORD "your_password"   // Optional: Password for authentication
   ```

### 2. Security Configuration (Recommended)

For security, it's recommended to enable authentication:

```cpp
#define OTA_USERNAME "admin"
#define OTA_PASSWORD "your_secure_password"
```

**Security Notes:**
- Use a strong password for OTA authentication
- The `wifi_config.h` file is automatically excluded from git to protect your credentials
- Never commit WiFi passwords or OTA credentials to version control

### 3. Build and Upload

1. **Build the project:**
   ```bash
   pio run
   ```

2. **Upload to device:**
   ```bash
   pio run --target upload
   ```

### 4. Verify WiFi Connection

1. **Open Serial Monitor:**
   ```bash
   pio device monitor
   ```

2. **Check for WiFi connection messages:**
   ```
   ESP32 RaceChrono CAN Bridge starting...
   Setting up WiFi...
   WiFi connected! IP address: 192.168.1.100
   OTA ready!
   Open http://192.168.1.100:80 in your browser to upload firmware
   ```

3. **Note the IP address** - you'll need this for OTA updates

## Using OTA Updates

### 1. Access the OTA Interface

1. **Open a web browser** on the same network as your device
2. **Navigate to** `http://<device-ip-address>` (e.g., `http://192.168.1.100`)
3. **Enter credentials** if authentication is enabled

### 2. Upload New Firmware

1. **Build firmware binary:**
   ```bash
   pio run
   ```
   The firmware binary will be created at `.pio/build/esp32dev/firmware.bin`

2. **Upload via web interface:**
   - Click "Choose File" and select the firmware.bin file
   - Click "Upload" to start the OTA update
   - Wait for the upload to complete (progress will be shown)
   - The device will automatically reboot with the new firmware

### 3. Monitor Update Process

The device will show OTA progress in the serial monitor:
```
OTA update started!
OTA Progress: 25%
OTA Progress: 50%
OTA Progress: 75%
OTA Progress: 100%
OTA update completed successfully!
```

## Troubleshooting

### WiFi Connection Issues

**Problem:** WiFi connection failed
**Solutions:**
- Verify SSID and password in `wifi_config.h`
- Check if the WiFi network is in range
- Ensure the network uses WPA/WPA2 security (WEP is not supported)

### OTA Web Interface Not Accessible

**Problem:** Cannot access OTA web interface
**Solutions:**
- Verify the device IP address in serial monitor
- Ensure your computer is on the same network
- Check if port 80 is blocked by firewall
- Try using a different port in `wifi_config.h`

### Upload Failures

**Problem:** Firmware upload fails
**Solutions:**
- Use the correct firmware.bin file from `.pio/build/esp32dev/`
- Ensure stable WiFi connection during upload
- Try uploading a smaller file first to test connectivity
- Check available flash memory on the device

## Fallback Options

### Serial Upload

If OTA update fails, you can always fall back to serial upload:
```bash
pio run --target upload
```

### Reset to Factory Settings

To reset WiFi settings:
1. Delete `src/wifi_config.h`
2. Rebuild and upload via serial connection
3. Reconfigure WiFi settings

## Advanced Configuration

### Custom Port

To use a different port (e.g., 8080):
```cpp
#define OTA_PORT 8080
```

### Multiple Networks

For multiple WiFi networks, modify the `setupWiFi()` function to try multiple credentials.

### Static IP

To use a static IP address, modify the `setupWiFi()` function:
```cpp
WiFi.config(IPAddress(192, 168, 1, 100), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
```

## Security Best Practices

1. **Always use authentication** in production deployments
2. **Use strong passwords** for OTA access
3. **Limit network access** to the OTA interface when possible
4. **Regularly update firmware** to include security patches
5. **Monitor device logs** for unauthorized access attempts

## Performance Considerations

- OTA updates temporarily pause CAN-BLE bridge functionality
- Large firmware files may take several minutes to upload
- WiFi connection quality affects upload speed and reliability
- The device will automatically resume normal operation after OTA completion