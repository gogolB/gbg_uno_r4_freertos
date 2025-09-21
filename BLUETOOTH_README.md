# GBG UNO R4 FreeRTOS Robot - Bluetooth Integration

## Overview
This project adds Bluetooth functionality to the GBG robot, allowing remote control via Bluetooth Low Energy (BLE).

## Bluetooth Features

### Remote Control
- **Joystick Input**: Receive X,Y axis values from remote Bluetooth device
- **Stop Command**: Emergency stop functionality via Bluetooth
- **Input Priority**: Bluetooth inputs override local joystick when connected

### Device Management
- **Automatic Pairing**: First connected device is automatically paired and remembered
- **Persistent Storage**: Paired device information stored in flash memory
- **Auto-reconnection**: Robot attempts to reconnect to known devices on startup

### Configuration
Access Bluetooth settings through the serial menu (send 'm'):
1. **Enable/Disable Bluetooth**: Turn Bluetooth functionality on/off
2. **Override Local Input**: Configure whether Bluetooth overrides local joystick
3. **Timeout Settings**: Configure connection timeout (1-30 seconds)
4. **Pairing Mode**: Enter pairing mode to connect new devices
5. **Device Management**: Clear paired devices or view device information

## Bluetooth Protocol

### Service UUID
- Service: `12345678-1234-1234-1234-123456789abc`

### Characteristics
1. **Joystick Data** (`12345678-1234-1234-1234-123456789abd`)
   - Format: 8 bytes (X low, X high, Y low, Y high + 4 padding bytes)
   - X,Y values: 16-bit signed integers (-100 to +100 range)
   - Properties: Read, Write, Notify

2. **Stop Command** (`12345678-1234-1234-1234-123456789abe`)
   - Format: 1 byte (0 = no stop, non-zero = stop)
   - Properties: Read, Write, Notify

## Usage

### Pairing a New Device
1. Send 'm' via serial to open configuration menu
2. Select "4. Bluetooth Settings"
3. Select "4. Enter Pairing Mode"
4. Connect from your Bluetooth device to "GBG Robot"
5. Device will be automatically paired and saved

### Sending Commands
- **Joystick Control**: Write 8-byte array to joystick characteristic
- **Emergency Stop**: Write non-zero value to stop characteristic

### Configuration Options
- **Bluetooth Enabled**: Enable/disable Bluetooth functionality
- **Override Local**: Whether Bluetooth input overrides local joystick
- **Timeout**: How long to wait for Bluetooth data before switching back to local input

## Implementation Details

### Thread Safety
- Uses FreeRTOS mutexes to protect shared motor power and Bluetooth input variables
- Bluetooth inputs are checked atomically in the joystick thread

### Priority System
When Bluetooth is enabled and override is configured:
1. Check for Bluetooth input first
2. If valid Bluetooth data available, use it instead of local joystick
3. If no Bluetooth data or timeout, fall back to local joystick
4. Stop commands are processed immediately regardless of other input

### Persistence
Bluetooth configuration and paired device information is stored in ESP32 Preferences:
- Device name and MAC address
- Bluetooth enable/disable state
- Override and timeout settings