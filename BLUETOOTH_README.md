# GBG UNO R4 FreeRTOS Robot - Bluetooth Integration

## Overview
This project provides Bluetooth Low Energy (BLE) control and configuration for the GBG power chair retrofit running on an Arduino UNO R4 with FreeRTOS. Two ready-to-flash sketches now ship side by side:

- **`gbg-freertos.ino` (Joystick Build)** – retains the original analog joystick task and allows the companion Bluetooth app to steer with remote X/Y data. Local joystick input can still be used when Bluetooth is disabled or times out.
- **`gbg-freertos-footpedal.ino` (Foot-Pedal Build)** – replaces the joystick task with a digital foot pedal throttle and forward/reverse toggle switch. Bluetooth is only used for the emergency stop characteristic; pedal and toggle inputs always determine direction.

### Choosing a Sketch
- Flash **`gbg-freertos.ino`** when the mobility base is equipped with the stock joystick pod or another analog stick that feeds the UNO R4.
- Flash **`gbg-freertos-footpedal.ino`** when the joystick has been removed and driving is handled by a normally-open foot pedal plus a direction selector switch. This build reuses the same motor ramp/limit logic as the joystick version so parental speed limits still apply.

## Bluetooth Features

### Common Features
- **Stop Command:** Both sketches expose a stop characteristic that instantly zeroes motor power when a non-zero value is written.
- **Device Management:** Paired device info is stored in flash and reused on reboot. The serial menu can clear the pairing or re-enter pairing mode.
- **Timeout Handling:** Configurable timeout releases the stop command if no BLE update is received for the configured window.

### Additional Joystick-Build Features (`gbg-freertos.ino`)
- **Remote Joystick Input:** BLE X/Y values can drive the motors when override is enabled.
- **Input Priority:** Remote data can take priority over the local joystick until a timeout occurs.

### Additional Foot-Pedal-Build Features (`gbg-freertos-footpedal.ino`)
- **Digital Pedal Mapping:** Reads a debounced foot pedal switch and forward/reverse toggle, translating them into motor commands that respect configured ramps and limits.
- **Local Priority:** Direction and throttle always come from the pedal/toggle hardware. The BLE stop command is the only remote override and can be disabled entirely via the serial menu.

## Bluetooth Protocol
- **Service UUID:** `12345678-1234-1234-1234-123456789abc`
- **Stop Characteristic (`12345678-1234-1234-1234-123456789abe`):** Single byte where zero releases the stop and non-zero engages it.
- **Joystick Characteristic (`12345678-1234-1234-1234-123456789abd`):** Only used by the joystick build; provides signed 16-bit X/Y values encoded in an 8-byte payload.

## Configuration via Serial Menu
Send `m` over the USB serial console to open the configuration UI. Relevant options include:

1. **Motor Settings:** Adjust maximum motor power, ramp increments, and loop timing.
2. **Foot Pedal Settings (foot-pedal build) / Joystick Settings (joystick build):** Configure input polarity, debounce timing, and loop cadence for the selected control hardware.
3. **Bluetooth Settings:** Enable/disable BLE, toggle the "Allow Stop Command" option, configure timeout, manage paired devices, and re-enter pairing mode.
4. **Save/Reset:** Persist changes to flash or revert to defaults.

## Pairing and Command Flow
1. Enter the Bluetooth settings menu and choose "Enter Pairing Mode" to clear any stored device.
2. Connect from the companion app to the advertised "GBG Robot" peripheral.
3. Joystick build clients can stream joystick packets to the joystick characteristic; both builds can trigger the stop by writing a non-zero byte to the stop characteristic.
4. When "Allow Stop Command" is disabled (foot-pedal build default is enabled), incoming stop writes are ignored and the characteristic is reset to zero.

## Implementation Notes
- All shared state between FreeRTOS tasks is protected with critical sections.
- Motor ramping, parental speed limits, and Bluetooth storage reuse the same code paths in both sketches, ensuring consistent behaviour regardless of input hardware.
