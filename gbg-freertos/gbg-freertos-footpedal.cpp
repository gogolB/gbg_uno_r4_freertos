/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

// Foot pedal build of the GBG UNO R4 FreeRTOS robot control sketch.
// Use this variant when the drive base is operated with a digital foot pedal throttle
// and a forward/reverse toggle switch. The joystick-centric sketch remains available
// alongside this file as `gbg-freertos.ino` for chair configurations that still use
// the analog joystick assembly.

#include "gbg-variant.h"

struct RobotConfig;
struct MotorConfigUpdate;

#if GBG_VARIANT_FOOT_PEDAL

#include <Arduino.h>
#include <Arduino_FreeRTOS.h> // DO NOT Install this one, this one exists automatically for Uno R4+
#include <LibPrintf.h> // Install this one
#include <DualVNH5019MotorShield.h> // Install this one
#include <ArduinoBLE.h> // Install this one
#include <Preferences.h> // Install this one.

// Lightweight protection for short shared-variable updates
#define CRIT_BEGIN()  taskENTER_CRITICAL()
#define CRIT_END()    taskEXIT_CRITICAL()
 
/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

// Configuration structure for all configurable parameters
struct RobotConfig {
  // Motor settings
  int maxFwdLeftMotorPower;
  int maxFwdRightMotorPower;
  int maxBwdLeftMotorPower;
  int maxBwdRightMotorPower;
  bool revLeftDrive;
  bool revRightDrive;
  bool swapMotors;
  int motorPowerIncrement;
  int motorLoopDelayMs;
  int timeBtwnMotorIncrementMs;
  
  // Foot pedal settings
  int footPedalPin;
  bool footPedalActiveHigh;
  int directionTogglePin;
  bool directionToggleActiveHigh;
  int pedalPressedPower;
  int pedalDebounceMs;
  int toggleDebounceMs;
  int footPedalLoopDelayMs;
  
  // Bluetooth settings
  bool bluetoothEnabled;
  char pairedDeviceName[32];
  char pairedDeviceAddress[18];
  bool bluetoothAllowStopCommand;
  bool bluetoothOverrideLocal;
  int bluetoothTimeoutMs;
};

// Global configuration instance
RobotConfig config;
Preferences preferences;

// Initialize default configuration values
void initDefaultConfig() {
  // Motor settings defaults
  config.maxFwdLeftMotorPower = 100;
  config.maxFwdRightMotorPower = 100;
  config.maxBwdLeftMotorPower = -100;
  config.maxBwdRightMotorPower = -100;
  config.revLeftDrive = false;
  config.revRightDrive = false;
  config.swapMotors = false;
  config.motorPowerIncrement = 5;
  config.motorLoopDelayMs = 10;
  config.timeBtwnMotorIncrementMs = 100;
  
  // Foot pedal settings defaults
  config.footPedalPin = 2;
  config.footPedalActiveHigh = false;
  config.directionTogglePin = 3;
  config.directionToggleActiveHigh = true;
  config.pedalPressedPower = 100;
  config.pedalDebounceMs = 25;
  config.toggleDebounceMs = 50;
  config.footPedalLoopDelayMs = 10;
  
  // Bluetooth settings defaults
  config.bluetoothEnabled = true;
  strcpy(config.pairedDeviceName, "");
  strcpy(config.pairedDeviceAddress, "");
  config.bluetoothAllowStopCommand = true;
  config.bluetoothOverrideLocal = true;
  config.bluetoothTimeoutMs = 5000;
}

// Load configuration from preferences
void loadConfig() {
  preferences.begin("robot_config", false);
  
  // Load motor settings
  config.maxFwdLeftMotorPower = preferences.getInt("maxFwdLeftPwr", config.maxFwdLeftMotorPower);
  config.maxFwdRightMotorPower = preferences.getInt("maxFwdRightPwr", config.maxFwdRightMotorPower);
  config.maxBwdLeftMotorPower = preferences.getInt("maxBwdLeftPwr", config.maxBwdLeftMotorPower);
  config.maxBwdRightMotorPower = preferences.getInt("maxBwdRightPwr", config.maxBwdRightMotorPower);
  config.revLeftDrive = preferences.getBool("revLeftDrive", config.revLeftDrive);
  config.revRightDrive = preferences.getBool("revRightDrive", config.revRightDrive);
  config.swapMotors = preferences.getBool("swapMotors", config.swapMotors);
  config.motorPowerIncrement = preferences.getInt("motorPwrInc", config.motorPowerIncrement);
  config.motorLoopDelayMs = preferences.getInt("motorLoopDelay", config.motorLoopDelayMs);
  config.timeBtwnMotorIncrementMs = preferences.getInt("motorIncTime", config.timeBtwnMotorIncrementMs);
  
  // Load foot pedal settings
  config.footPedalPin = preferences.getInt("pedalPin", config.footPedalPin);
  config.footPedalActiveHigh = preferences.getBool("pedalActiveHi", config.footPedalActiveHigh);
  config.directionTogglePin = preferences.getInt("dirTogglePin", config.directionTogglePin);
  config.directionToggleActiveHigh = preferences.getBool("dirToggleHi", config.directionToggleActiveHigh);
  config.pedalPressedPower = preferences.getInt("pedalPower", config.pedalPressedPower);
  config.pedalDebounceMs = preferences.getInt("pedalDbnc", config.pedalDebounceMs);
  config.toggleDebounceMs = preferences.getInt("toggleDbnc", config.toggleDebounceMs);
  config.footPedalLoopDelayMs = preferences.getInt("pedalLoop", config.footPedalLoopDelayMs);
  
  // Load Bluetooth settings
  config.bluetoothEnabled = preferences.getBool("btEnabled", config.bluetoothEnabled);
  preferences.getString("btDeviceName", config.pairedDeviceName, sizeof(config.pairedDeviceName));
  preferences.getString("btDeviceAddr", config.pairedDeviceAddress, sizeof(config.pairedDeviceAddress));
  config.bluetoothAllowStopCommand = preferences.getBool("btAllowStop", config.bluetoothAllowStopCommand);
  config.bluetoothOverrideLocal = preferences.getBool("btOverrideLocal", config.bluetoothOverrideLocal);
  config.bluetoothTimeoutMs = preferences.getInt("btTimeout", config.bluetoothTimeoutMs);
  
  preferences.end();
}

// Save configuration to preferences
void saveConfig() {
  preferences.begin("robot_config", false);
  
  // Save motor settings
  preferences.putInt("maxFwdLeftPwr", config.maxFwdLeftMotorPower);
  preferences.putInt("maxFwdRightPwr", config.maxFwdRightMotorPower);
  preferences.putInt("maxBwdLeftPwr", config.maxBwdLeftMotorPower);
  preferences.putInt("maxBwdRightPwr", config.maxBwdRightMotorPower);
  preferences.putBool("revLeftDrive", config.revLeftDrive);
  preferences.putBool("revRightDrive", config.revRightDrive);
  preferences.putBool("swapMotors", config.swapMotors);
  preferences.putInt("motorPwrInc", config.motorPowerIncrement);
  preferences.putInt("motorLoopDelay", config.motorLoopDelayMs);
  preferences.putInt("motorIncTime", config.timeBtwnMotorIncrementMs);
  
  // Save foot pedal settings
  preferences.putInt("pedalPin", config.footPedalPin);
  preferences.putBool("pedalActiveHi", config.footPedalActiveHigh);
  preferences.putInt("dirTogglePin", config.directionTogglePin);
  preferences.putBool("dirToggleHi", config.directionToggleActiveHigh);
  preferences.putInt("pedalPower", config.pedalPressedPower);
  preferences.putInt("pedalDbnc", config.pedalDebounceMs);
  preferences.putInt("toggleDbnc", config.toggleDebounceMs);
  preferences.putInt("pedalLoop", config.footPedalLoopDelayMs);
  
  // Save Bluetooth settings
  preferences.putBool("btEnabled", config.bluetoothEnabled);
  preferences.putString("btDeviceName", config.pairedDeviceName);
  preferences.putString("btDeviceAddr", config.pairedDeviceAddress);
  preferences.putBool("btAllowStop", config.bluetoothAllowStopCommand);
  preferences.putBool("btOverrideLocal", config.bluetoothOverrideLocal);
  preferences.putInt("btTimeout", config.bluetoothTimeoutMs);
  
  preferences.end();
}

TaskHandle_t loop_task, blinky_task, motor_drive_task, foot_pedal_task, menu_task, bluetooth_task;

// Bluetooth globals
BLEService robotControlService("12345678-1234-1234-1234-123456789abc");
BLECharacteristic joystickCharacteristic("12345678-1234-1234-1234-123456789abd", BLERead | BLEWrite | BLENotify, 8);
BLECharacteristic stopCharacteristic("12345678-1234-1234-1234-123456789abe", BLERead | BLEWrite | BLENotify, 1);
BLECharacteristic motorConfigCharacteristic("12345678-1234-1234-1234-123456789abf", BLERead | BLEWrite | BLENotify, 13);

const int16_t MOTOR_POWER_LIMIT = 400;
const uint16_t MOTOR_POWER_INCREMENT_MIN = 1;
const uint16_t MOTOR_POWER_INCREMENT_MAX = 100;
const uint16_t MOTOR_INCREMENT_TIME_MIN = 5;
const uint16_t MOTOR_INCREMENT_TIME_MAX = 2000;

const uint8_t MOTOR_CONFIG_ACK_IDLE = 0x00;
const uint8_t MOTOR_CONFIG_ACK_SUCCESS = 0x01;
const uint8_t MOTOR_CONFIG_ACK_FAILURE = 0x02;
const uint8_t MOTOR_CONFIG_ACK_CURRENT = 0x03;

const size_t MOTOR_CONFIG_PAYLOAD_SIZE = 12;
const size_t MOTOR_CONFIG_RESPONSE_SIZE = 13;

struct MotorConfigUpdate {
  int16_t maxFwdLeftMotorPower;
  int16_t maxFwdRightMotorPower;
  int16_t maxBwdLeftMotorPower;
  int16_t maxBwdRightMotorPower;
  uint16_t motorPowerIncrement;
  uint16_t timeBtwnMotorIncrementMs;
};

void writeInt16LE(uint8_t *buffer, int16_t value) {
  buffer[0] = lowByte(value);
  buffer[1] = highByte(value);
}

void writeUInt16LE(uint8_t *buffer, uint16_t value) {
  buffer[0] = lowByte(value);
  buffer[1] = highByte(value);
}

void packMotorConfigPayload(uint8_t *buffer, const RobotConfig &source) {
  writeInt16LE(&buffer[0], static_cast<int16_t>(source.maxFwdLeftMotorPower));
  writeInt16LE(&buffer[2], static_cast<int16_t>(source.maxFwdRightMotorPower));
  writeInt16LE(&buffer[4], static_cast<int16_t>(source.maxBwdLeftMotorPower));
  writeInt16LE(&buffer[6], static_cast<int16_t>(source.maxBwdRightMotorPower));
  writeUInt16LE(&buffer[8], static_cast<uint16_t>(source.motorPowerIncrement));
  writeUInt16LE(&buffer[10], static_cast<uint16_t>(source.timeBtwnMotorIncrementMs));
}

void publishMotorConfigResponse(uint8_t statusCode) {
  uint8_t response[MOTOR_CONFIG_RESPONSE_SIZE];
  response[0] = statusCode;
  packMotorConfigPayload(&response[1], config);
  motorConfigCharacteristic.writeValue(response, MOTOR_CONFIG_RESPONSE_SIZE);
}

bool decodeMotorConfigPayload(const uint8_t *data, size_t length, MotorConfigUpdate &update, String &errorMessage) {
  if (length != MOTOR_CONFIG_PAYLOAD_SIZE) {
    errorMessage = "Invalid payload length";
    return false;
  }

  update.maxFwdLeftMotorPower = static_cast<int16_t>((data[1] << 8) | data[0]);
  update.maxFwdRightMotorPower = static_cast<int16_t>((data[3] << 8) | data[2]);
  update.maxBwdLeftMotorPower = static_cast<int16_t>((data[5] << 8) | data[4]);
  update.maxBwdRightMotorPower = static_cast<int16_t>((data[7] << 8) | data[6]);
  update.motorPowerIncrement = static_cast<uint16_t>((data[9] << 8) | data[8]);
  update.timeBtwnMotorIncrementMs = static_cast<uint16_t>((data[11] << 8) | data[10]);

  if (update.maxFwdLeftMotorPower < 0 || update.maxFwdLeftMotorPower > MOTOR_POWER_LIMIT) {
    errorMessage = "maxFwdLeftMotorPower out of range";
    return false;
  }
  if (update.maxFwdRightMotorPower < 0 || update.maxFwdRightMotorPower > MOTOR_POWER_LIMIT) {
    errorMessage = "maxFwdRightMotorPower out of range";
    return false;
  }
  if (update.maxBwdLeftMotorPower > 0 || update.maxBwdLeftMotorPower < -MOTOR_POWER_LIMIT) {
    errorMessage = "maxBwdLeftMotorPower out of range";
    return false;
  }
  if (update.maxBwdRightMotorPower > 0 || update.maxBwdRightMotorPower < -MOTOR_POWER_LIMIT) {
    errorMessage = "maxBwdRightMotorPower out of range";
    return false;
  }
  if (update.motorPowerIncrement < MOTOR_POWER_INCREMENT_MIN || update.motorPowerIncrement > MOTOR_POWER_INCREMENT_MAX) {
    errorMessage = "motorPowerIncrement out of range";
    return false;
  }
  if (update.timeBtwnMotorIncrementMs < MOTOR_INCREMENT_TIME_MIN || update.timeBtwnMotorIncrementMs > MOTOR_INCREMENT_TIME_MAX) {
    errorMessage = "timeBtwnMotorIncrementMs out of range";
    return false;
  }

  return true;
}

// Bluetooth input state
struct BluetoothInput {
  bool active;
  int xValue;
  int yValue;
  bool stopCommand;
  unsigned long lastUpdateTime;
};

volatile BluetoothInput bluetoothInput = {false, 0, 0, false, 0};

 void loop()
 {
   vTaskDelay(configTICK_RATE_HZ/4);
 }
 
 void loop_thread_func(void *pvParameters)
 {
   for(;;)
   {
     loop();
     taskYIELD();
   }
 }
 
 // ========================================================
 //                     Blink Thread
 // ========================================================
 
 void blinky_thread_func(void *pvParameters)
 {
   /* setup() */
   pinMode(LED_BUILTIN, OUTPUT);
   digitalWrite(LED_BUILTIN, LOW);
 
   /* loop() */
   for(;;)
   {
     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
     vTaskDelay(configTICK_RATE_HZ);
   }
 }
 
 // ========================================================
 //                Motor Control Thread
 // ========================================================
 // Motor Driver shield definiation
 DualVNH5019MotorShield md;
 
// This is the motor power. A power of greater than 0 is forward.
// A power of less than 0 is backward.
volatile int leftMotorPower = 0;
volatile int rightMotorPower = 0;
 int motorPrintCounter = 0;
 
 void stopIfFault()
 {
   if (md.getM1Fault())
   {
     Serial.println("M1 fault");
     while(1);
   }
   if (md.getM2Fault())
   {
     Serial.println("M2 fault");
     while(1);
   }
 }
 
  /*
   * This is the motor control function that is primarily responsible 
   * for setting motor values once they are calculated.
   */
 
  void motor_drive_func(void *pvParams) 
 {
  // Setup()
  CRIT_BEGIN();
  leftMotorPower = 0;
  rightMotorPower = 0;
  CRIT_END();
 
   md.init();
 
   int leftMotorRequestedPower = 0;
   int rightMotorRequestedPower = 0;
 
   int leftMotorAppliedPower = 0;
   int rightMotorAppliedPower = 0;
   unsigned long last_motor_increment_time = 0;
 
   // loop()
   for (;;)
   {
    int leftPowerSnapshot = 0;
    int rightPowerSnapshot = 0;

    CRIT_BEGIN();
    leftMotorPower = constrain(leftMotorPower, -100, 100);
    rightMotorPower = constrain(rightMotorPower, -100, 100);

    leftPowerSnapshot = leftMotorPower;
    rightPowerSnapshot = rightMotorPower;
    CRIT_END();

    int leftMotorPowerLocal = leftPowerSnapshot;
    int rightMotorPowerLocal = rightPowerSnapshot;

    if (config.revLeftDrive)
      leftMotorPowerLocal = leftMotorPowerLocal * -1;

    if (config.revRightDrive)
      rightMotorPowerLocal = rightMotorPowerLocal * -1;

    if (config.swapMotors) {
      leftMotorPowerLocal = leftMotorPowerLocal ^ rightMotorPowerLocal;
      rightMotorPowerLocal = leftMotorPowerLocal ^ rightMotorPowerLocal;
      leftMotorPowerLocal = leftMotorPowerLocal ^ rightMotorPowerLocal;
    }

    // Smooth startup ramping
    if (abs(leftMotorPowerLocal) > 10) {
      leftMotorRequestedPower = map(leftMotorPowerLocal, -100, 100, config.maxBwdLeftMotorPower, config.maxFwdLeftMotorPower);
      leftMotorRequestedPower = constrain(leftMotorRequestedPower, config.maxBwdLeftMotorPower, config.maxFwdLeftMotorPower);
    } else {
      leftMotorRequestedPower = 0;
    }

    if (abs(rightMotorPowerLocal) > 10) {
      rightMotorRequestedPower = map(rightMotorPowerLocal, -100, 100, config.maxBwdRightMotorPower, config.maxFwdRightMotorPower);
      rightMotorRequestedPower = constrain(rightMotorRequestedPower, config.maxBwdRightMotorPower, config.maxFwdRightMotorPower);
    } else {
      rightMotorRequestedPower = 0;
     }


     // Handle the acceleration and deceleration of the motors.
     if (millis() - last_motor_increment_time > config.timeBtwnMotorIncrementMs) {
        last_motor_increment_time = millis();

        if (leftMotorAppliedPower < leftMotorRequestedPower) {
          leftMotorAppliedPower += min(config.motorPowerIncrement, leftMotorRequestedPower - leftMotorAppliedPower);
        } else if (leftMotorAppliedPower > leftMotorRequestedPower) {
          leftMotorAppliedPower -= min(config.motorPowerIncrement, leftMotorAppliedPower - leftMotorRequestedPower);
        } else {
          leftMotorAppliedPower = leftMotorRequestedPower;
        }

        if (rightMotorAppliedPower < rightMotorRequestedPower) {
          rightMotorAppliedPower += min(config.motorPowerIncrement, rightMotorRequestedPower - rightMotorAppliedPower); 
        } else if (rightMotorAppliedPower > rightMotorRequestedPower) {
          rightMotorAppliedPower -= min(config.motorPowerIncrement, rightMotorAppliedPower - rightMotorRequestedPower);
        } else {
          rightMotorAppliedPower = rightMotorRequestedPower; 
        }
        
        md.setSpeeds(leftMotorAppliedPower, rightMotorAppliedPower);
        stopIfFault();
     }
 
     int left_motor_current = md.getM1CurrentMilliamps();
     int right_motor_current = md.getM2CurrentMilliamps();
 
     if (motorPrintCounter >= 10) {
       //printf("[Motor Dr Thread] LRP=%5d, RRP=%5d, | LMAP=%5d, RMAP=%5d | LM_CUR=%5d mA, RM_CUR=%5d mA \n", 
       //        leftMotorRequestedPower, rightMotorRequestedPower, leftMotorAppliedPower, rightMotorAppliedPower, left_motor_current, right_motor_current);
       motorPrintCounter = 0;
     }
     motorPrintCounter++;
 
     vTaskDelay(config.motorLoopDelayMs / portTICK_PERIOD_MS);
   }
 }
  
 // ========================================================
 //                Foot Pedal Thread
 // ========================================================

void foot_pedal_func(void *pvParams)
{
  // Setup input pins
  pinMode(config.footPedalPin, config.footPedalActiveHigh ? INPUT : INPUT_PULLUP);
  pinMode(config.directionTogglePin, config.directionToggleActiveHigh ? INPUT : INPUT_PULLUP);

  bool stablePedalPressed = false;
  bool lastRawPedalPressed = false;
  unsigned long lastPedalChangeMs = 0;

  bool stableReverseToggle = false;
  bool lastRawReverseToggle = false;
  unsigned long lastToggleChangeMs = 0;

  bool stopNoticeShown = false;

  for (;;)
  {
    const bool rawPedal = digitalRead(config.footPedalPin) == (config.footPedalActiveHigh ? HIGH : LOW);
    if (rawPedal != lastRawPedalPressed) {
      lastRawPedalPressed = rawPedal;
      lastPedalChangeMs = millis();
    }
    if (millis() - lastPedalChangeMs >= static_cast<unsigned long>(config.pedalDebounceMs)) {
      stablePedalPressed = rawPedal;
    }

    const bool rawReverse = digitalRead(config.directionTogglePin) == (config.directionToggleActiveHigh ? HIGH : LOW);
    if (rawReverse != lastRawReverseToggle) {
      lastRawReverseToggle = rawReverse;
      lastToggleChangeMs = millis();
    }
    if (millis() - lastToggleChangeMs >= static_cast<unsigned long>(config.toggleDebounceMs)) {
      stableReverseToggle = rawReverse;
    }

    bool remoteStop = false;
    bool remoteActive = false;
    int remoteX = 0;
    int remoteY = 0;

    CRIT_BEGIN();
    remoteStop = bluetoothInput.stopCommand;
    remoteActive = bluetoothInput.active;
    remoteX = bluetoothInput.xValue;
    remoteY = bluetoothInput.yValue;
    CRIT_END();

    const bool bluetoothOverride = config.bluetoothEnabled && config.bluetoothOverrideLocal && remoteActive && !remoteStop;

    int newLeftPower = 0;
    int newRightPower = 0;

    if (remoteStop) {
      newLeftPower = 0;
      newRightPower = 0;
    } else if (bluetoothOverride) {
      int scaledX = constrain(remoteX, -100, 100);
      int scaledY = constrain(remoteY, -100, 100);

      int maximum = max(abs(scaledY), abs(scaledX));
      int total = scaledY + scaledX;
      int difference = scaledY - scaledX;

      if (scaledY >= 0) {
        if (scaledX >= 0) {
          newLeftPower = maximum;
          newRightPower = difference;
        } else {
          newLeftPower = total;
          newRightPower = maximum;
        }
      } else {
        if (scaledX >= 0) {
          newLeftPower = total;
          newRightPower = -maximum;
        } else {
          newLeftPower = -maximum;
          newRightPower = difference;
        }
      }
    } else {
      int requestedPower = 0;
      if (stablePedalPressed) {
        requestedPower = constrain(config.pedalPressedPower, -100, 100);
        if (stableReverseToggle) {
          requestedPower *= -1;
        }
      }
      newLeftPower = requestedPower;
      newRightPower = requestedPower;
    }

    CRIT_BEGIN();
    leftMotorPower = newLeftPower;
    rightMotorPower = newRightPower;
    CRIT_END();

    if (remoteStop && stablePedalPressed) {
      if (!stopNoticeShown) {
        Serial.println("[Foot Pedal Thread] Remote stop active, holding motors at zero.");
        stopNoticeShown = true;
      }
    } else if (!remoteStop) {
      stopNoticeShown = false;
    }

    vTaskDelay(config.footPedalLoopDelayMs / portTICK_PERIOD_MS);
  }
}
 
 // ========================================================
 //                Bluetooth Thread
 // ========================================================

void bluetooth_func(void *pvParams) {
  Serial.println("[Bluetooth Thread] Initializing Bluetooth...");

  if (!BLE.begin()) {
    Serial.println("[Bluetooth Thread] Failed to initialize BLE!");
    vTaskDelete(nullptr);
    return;
  }

  BLE.setLocalName("GBG Robot");
  BLE.setAdvertisedService(robotControlService);

  robotControlService.addCharacteristic(joystickCharacteristic);
  robotControlService.addCharacteristic(stopCharacteristic);
  robotControlService.addCharacteristic(motorConfigCharacteristic);

  BLE.addService(robotControlService);

  uint8_t joystickData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  joystickCharacteristic.writeValue(joystickData, sizeof(joystickData));

  uint8_t stopData = 0;
  stopCharacteristic.writeValue(stopData);

  publishMotorConfigResponse(MOTOR_CONFIG_ACK_IDLE);

  BLE.advertise();
  Serial.println("[Bluetooth Thread] Bluetooth ready and advertising...");

  unsigned long lastConnectionCheck = 0;
  bool wasConnected = false;

  for (;;) {
    BLE.poll();

    bool isConnected = BLE.connected();

    if (isConnected && !wasConnected) {
      Serial.println("[Bluetooth Thread] Device connected!");
      publishMotorConfigResponse(MOTOR_CONFIG_ACK_CURRENT);

      BLEDevice central = BLE.central();
      if (central && strlen(config.pairedDeviceAddress) == 0) {
        String address = central.address();
        address.toCharArray(config.pairedDeviceAddress, sizeof(config.pairedDeviceAddress));
        String localName = central.localName();
        if (localName.length() > 0) {
          localName.toCharArray(config.pairedDeviceName, sizeof(config.pairedDeviceName));
        } else {
          strcpy(config.pairedDeviceName, "Unknown Device");
        }
        saveConfig();
        Serial.println("[Bluetooth Thread] Device paired and saved: " + String(config.pairedDeviceName));
      }
    } else if (!isConnected && wasConnected) {
      Serial.println("[Bluetooth Thread] Device disconnected!");

      CRIT_BEGIN();
      bluetoothInput.active = false;
      bluetoothInput.stopCommand = false;
      CRIT_END();
      stopCharacteristic.writeValue(static_cast<uint8_t>(0));
    }

    wasConnected = isConnected;

    if (isConnected) {
      if (joystickCharacteristic.written()) {
        uint8_t data[8];
        joystickCharacteristic.readValue(data, sizeof(data));

        int16_t x = (static_cast<int16_t>(data[1]) << 8) | data[0];
        int16_t y = (static_cast<int16_t>(data[3]) << 8) | data[2];

        CRIT_BEGIN();
        bluetoothInput.active = true;
        bluetoothInput.xValue = x;
        bluetoothInput.yValue = y;
        bluetoothInput.lastUpdateTime = millis();
        CRIT_END();
      }

      if (stopCharacteristic.written()) {
        uint8_t stopValue = 0;
        stopCharacteristic.readValue(stopValue);
        const bool asserted = stopValue != 0;

        if (config.bluetoothAllowStopCommand) {
          CRIT_BEGIN();
          bluetoothInput.stopCommand = asserted;
          bluetoothInput.lastUpdateTime = millis();
          CRIT_END();

          Serial.println(asserted ? "[Bluetooth Thread] Stop command asserted!" : "[Bluetooth Thread] Stop command cleared.");
        } else {
          Serial.println("[Bluetooth Thread] Stop command received but disabled in configuration.");
          stopCharacteristic.writeValue(static_cast<uint8_t>(0));
        }
      }

      if (motorConfigCharacteristic.written()) {
        int valueLength = motorConfigCharacteristic.valueLength();
        uint8_t rawData[MOTOR_CONFIG_RESPONSE_SIZE];
        int bytesRead = 0;

        if (valueLength > 0) {
          bytesRead = motorConfigCharacteristic.readValue(rawData, min(valueLength, static_cast<int>(sizeof(rawData))));
        }

        if (valueLength < static_cast<int>(MOTOR_CONFIG_PAYLOAD_SIZE)) {
          Serial.println("[Bluetooth Thread] Motor config snapshot requested by central");
          publishMotorConfigResponse(MOTOR_CONFIG_ACK_CURRENT);
        } else {
          const uint8_t *payloadPtr = rawData;
          size_t payloadLength = bytesRead;

          if (bytesRead == MOTOR_CONFIG_RESPONSE_SIZE) {
            payloadPtr = &rawData[1];
            payloadLength = MOTOR_CONFIG_PAYLOAD_SIZE;
          }

          MotorConfigUpdate update;
          String errorMessage = "";

          if (!decodeMotorConfigPayload(payloadPtr, payloadLength, update, errorMessage)) {
            Serial.println("[Bluetooth Thread] Motor config update rejected: " + errorMessage);
            publishMotorConfigResponse(MOTOR_CONFIG_ACK_FAILURE);
          } else {
            CRIT_BEGIN();
            config.maxFwdLeftMotorPower = update.maxFwdLeftMotorPower;
            config.maxFwdRightMotorPower = update.maxFwdRightMotorPower;
            config.maxBwdLeftMotorPower = update.maxBwdLeftMotorPower;
            config.maxBwdRightMotorPower = update.maxBwdRightMotorPower;
            config.motorPowerIncrement = update.motorPowerIncrement;
            config.timeBtwnMotorIncrementMs = update.timeBtwnMotorIncrementMs;
            CRIT_END();

            saveConfig();

            Serial.print("[Bluetooth Thread] Motor config updated. Fwd L/R: ");
            Serial.print(config.maxFwdLeftMotorPower);
            Serial.print("/");
            Serial.print(config.maxFwdRightMotorPower);
            Serial.print("  Bwd L/R: ");
            Serial.print(config.maxBwdLeftMotorPower);
            Serial.print("/");
            Serial.print(config.maxBwdRightMotorPower);
            Serial.print("  Inc: ");
            Serial.print(config.motorPowerIncrement);
            Serial.print("  Delay: ");
            Serial.println(config.timeBtwnMotorIncrementMs);

            publishMotorConfigResponse(MOTOR_CONFIG_ACK_SUCCESS);
          }
        }
      }

      if (config.bluetoothAllowStopCommand && config.bluetoothTimeoutMs > 0) {
        bool stopActive = false;
        unsigned long stopLastUpdate = 0;

        CRIT_BEGIN();
        stopActive = bluetoothInput.stopCommand;
        stopLastUpdate = bluetoothInput.lastUpdateTime;
        CRIT_END();

        if (stopActive && millis() - stopLastUpdate > static_cast<unsigned long>(config.bluetoothTimeoutMs)) {
          CRIT_BEGIN();
          bluetoothInput.stopCommand = false;
          CRIT_END();
          Serial.println("[Bluetooth Thread] Stop command timeout expired, releasing motors.");
          stopCharacteristic.writeValue(static_cast<uint8_t>(0));
        }
      }

      if (config.bluetoothTimeoutMs > 0) {
        bool inputActive = false;
        unsigned long inputLastUpdate = 0;

        CRIT_BEGIN();
        inputActive = bluetoothInput.active;
        inputLastUpdate = bluetoothInput.lastUpdateTime;
        CRIT_END();

        if (inputActive && millis() - inputLastUpdate > static_cast<unsigned long>(config.bluetoothTimeoutMs)) {
          CRIT_BEGIN();
          bluetoothInput.active = false;
          CRIT_END();
          Serial.println("[Bluetooth Thread] Bluetooth input timeout");
        }
      }
    } else {
      if (strlen(config.pairedDeviceAddress) > 0) {
        if (millis() - lastConnectionCheck > 10000) {
          lastConnectionCheck = millis();
          Serial.println("[Bluetooth Thread] Waiting for known device to reconnect...");
        }
      }
    }

    if (!config.bluetoothAllowStopCommand) {
      bool wasActive = false;
      CRIT_BEGIN();
      wasActive = bluetoothInput.stopCommand;
      bluetoothInput.stopCommand = false;
      CRIT_END();
      if (wasActive) {
        stopCharacteristic.writeValue(static_cast<uint8_t>(0));
      }
    }

    if (!config.bluetoothOverrideLocal) {
      CRIT_BEGIN();
      bluetoothInput.active = false;
      CRIT_END();
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
 // ========================================================
 //                Serial Menu Task
 // ========================================================
 
void printMainMenu() {
  Serial.println("\n==== Robot Configuration Menu ====");
  Serial.println("1. View Current Settings");
  Serial.println("2. Motor Settings");
  Serial.println("3. Foot Pedal Settings");
  Serial.println("4. Bluetooth Settings");
  Serial.println("5. Save Settings to Flash");
  Serial.println("6. Reset to Defaults");
  Serial.println("0. Exit Menu");
  Serial.print("Enter choice: ");
}

void printMotorMenu() {
  Serial.println("\n---- Motor Settings ----");
  Serial.println("1. Max Forward Left Motor Power");
  Serial.println("2. Max Forward Right Motor Power");
  Serial.println("3. Max Backward Left Motor Power");
  Serial.println("4. Max Backward Right Motor Power");
  Serial.println("5. Reverse Left Drive");
  Serial.println("6. Reverse Right Drive");
  Serial.println("7. Swap Motors");
  Serial.println("8. Motor Power Increment");
  Serial.println("9. Motor Loop Delay (ms)");
  Serial.println("10. Time Between Motor Increments (ms)");
  Serial.println("0. Back to Main Menu");
  Serial.print("Enter choice: ");
}

void printFootPedalMenu() {
  Serial.println("\n---- Foot Pedal Settings ----");
  Serial.println("1. Foot Pedal Pin");
  Serial.println("2. Foot Pedal Active High");
  Serial.println("3. Direction Toggle Pin");
  Serial.println("4. Direction Toggle Active High");
  Serial.println("5. Pedal Pressed Power (0-100)");
  Serial.println("6. Pedal Debounce (ms)");
  Serial.println("7. Toggle Debounce (ms)");
  Serial.println("8. Foot Pedal Loop Delay (ms)");
  Serial.println("0. Back to Main Menu");
  Serial.print("Enter choice: ");
}

void printBluetoothMenu() {
  Serial.println("\n---- Bluetooth Settings ----");
  Serial.println("1. Enable/Disable Bluetooth");
  Serial.println("2. Allow Stop Command");
  Serial.println("3. Bluetooth Override Local Input");
  Serial.println("4. Bluetooth Timeout (ms)");
  Serial.println("5. Enter Pairing Mode");
  Serial.println("6. Clear Paired Device");
  Serial.println("7. View Paired Device Info");
  Serial.println("0. Back to Main Menu");
  Serial.print("Enter choice: ");
}

void printCurrentSettings() {
  Serial.println("\n==== Current Settings ====");
  Serial.println("--- Motor Settings ---");
  Serial.println("Max Fwd Left Power: " + String(config.maxFwdLeftMotorPower));
  Serial.println("Max Fwd Right Power: " + String(config.maxFwdRightMotorPower));
  Serial.println("Max Bwd Left Power: " + String(config.maxBwdLeftMotorPower));
  Serial.println("Max Bwd Right Power: " + String(config.maxBwdRightMotorPower));
  Serial.println("Reverse Left Drive: " + String(config.revLeftDrive ? "Yes" : "No"));
  Serial.println("Reverse Right Drive: " + String(config.revRightDrive ? "Yes" : "No"));
  Serial.println("Swap Motors: " + String(config.swapMotors ? "Yes" : "No"));
  Serial.println("Motor Power Increment: " + String(config.motorPowerIncrement));
  Serial.println("Motor Loop Delay: " + String(config.motorLoopDelayMs) + " ms");
  Serial.println("Motor Increment Time: " + String(config.timeBtwnMotorIncrementMs) + " ms");
  
  Serial.println("--- Foot Pedal Settings ---");
  Serial.println("Foot Pedal Pin: " + String(config.footPedalPin));
  Serial.println("Foot Pedal Active High: " + String(config.footPedalActiveHigh ? "Yes" : "No"));
  Serial.println("Direction Toggle Pin: " + String(config.directionTogglePin));
  Serial.println("Direction Toggle Active High: " + String(config.directionToggleActiveHigh ? "Yes" : "No"));
  Serial.println("Pedal Pressed Power: " + String(config.pedalPressedPower));
  Serial.println("Pedal Debounce: " + String(config.pedalDebounceMs) + " ms");
  Serial.println("Toggle Debounce: " + String(config.toggleDebounceMs) + " ms");
  Serial.println("Foot Pedal Loop Delay: " + String(config.footPedalLoopDelayMs) + " ms");
  
  Serial.println("--- Bluetooth Settings ---");
  Serial.println("Bluetooth Enabled: " + String(config.bluetoothEnabled ? "Yes" : "No"));
  Serial.println("Stop Command Enabled: " + String(config.bluetoothAllowStopCommand ? "Yes" : "No"));
  Serial.println("Remote Override Enabled: " + String(config.bluetoothOverrideLocal ? "Yes" : "No"));
  Serial.println("Bluetooth Timeout: " + String(config.bluetoothTimeoutMs) + " ms");
  Serial.println("Paired Device Name: " + String(strlen(config.pairedDeviceName) > 0 ? config.pairedDeviceName : "None"));
  Serial.println("Paired Device Address: " + String(strlen(config.pairedDeviceAddress) > 0 ? config.pairedDeviceAddress : "None"));
}

int readIntegerInput(int minVal, int maxVal) {
  while (!Serial.available()) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  String input = Serial.readStringUntil('\n');
  input.trim();
  int value = input.toInt();
  
  if (value == 0 && input != "0") {
    Serial.println("Invalid input! Using previous value.");
    return minVal; // Return minimum as fallback
  }
  
  if (value < minVal || value > maxVal) {
    Serial.println("Value out of range (" + String(minVal) + "-" + String(maxVal) + ")! Using constrained value.");
    value = constrain(value, minVal, maxVal);
  }
  
  return value;
}

bool readBoolInput() {
  while (!Serial.available()) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  String input = Serial.readStringUntil('\n');
  input.trim();
  input.toLowerCase();
  
  if (input == "y" || input == "yes" || input == "1" || input == "true") {
    return true;
  } else if (input == "n" || input == "no" || input == "0" || input == "false") {
    return false;
  } else {
    Serial.println("Invalid input! Enter y/n, yes/no, 1/0, or true/false. Using 'No' as default.");
    return false;
  }
}

void handleMotorSettings() {
  int choice;
  do {
    printMotorMenu();
    choice = readIntegerInput(0, 10);
    Serial.println(choice);
    
    switch (choice) {
      case 1:
        Serial.println("Current: " + String(config.maxFwdLeftMotorPower));
        Serial.print("Enter new Max Forward Left Motor Power (1-400): ");
        config.maxFwdLeftMotorPower = readIntegerInput(1, 400);
        Serial.println("Set to: " + String(config.maxFwdLeftMotorPower));
        break;
      case 2:
        Serial.println("Current: " + String(config.maxFwdRightMotorPower));
        Serial.print("Enter new Max Forward Right Motor Power (1-400): ");
        config.maxFwdRightMotorPower = readIntegerInput(1, 400);
        Serial.println("Set to: " + String(config.maxFwdRightMotorPower));
        break;
      case 3:
        Serial.println("Current: " + String(config.maxBwdLeftMotorPower));
        Serial.print("Enter new Max Backward Left Motor Power (-400 to -1): ");
        config.maxBwdLeftMotorPower = readIntegerInput(-400, -1);
        Serial.println("Set to: " + String(config.maxBwdLeftMotorPower));
        break;
      case 4:
        Serial.println("Current: " + String(config.maxBwdRightMotorPower));
        Serial.print("Enter new Max Backward Right Motor Power (-400 to -1): ");
        config.maxBwdRightMotorPower = readIntegerInput(-400, -1);
        Serial.println("Set to: " + String(config.maxBwdRightMotorPower));
        break;
      case 5:
        Serial.println("Current: " + String(config.revLeftDrive ? "Yes" : "No"));
        Serial.print("Reverse Left Drive (y/n): ");
        config.revLeftDrive = readBoolInput();
        Serial.println("Set to: " + String(config.revLeftDrive ? "Yes" : "No"));
        break;
      case 6:
        Serial.println("Current: " + String(config.revRightDrive ? "Yes" : "No"));
        Serial.print("Reverse Right Drive (y/n): ");
        config.revRightDrive = readBoolInput();
        Serial.println("Set to: " + String(config.revRightDrive ? "Yes" : "No"));
        break;
      case 7:
        Serial.println("Current: " + String(config.swapMotors ? "Yes" : "No"));
        Serial.print("Swap Motors (y/n): ");
        config.swapMotors = readBoolInput();
        Serial.println("Set to: " + String(config.swapMotors ? "Yes" : "No"));
        break;
      case 8:
        Serial.println("Current: " + String(config.motorPowerIncrement));
        Serial.print("Enter new Motor Power Increment (1-50): ");
        config.motorPowerIncrement = readIntegerInput(1, 50);
        Serial.println("Set to: " + String(config.motorPowerIncrement));
        break;
      case 9:
        Serial.println("Current: " + String(config.motorLoopDelayMs));
        Serial.print("Enter new Motor Loop Delay in ms (1-100): ");
        config.motorLoopDelayMs = readIntegerInput(1, 100);
        Serial.println("Set to: " + String(config.motorLoopDelayMs));
        break;
      case 10:
        Serial.println("Current: " + String(config.timeBtwnMotorIncrementMs));
        Serial.print("Enter new Time Between Motor Increments in ms (10-1000): ");
        config.timeBtwnMotorIncrementMs = readIntegerInput(10, 1000);
        Serial.println("Set to: " + String(config.timeBtwnMotorIncrementMs));
        break;
    }
  } while (choice != 0);
}

void handleFootPedalSettings() {
  int choice;
  do {
    printFootPedalMenu();
    choice = readIntegerInput(0, 8);
    Serial.println(choice);

    switch (choice) {
      case 1:
        Serial.println("Current: " + String(config.footPedalPin));
        Serial.print("Enter new Foot Pedal Pin (0-13): ");
        config.footPedalPin = readIntegerInput(0, 13);
        Serial.println("Set to: " + String(config.footPedalPin));
        break;
      case 2:
        Serial.println("Current: " + String(config.footPedalActiveHigh ? "Active High" : "Active Low"));
        Serial.print("Foot Pedal Active High (y/n): ");
        config.footPedalActiveHigh = readBoolInput();
        Serial.println("Set to: " + String(config.footPedalActiveHigh ? "Active High" : "Active Low"));
        break;
      case 3:
        Serial.println("Current: " + String(config.directionTogglePin));
        Serial.print("Enter new Direction Toggle Pin (0-13): ");
        config.directionTogglePin = readIntegerInput(0, 13);
        Serial.println("Set to: " + String(config.directionTogglePin));
        break;
      case 4:
        Serial.println("Current: " + String(config.directionToggleActiveHigh ? "Active High" : "Active Low"));
        Serial.print("Direction Toggle Active High (y/n): ");
        config.directionToggleActiveHigh = readBoolInput();
        Serial.println("Set to: " + String(config.directionToggleActiveHigh ? "Active High" : "Active Low"));
        break;
      case 5:
        Serial.println("Current: " + String(config.pedalPressedPower));
        Serial.print("Enter new Pedal Pressed Power (0-100): ");
        config.pedalPressedPower = readIntegerInput(0, 100);
        Serial.println("Set to: " + String(config.pedalPressedPower));
        break;
      case 6:
        Serial.println("Current: " + String(config.pedalDebounceMs));
        Serial.print("Enter new Pedal Debounce in ms (0-500): ");
        config.pedalDebounceMs = readIntegerInput(0, 500);
        Serial.println("Set to: " + String(config.pedalDebounceMs));
        break;
      case 7:
        Serial.println("Current: " + String(config.toggleDebounceMs));
        Serial.print("Enter new Toggle Debounce in ms (0-500): ");
        config.toggleDebounceMs = readIntegerInput(0, 500);
        Serial.println("Set to: " + String(config.toggleDebounceMs));
        break;
      case 8:
        Serial.println("Current: " + String(config.footPedalLoopDelayMs));
        Serial.print("Enter new Foot Pedal Loop Delay in ms (1-100): ");
        config.footPedalLoopDelayMs = readIntegerInput(1, 100);
        Serial.println("Set to: " + String(config.footPedalLoopDelayMs));
        break;
    }
  } while (choice != 0);
}


void handleBluetoothSettings() {
  int choice;
  do {
    printBluetoothMenu();
    choice = readIntegerInput(0, 7);
    Serial.println(choice);
    
    switch (choice) {
      case 1:
        Serial.println("Current: " + String(config.bluetoothEnabled ? "Enabled" : "Disabled"));
        Serial.print("Enable Bluetooth (y/n): ");
        config.bluetoothEnabled = readBoolInput();
        Serial.println("Set to: " + String(config.bluetoothEnabled ? "Enabled" : "Disabled"));
        break;
      case 2:
        Serial.println("Current: " + String(config.bluetoothAllowStopCommand ? "Enabled" : "Disabled"));
        Serial.print("Allow Stop Command (y/n): ");
        config.bluetoothAllowStopCommand = readBoolInput();
        Serial.println("Set to: " + String(config.bluetoothAllowStopCommand ? "Enabled" : "Disabled"));
        break;
      case 3:
        Serial.println("Current: " + String(config.bluetoothOverrideLocal ? "Enabled" : "Disabled"));
        Serial.print("Bluetooth Override Local Input (y/n): ");
        config.bluetoothOverrideLocal = readBoolInput();
        Serial.println("Set to: " + String(config.bluetoothOverrideLocal ? "Enabled" : "Disabled"));
        break;
      case 4:
        Serial.println("Current: " + String(config.bluetoothTimeoutMs));
        Serial.print("Enter new Bluetooth Timeout (1000-30000 ms): ");
        config.bluetoothTimeoutMs = readIntegerInput(1000, 30000);
        Serial.println("Set to: " + String(config.bluetoothTimeoutMs));
        break;
      case 5:
        Serial.println("Entering pairing mode...");
        Serial.println("The robot is now discoverable as 'GBG Robot'");
        Serial.println("Use your Bluetooth device to connect.");
        Serial.println("Device will be automatically paired on first connection.");
        Serial.println("Press any key to exit pairing mode...");
        
        // Clear any existing pairing to allow new device
        strcpy(config.pairedDeviceName, "");
        strcpy(config.pairedDeviceAddress, "");
        saveConfig();
        
        // Wait for user input to exit pairing mode
        while (!Serial.available()) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        while (Serial.available()) {
          Serial.read(); // Clear buffer
        }
        Serial.println("Exited pairing mode.");
        break;
      case 6:
        Serial.println("Current paired device: " + String(strlen(config.pairedDeviceName) > 0 ? config.pairedDeviceName : "None"));
        Serial.print("Clear paired device (y/n): ");
        if (readBoolInput()) {
          strcpy(config.pairedDeviceName, "");
          strcpy(config.pairedDeviceAddress, "");
          Serial.println("Paired device cleared.");
        } else {
          Serial.println("Paired device not cleared.");
        }
        break;
      case 7:
        Serial.println("=== Paired Device Info ===");
        if (strlen(config.pairedDeviceName) > 0) {
          Serial.println("Device Name: " + String(config.pairedDeviceName));
          Serial.println("Device Address: " + String(config.pairedDeviceAddress));
        } else {
          Serial.println("No device paired.");
        }
        break;
      case 0:
        break;
      default:
        Serial.println("Invalid choice!");
        break;
    }
  } while (choice != 0);
}

void menu_task_func(void *pvParams) {
  // Setup
  Serial.println("\n[Menu Task] Serial menu system initialized.");
  Serial.println("Send 'm' to open the configuration menu at any time.");
  
  // Main loop
  for (;;) {
    if (Serial.available()) {
      char input = Serial.read();
      
      if (input == 'm' || input == 'M') {
        // Clear any remaining input
        while (Serial.available()) {
          Serial.read();
        }
        
        int choice;
        do {
          printMainMenu();
          choice = readIntegerInput(0, 6);
          Serial.println(choice);
          
          switch (choice) {
            case 1:
              printCurrentSettings();
              break;
            case 2:
              handleMotorSettings();
              break;
            case 3:
              handleFootPedalSettings();
              break;
            case 4:
              handleBluetoothSettings();
              break;
            case 5:
              saveConfig();
              Serial.println("Settings saved to flash memory!");
              break;
            case 6:
              initDefaultConfig();
              Serial.println("Settings reset to defaults!");
              break;
            case 0:
              Serial.println("Exiting menu...");
              break;
            default:
              Serial.println("Invalid choice!");
              break;
          }
        } while (choice != 0);
        
        Serial.println("\nSend 'm' to open the configuration menu again.");
      }
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

  /**************************************************************************************
  * SETUP/LOOP
  **************************************************************************************/
 
  void setup()
  {
    Serial.begin(115200);
    while (!Serial) { }
    
    // Initialize configuration system
    initDefaultConfig();
    loadConfig();
    Serial.println("Configuration system initialized.");
  
      /* Init a task that calls 'loop'
     * since after the call to
     * 'vTaskStartScheduler' we'll never
     * get out of setup() and therefore
     * would never get to loop(), as we
     * are leaving the default execution
     * flow.
     */
    auto const rc_loop = xTaskCreate
      (
        loop_thread_func,
        static_cast<const char*>("Loop Thread"),
        512 / 4,   /* usStackDepth in words */
        nullptr,   /* pvParameters */
        1,         /* uxPriority */
        &loop_task /* pxCreatedTask */
      );
  
    if (rc_loop != pdPASS) {
      Serial.println("Failed to create 'loop' thread");
      return;
    }
  
    auto const rc_blinky = xTaskCreate
      (
        blinky_thread_func,
        static_cast<const char*>("Blinky Thread"),
        512 / 4,     /* usStackDepth in words */
        nullptr,     /* pvParameters */
        1,           /* uxPriority */
        &blinky_task /* pxCreatedTask */
      );
  
    if (rc_blinky != pdPASS) {
      Serial.println("Failed to create 'loop' thread");
      return;
    }
  
    auto const rc_motor_drive = xTaskCreate
      (
        motor_drive_func,
        static_cast<const char*>("Motor Drive Thread"),
        512 / 4,
        nullptr,
        1,
        &motor_drive_task
      );
    
    if (rc_motor_drive != pdPASS) {
      Serial.println("Failed to create 'motor drive' thread");
      return;
    }
  
    auto const rc_foot_pedal = xTaskCreate
      (
        foot_pedal_func,
        static_cast<const char*>("Foot Pedal Thread"),
        512/4,
        nullptr,
        1,
        &foot_pedal_task
      );

    if (rc_foot_pedal != pdPASS) {
      Serial.println("Failed to create 'foot pedal' thread");
      return;
    }

    // Create Bluetooth task only if enabled
    if (config.bluetoothEnabled) {
      auto const rc_bluetooth = xTaskCreate
        (
          bluetooth_func,
          static_cast<const char*>("Bluetooth Thread"),
          1024/4,
          nullptr,
          1,
          &bluetooth_task
        );
      
      if (rc_bluetooth != pdPASS) {
        Serial.println("Failed to create 'bluetooth' thread");
        return;
      }
    }

    auto const rc_menu = xTaskCreate
      (
        menu_task_func,
        static_cast<const char*>("Menu Task"),
        1024/4,
        nullptr,
        1,
        &menu_task
      );
    
    if (rc_menu != pdPASS) {
      Serial.println("Failed to create 'menu' thread");
      return;
    }
  
  Serial.println("Starting scheduler ...");
  /* Start the scheduler. */
  vTaskStartScheduler();
  /* We'll never get here. */
  for( ;; );
}

#endif  // GBG_VARIANT_FOOT_PEDAL
