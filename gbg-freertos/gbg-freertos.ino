/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_FreeRTOS.h>
#include <LibPrintf.h>
#include <DualVNH5019MotorShield.h>
#include <ArduinoBLE.h>
#include <Preferences.h>
#include <semphr.h>
 
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
  
  // Joystick settings
  int joystickXPin;
  int joystickYPin;
  bool flipXAxis;
  bool flipYAxis;
  bool xyTranspose;
  int maxXAxis;
  int maxYAxis;
  int midXAxis;
  int midYAxis;
  int minXAxis;
  int minYAxis;
  int xAxisDeadzone;
  int yAxisDeadzone;
  bool useSelfCalJoystick;
  int joystickLoopDelayMs;
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
  
  // Joystick settings defaults
  config.joystickXPin = A2;
  config.joystickYPin = A3;
  config.flipXAxis = false;
  config.flipYAxis = false;
  config.xyTranspose = true;
  config.maxXAxis = 1023;
  config.maxYAxis = 1023;
  config.midXAxis = 495;
  config.midYAxis = 495;
  config.minXAxis = 0;
  config.minYAxis = 0;
  config.xAxisDeadzone = 50;
  config.yAxisDeadzone = 50;
  config.useSelfCalJoystick = true;
  config.joystickLoopDelayMs = 20;
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
  
  // Load joystick settings  
  config.flipXAxis = preferences.getBool("flipXAxis", config.flipXAxis);
  config.flipYAxis = preferences.getBool("flipYAxis", config.flipYAxis);
  config.xyTranspose = preferences.getBool("xyTranspose", config.xyTranspose);
  config.maxXAxis = preferences.getInt("maxXAxis", config.maxXAxis);
  config.maxYAxis = preferences.getInt("maxYAxis", config.maxYAxis);
  config.midXAxis = preferences.getInt("midXAxis", config.midXAxis);
  config.midYAxis = preferences.getInt("midYAxis", config.midYAxis);
  config.minXAxis = preferences.getInt("minXAxis", config.minXAxis);
  config.minYAxis = preferences.getInt("minYAxis", config.minYAxis);
  config.xAxisDeadzone = preferences.getInt("xDeadzone", config.xAxisDeadzone);
  config.yAxisDeadzone = preferences.getInt("yDeadzone", config.yAxisDeadzone);
  config.useSelfCalJoystick = preferences.getBool("useSelfCal", config.useSelfCalJoystick);
  config.joystickLoopDelayMs = preferences.getInt("joyLoopDelay", config.joystickLoopDelayMs);
  
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
  
  // Save joystick settings
  preferences.putBool("flipXAxis", config.flipXAxis);
  preferences.putBool("flipYAxis", config.flipYAxis);
  preferences.putBool("xyTranspose", config.xyTranspose);
  preferences.putInt("maxXAxis", config.maxXAxis);
  preferences.putInt("maxYAxis", config.maxYAxis);
  preferences.putInt("midXAxis", config.midXAxis);
  preferences.putInt("midYAxis", config.midYAxis);
  preferences.putInt("minXAxis", config.minXAxis);
  preferences.putInt("minYAxis", config.minYAxis);
  preferences.putInt("xDeadzone", config.xAxisDeadzone);
  preferences.putInt("yDeadzone", config.yAxisDeadzone);
  preferences.putBool("useSelfCal", config.useSelfCalJoystick);
  preferences.putInt("joyLoopDelay", config.joystickLoopDelayMs);
  
  preferences.end();
}

 TaskHandle_t loop_task, blinky_task, motor_drive_task, joystick_task, menu_task;

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
int leftMotorPower = 0;
int rightMotorPower = 0;
SemaphoreHandle_t motorPowerMutex = nullptr;
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
  if (xSemaphoreTake(motorPowerMutex, portMAX_DELAY) == pdTRUE)
  {
    leftMotorPower = 0;
    rightMotorPower = 0;
    xSemaphoreGive(motorPowerMutex);
  }
 
   md.init();
 
   int leftMotorRequestedPower = 0;
   int rightMotorRequestedPower = 0;
 
   int leftMotorAppliedPower = 0;
   int rightMotorAppliedPower = 0;
   int last_motor_increment_time = 0;
 
   // loop()
   for (;;)
   {
    int leftPowerSnapshot = 0;
    int rightPowerSnapshot = 0;

    if (xSemaphoreTake(motorPowerMutex, portMAX_DELAY) == pdTRUE)
    {
      leftMotorPower = constrain(leftMotorPower, -100, 100);
      rightMotorPower = constrain(rightMotorPower, -100, 100);

      leftPowerSnapshot = leftMotorPower;
      rightPowerSnapshot = rightMotorPower;

      xSemaphoreGive(motorPowerMutex);
    }

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
       printf("[Motor Dr Thread] LRP=%5d, RRP=%5d, | LMAP=%5d, RMAP=%5d | LM_CUR=%5d mA, RM_CUR=%5d mA \n", 
               leftMotorRequestedPower, rightMotorRequestedPower, leftMotorAppliedPower, rightMotorAppliedPower, left_motor_current, right_motor_current);
       motorPrintCounter = 0;
     }
     motorPrintCounter++;
 
     vTaskDelay(config.motorLoopDelayMs / portTICK_PERIOD_MS);
   }
 }
  
 // ========================================================
 //                Joystick Thread
 // ========================================================
 
  void joystick_func(void *pvParams)
 {
   // setup()
   int xValue = 0;
   int yValue = 0;
   int correctedXValue = 0;
   int correctedYValue= 0;
   int joystickPrintCount = 0;
   int X_MID = config.midXAxis;
   int Y_MID = config.midYAxis;
 
   if (config.useSelfCalJoystick) {
     // We are going to calibrate the joystick, so lets block.
     Serial.println("[Joystick Thread] Calibrating Joystick...");
     for (int i = 0; i < 256; i++)
     {
       xValue = analogRead(config.joystickXPin);
       yValue = analogRead(config.joystickYPin);
 
       X_MID += xValue;
       Y_MID += yValue;
     }
 
     // Quick and easy division.
     X_MID = X_MID >> 8;
     Y_MID = Y_MID >> 8;
 
 
    int leftMotorSnapshot = 0;
    int rightMotorSnapshot = 0;
    if (xSemaphoreTake(motorPowerMutex, portMAX_DELAY) == pdTRUE)
    {
      leftMotorSnapshot = leftMotorPower;
      rightMotorSnapshot = rightMotorPower;
      xSemaphoreGive(motorPowerMutex);
    }

    printf("[Joystick Thread] X_MID = %4d, Y_MID = %4d", X_MID, Y_MID);
    printf("[Joystick Thread] LeftMotor=%d, RightMotor=%d\n", leftMotorSnapshot, rightMotorSnapshot);
 
   }

   const int xFlip = config.flipXAxis ? -1 : 1;
   const int yFlip = config.flipYAxis ? -1 : 1;
 
   int scaledX = 0;
   int scaledY = 0;
 
   int maximum = 0;
   int total = 0;
   int difference = 0;
 
   for(;;)
   {
     // read analog X and Y analog values
     xValue = analogRead(config.joystickXPin);
     yValue = analogRead(config.joystickYPin);
 
     correctedXValue = xValue;
     correctedYValue = yValue;
 
     // If the joystick values are close to the deadzones, let them be in the dead zone. 
     // Allows for some buffer.
     if (xValue - X_MID < config.xAxisDeadzone && xValue - X_MID > -config.xAxisDeadzone )
     {
         correctedXValue = X_MID;
     }
     if (yValue - Y_MID < config.yAxisDeadzone && yValue - Y_MID > -config.yAxisDeadzone )
     {
         correctedYValue = Y_MID;
     }
 
     // Now we need to scale the captured values, to values between -100 and 100.
     scaledX = map(correctedXValue - X_MID, config.minXAxis - X_MID, config.maxXAxis - X_MID, -100, 100);
     scaledY = map(correctedYValue - Y_MID, config.minYAxis - Y_MID, config.maxYAxis - Y_MID, -100, 100);
     
 
     scaledX = scaledX * xFlip;
     scaledY = scaledY * yFlip;
 
     if (config.xyTranspose) {
       // Transpose the two axis, don't allocate additional memory.
       scaledX = scaledX ^ scaledY;
       scaledY = scaledX ^ scaledY;
       scaledX = scaledX ^ scaledY;
     }
 
    if (joystickPrintCount >= 10)
    {
      //printf("[Joystick Thread] X=%4d, Y=%4d, Xc=%4d, Yc=%4d, Xs=%4d, Ys=%4d \n", xValue, yValue, correctedXValue, correctedYValue, scaledX, scaledY);
      joystickPrintCount = 0;
    }
    joystickPrintCount++;

    // Now we have the motor data. We can calculate the arcade drive values.
    maximum = max(abs(scaledY), abs(scaledX));
    total =  scaledY + scaledX;
    difference = scaledY - scaledX;

    int newLeftMotorPower = 0;
    int newRightMotorPower = 0;

    if (scaledY >= 0)
    {
      if (scaledX >= 0)
      {
        // I quadrant
        newLeftMotorPower = maximum;
        newRightMotorPower = difference;
      }
      else
      {
        // II quadrant
        newLeftMotorPower = total;
        newRightMotorPower = maximum;
      }
    }
    else
    {
        if (scaledX >= 0)
        {
          // IV quadrant
          newLeftMotorPower = total;
          newRightMotorPower = -maximum;
        }
        else
        {
          // III quadrant
          newLeftMotorPower = -maximum;
          newRightMotorPower = difference;
        }
    }

    if (xSemaphoreTake(motorPowerMutex, portMAX_DELAY) == pdTRUE)
    {
      leftMotorPower = newLeftMotorPower;
      rightMotorPower = newRightMotorPower;
      xSemaphoreGive(motorPowerMutex);
    }
 
     vTaskDelay(config.joystickLoopDelayMs / portTICK_PERIOD_MS);
   }
 }
 
 // ========================================================
 //                Serial Menu Task
 // ========================================================
 
void printMainMenu() {
  Serial.println("\n==== Robot Configuration Menu ====");
  Serial.println("1. View Current Settings");
  Serial.println("2. Motor Settings");
  Serial.println("3. Joystick Settings");
  Serial.println("4. Save Settings to Flash");
  Serial.println("5. Reset to Defaults");
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

void printJoystickMenu() {
  Serial.println("\n---- Joystick Settings ----");
  Serial.println("1. Flip X Axis");
  Serial.println("2. Flip Y Axis");
  Serial.println("3. X-Y Transpose");
  Serial.println("4. Max X Axis");
  Serial.println("5. Max Y Axis");
  Serial.println("6. Mid X Axis");
  Serial.println("7. Mid Y Axis");
  Serial.println("8. Min X Axis");
  Serial.println("9. Min Y Axis");
  Serial.println("10. X Axis Deadzone");
  Serial.println("11. Y Axis Deadzone");
  Serial.println("12. Use Self Calibration");
  Serial.println("13. Joystick Loop Delay (ms)");
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
  
  Serial.println("--- Joystick Settings ---");
  Serial.println("Flip X Axis: " + String(config.flipXAxis ? "Yes" : "No"));
  Serial.println("Flip Y Axis: " + String(config.flipYAxis ? "Yes" : "No"));
  Serial.println("X-Y Transpose: " + String(config.xyTranspose ? "Yes" : "No"));
  Serial.println("Max X Axis: " + String(config.maxXAxis));
  Serial.println("Max Y Axis: " + String(config.maxYAxis));
  Serial.println("Mid X Axis: " + String(config.midXAxis));
  Serial.println("Mid Y Axis: " + String(config.midYAxis));
  Serial.println("Min X Axis: " + String(config.minXAxis));
  Serial.println("Min Y Axis: " + String(config.minYAxis));
  Serial.println("X Deadzone: " + String(config.xAxisDeadzone));
  Serial.println("Y Deadzone: " + String(config.yAxisDeadzone));
  Serial.println("Use Self Calibration: " + String(config.useSelfCalJoystick ? "Yes" : "No"));
  Serial.println("Joystick Loop Delay: " + String(config.joystickLoopDelayMs) + " ms");
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

void handleJoystickSettings() {
  int choice;
  do {
    printJoystickMenu();
    choice = readIntegerInput(0, 13);
    Serial.println(choice);
    
    switch (choice) {
      case 1:
        Serial.println("Current: " + String(config.flipXAxis ? "Yes" : "No"));
        Serial.print("Flip X Axis (y/n): ");
        config.flipXAxis = readBoolInput();
        Serial.println("Set to: " + String(config.flipXAxis ? "Yes" : "No"));
        break;
      case 2:
        Serial.println("Current: " + String(config.flipYAxis ? "Yes" : "No"));
        Serial.print("Flip Y Axis (y/n): ");
        config.flipYAxis = readBoolInput();
        Serial.println("Set to: " + String(config.flipYAxis ? "Yes" : "No"));
        break;
      case 3:
        Serial.println("Current: " + String(config.xyTranspose ? "Yes" : "No"));
        Serial.print("X-Y Transpose (y/n): ");
        config.xyTranspose = readBoolInput();
        Serial.println("Set to: " + String(config.xyTranspose ? "Yes" : "No"));
        break;
      case 4:
        Serial.println("Current: " + String(config.maxXAxis));
        Serial.print("Enter new Max X Axis (0-1023): ");
        config.maxXAxis = readIntegerInput(0, 1023);
        Serial.println("Set to: " + String(config.maxXAxis));
        break;
      case 5:
        Serial.println("Current: " + String(config.maxYAxis));
        Serial.print("Enter new Max Y Axis (0-1023): ");
        config.maxYAxis = readIntegerInput(0, 1023);
        Serial.println("Set to: " + String(config.maxYAxis));
        break;
      case 6:
        Serial.println("Current: " + String(config.midXAxis));
        Serial.print("Enter new Mid X Axis (0-1023): ");
        config.midXAxis = readIntegerInput(0, 1023);
        Serial.println("Set to: " + String(config.midXAxis));
        break;
      case 7:
        Serial.println("Current: " + String(config.midYAxis));
        Serial.print("Enter new Mid Y Axis (0-1023): ");
        config.midYAxis = readIntegerInput(0, 1023);
        Serial.println("Set to: " + String(config.midYAxis));
        break;
      case 8:
        Serial.println("Current: " + String(config.minXAxis));
        Serial.print("Enter new Min X Axis (0-1023): ");
        config.minXAxis = readIntegerInput(0, 1023);
        Serial.println("Set to: " + String(config.minXAxis));
        break;
      case 9:
        Serial.println("Current: " + String(config.minYAxis));
        Serial.print("Enter new Min Y Axis (0-1023): ");
        config.minYAxis = readIntegerInput(0, 1023);
        Serial.println("Set to: " + String(config.minYAxis));
        break;
      case 10:
        Serial.println("Current: " + String(config.xAxisDeadzone));
        Serial.print("Enter new X Axis Deadzone (0-200): ");
        config.xAxisDeadzone = readIntegerInput(0, 200);
        Serial.println("Set to: " + String(config.xAxisDeadzone));
        break;
      case 11:
        Serial.println("Current: " + String(config.yAxisDeadzone));
        Serial.print("Enter new Y Axis Deadzone (0-200): ");
        config.yAxisDeadzone = readIntegerInput(0, 200);
        Serial.println("Set to: " + String(config.yAxisDeadzone));
        break;
      case 12:
        Serial.println("Current: " + String(config.useSelfCalJoystick ? "Yes" : "No"));
        Serial.print("Use Self Calibration (y/n): ");
        config.useSelfCalJoystick = readBoolInput();
        Serial.println("Set to: " + String(config.useSelfCalJoystick ? "Yes" : "No"));
        break;
      case 13:
        Serial.println("Current: " + String(config.joystickLoopDelayMs));
        Serial.print("Enter new Joystick Loop Delay in ms (1-100): ");
        config.joystickLoopDelayMs = readIntegerInput(1, 100);
        Serial.println("Set to: " + String(config.joystickLoopDelayMs));
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
          choice = readIntegerInput(0, 5);
          Serial.println(choice);
          
          switch (choice) {
            case 1:
              printCurrentSettings();
              break;
            case 2:
              handleMotorSettings();
              break;
            case 3:
              handleJoystickSettings();
              break;
            case 4:
              saveConfig();
              Serial.println("Settings saved to flash memory!");
              break;
            case 5:
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
  
    motorPowerMutex = xSemaphoreCreateMutex();
    if (motorPowerMutex == nullptr)
    {
      Serial.println("Failed to create motor power mutex");
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
  
    auto const rc_joystick = xTaskCreate
      (
        joystick_func,
        static_cast<const char*>("Joystick Thread"),
        512/4,
        nullptr,
        1,
        &joystick_task
      );
    
    if (rc_joystick != pdPASS) {
      Serial.println("Failed to create 'joystick' thread");
      return;
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