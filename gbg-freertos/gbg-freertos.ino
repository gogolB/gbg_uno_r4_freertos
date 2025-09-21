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
 
 TaskHandle_t loop_task, blinky_task, motor_drive_task, joystick_task;

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
 #define REV_LEFT_DRIVE false       // Change these if the motors are wired backwards.
 #define REV_RIGHT_DRIVE false      // Change these if the motors are wired backwards.
 #define SWAP_MOTORS false          // Change if the left motor and the right motor are swapped.
 
 // This is the max constrained limit for the device. This goes up to 400.
 const int MAX_FWD_LEFT_MOTOR_POWER = 100;
 const int MAX_FWD_RIGHT_MOTOR_POWER = 100;
 const int MAX_BWD_LEFT_MOTOR_POWER = -100;
 const int MAX_BWD_RIGHT_MOTOR_POWER = -100;
 
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
 
 #define MOTOR_POWER_INCREMENT 5   // Increment power by 5 at a time
 #define MOTOR_LOOP_DELAY_MS 10   // Delay between increments (adjust as needed)
 #define TIME_BTWN_MOTOR_INCREMENT_MS 100 // Time between motor increments
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

    #if REV_LEFT_DRIVE
      leftMotorPowerLocal = leftMotorPowerLocal * -1;
    #endif

    #if REV_RIGHT_DRIVE
      rightMotorPowerLocal = rightMotorPowerLocal * -1;
    #endif

    #if SWAP_MOTORS
      leftMotorPowerLocal = leftMotorPowerLocal ^ rightMotorPowerLocal;
      rightMotorPowerLocal = leftMotorPowerLocal ^ rightMotorPowerLocal;
      leftMotorPowerLocal = leftMotorPowerLocal ^ rightMotorPowerLocal;
    #endif

    // Smooth startup ramping
    if (abs(leftMotorPowerLocal) > 10) {
      leftMotorRequestedPower = map(leftMotorPowerLocal, -100, 100, MAX_BWD_LEFT_MOTOR_POWER, MAX_FWD_LEFT_MOTOR_POWER);
      leftMotorRequestedPower = constrain(leftMotorRequestedPower, MAX_BWD_LEFT_MOTOR_POWER, MAX_FWD_LEFT_MOTOR_POWER);
    } else {
      leftMotorRequestedPower = 0;
    }

    if (abs(rightMotorPowerLocal) > 10) {
      rightMotorRequestedPower = map(rightMotorPowerLocal, -100, 100, MAX_BWD_LEFT_MOTOR_POWER, MAX_FWD_LEFT_MOTOR_POWER);
      rightMotorRequestedPower = constrain(rightMotorRequestedPower, MAX_BWD_LEFT_MOTOR_POWER, MAX_FWD_RIGHT_MOTOR_POWER);
    } else {
      rightMotorRequestedPower = 0;
     }


     // Handle the acceleration and deceleration of the motors.
     if (millis() - last_motor_increment_time > TIME_BTWN_MOTOR_INCREMENT_MS) {
        last_motor_increment_time = millis();

        if (leftMotorAppliedPower < leftMotorRequestedPower) {
          leftMotorAppliedPower += min(MOTOR_POWER_INCREMENT, leftMotorRequestedPower - leftMotorAppliedPower);
        } else if (leftMotorAppliedPower > leftMotorRequestedPower) {
          leftMotorAppliedPower -= min(MOTOR_POWER_INCREMENT, leftMotorAppliedPower - leftMotorRequestedPower);
        } else {
          leftMotorAppliedPower = leftMotorRequestedPower;
        }

        if (rightMotorAppliedPower < rightMotorRequestedPower) {
          rightMotorAppliedPower += min(MOTOR_POWER_INCREMENT, rightMotorRequestedPower - rightMotorAppliedPower); 
        } else if (rightMotorAppliedPower > rightMotorRequestedPower) {
          rightMotorAppliedPower -= min(MOTOR_POWER_INCREMENT, rightMotorAppliedPower - rightMotorRequestedPower);
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
 
     vTaskDelay(MOTOR_LOOP_DELAY_MS / portTICK_PERIOD_MS);
   }
 }
  
 // ========================================================
 //                Joystick Thread
 // ========================================================
 
 #define JOYSTICK_X_AXIS_PIN A2
 #define JOYSTICK_Y_AXIS_PIN A3
 
 #define FLIP_X_AXIS false
 #define FLIP_Y_AXIS false
 #define X_Y_TRANSPOSE true
 
 #define MAX_X_AXIS 1023
 #define MAX_Y_AXIS 1023
 
 #define MID_X_AXIS 495
 #define MID_Y_AXIS 495
 
 #define MIN_X_AXIS 0
 #define MIN_Y_AXIS 0
 
 #define X_AXIS_DEADZONE 50
 #define Y_AXIS_DEADZONE 50
 
 #define USE_SELF_CAL_JOYSTICK true

 #define JOYSTICK_LOOP_DELAY_MS 20
 
 void joystick_func(void *pvParams)
 {
   // setup()
   int xValue = 0;
   int yValue = 0;
   int correctedXValue = 0;
   int correctedYValue= 0;
   int joystickPrintCount = 0;
   int X_MID = MID_X_AXIS;
   int Y_MID = MID_Y_AXIS;
 
   #if USE_SELF_CAL_JOYSTICK
     // We are going to calibrate the joystick, so lets block.
     Serial.println("[Joystick Thread] Calibrating Joystick...");
     for (int i = 0; i < 256; i++)
     {
       xValue = analogRead(JOYSTICK_X_AXIS_PIN);
       yValue = analogRead(JOYSTICK_Y_AXIS_PIN);
 
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
 
   #endif
 
   #if FLIP_X_AXIS
     const int xFlip = -1;
   #else
     const int xFlip = 1;
   #endif
 
   #if FLIP_Y_AXIS
     const int yFlip = -1;
   #else
     const int yFlip = 1;
   #endif
 
   int scaledX = 0;
   int scaledY = 0;
 
   int maximum = 0;
   int total = 0;
   int difference = 0;
 
   for(;;)
   {
     // read analog X and Y analog values
     xValue = analogRead(JOYSTICK_X_AXIS_PIN);
     yValue = analogRead(JOYSTICK_Y_AXIS_PIN);
 
     correctedXValue = xValue;
     correctedYValue = yValue;
 
     // If the joystick values are close to the deadzones, let them be in the dead zone. 
     // Allows for some buffer.
     if (xValue - X_MID < X_AXIS_DEADZONE && xValue - X_MID > -X_AXIS_DEADZONE )
     {
         correctedXValue = X_MID;
     }
     if (yValue - Y_MID < Y_AXIS_DEADZONE && yValue - Y_MID > -Y_AXIS_DEADZONE )
     {
         correctedYValue = Y_MID;
     }
 
     // Now we need to scale the captured values, to values between -100 and 100.
     scaledX = map(correctedXValue - X_MID, MIN_X_AXIS - X_MID, MAX_X_AXIS - X_MID, -100, 100);
     scaledY = map(correctedYValue - Y_MID, MIN_Y_AXIS - Y_MID, MAX_Y_AXIS - Y_MID, -100, 100);
     
 
     scaledX = scaledX * xFlip;
     scaledY = scaledY * yFlip;
 
     #if X_Y_TRANSPOSE
       // Transpose the two axis, don't allocate additional memory.
       scaledX = scaledX ^ scaledY;
       scaledY = scaledX ^ scaledY;
       scaledX = scaledX ^ scaledY;
     #endif
 
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
 
     vTaskDelay(JOYSTICK_LOOP_DELAY_MS / portTICK_PERIOD_MS);
   }
 }
 
  /**************************************************************************************
  * SETUP/LOOP
  **************************************************************************************/
 
  void setup()
  {
    Serial.begin(115200);
    while (!Serial) { }
  
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
  
    Serial.println("Starting scheduler ...");
    /* Start the scheduler. */
    vTaskStartScheduler();
    /* We'll never get here. */
    for( ;; );
  }