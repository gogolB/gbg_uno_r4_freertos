/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_FreeRTOS.h>
#include <LibPrintf.h>
#include <DualVNH5019MotorShield.h>
#include <ArduinoBLE.h>
#include <Preferences.h>

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

TaskHandle_t loop_task, blinky_task, motor_drive_task, joystick_task, ble_task;

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

  auto const ble_server = xTaskCreate
    (
      ble_func,
      static_cast<const char*>("BLE Server Thread"),
      512/4,
      nullptr,
      1,
      &ble_task
    );

  if (ble_server != pdPASS)
  {
    Serial.println("Failed to create 'ble server' thread");
    return;
  }

  Serial.println("Starting scheduler ...");
  /* Start the scheduler. */
  vTaskStartScheduler();
  /* We'll never get here. */
  for( ;; );
}

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

  // ==================================================
  //                    BLE Server
  // ==================================================


#define FORCE_BLE_RESET false

  BLEService remoteControl("06f3d410-1f9d-4020-b6c0-ad20d295d869");
  BLEBoolCharacteristic remoteControlEnabled("cc33b6ac-e38e-45ab-a5f4-d39b77928ec9", BLERead | BLEWrite);
  BLEBoolCharacteristic remoteControlStop("2f81b369-7d9f-4d5e-92fa-1c00c6651b5d", BLERead | BLEWrite);
  BLEShortCharacteristic remoteControlJoystickX("a204e8f5-cea7-47e8-89eb-59bcc2ba28d1", BLERead | BLEWrite);
  BLEShortCharacteristic remoteControlJoystickY("2e55d9fb-094e-4a92-a1ca-fe5055d0c54e", BLERead | BLEWrite);

  bool isConnected = false;

  Preferences prefs;

  int remoteX;
  int remoteY;
  bool remoteEnabled;
  bool remoteStop;

  void ble_func(void *pvParams)
  {
    // Setup()


    remoteX = 0;
    remoteY = 0;
    remoteEnabled = false;
    remoteStop = false;

    if (!BLE.begin()) 
    {
      Serial.println("Starting BLE failed!");
      while (1);
    }

    BLE.setLocalName("EnMed-GBG-Car-V1.0");
    BLE.setAdvertisedService(remoteControl);
    remoteControl.addCharacteristic(remoteControlEnabled);
    remoteControl.addCharacteristic(remoteControlStop);
    remoteControl.addCharacteristic(remoteControlJoystickX);
    remoteControl.addCharacteristic(remoteControlJoystickY);
    BLE.addService(remoteControl);
    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");
    
    prefs.begin("enmed-gbg");
    
    // First ever init, we don't have a remote control address.
    if (!prefs.isKey("remote-control") || FORCE_BLE_RESET)
    {
      prefs.putString("remote-control", "n/a");
    }

    isConnected = false;

    BLEDevice central;
    central = BLE.central();

    // loop()
    for(;;)
    {

      if (central && !isConnected)
      {
        String connected_mac = central.address();
        String stored_mac = prefs.getString("remote-control");
        if (stored_mac == "n/a")
        {
          prefs.putString("remote-control", connected_mac);
        }
        else if (connected_mac == stored_mac)
        {
          BLE.stopAdvertise();
          isConnected = true;
        }
        else
        {
          BLE.disconnect();
          central = BLE.central();
          isConnected = false;
        }
      } 
      else if (central && isConnected && BLE.connected())
      {
        printf("[BLE Thread] Is connected to %s\n", central.address());

        remoteX = remoteControlJoystickX.value();
        remoteY = remoteControlJoystickY.value();
        remoteStop = remoteControlStop.value();
        remoteEnabled = remoteControlEnabled.value();
        isConnected = true;
      }
      else
      {
        isConnected = false;
      }
      
      const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
      vTaskDelay(xDelay);
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
const int MAX_FWD_LEFT_MOTOR_POWER = 150;
const int MAX_FWD_RIGHT_MOTOR_POWER = 150;
const int MAX_BWD_LEFT_MOTOR_POWER = -150;
const int MAX_BWD_RIGHT_MOTOR_POWER = -150;

// This is the motor power. A power of greater than 0 is forward.
// A power of less than 0 is backward.
int leftMotorPower = 0;
int rightMotorPower = 0;

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
  leftMotorPower = 0;
  rightMotorPower = 0;

  md.init();

  int leftMotorAppliedPower = 0;
  int rightMotorAppliedPower = 0;
  // loop()
  for(;;)
  {

    // Motor power is constrained to between -100% to 100%
    // This is then mapped to the PWM range necessary for use on the motor driver.
    leftMotorPower = constrain(leftMotorPower, -50, 100);
    rightMotorPower = constrain(rightMotorPower, -50, 100);

    #if REV_LEFT_DRIVE
      leftMotorPower = leftMotorPower * -1;
    #endif
    #if REV_RIGHT_DRIVE
      rightMotorPower = rightMotorPower * -1;
    #endif

    #if SWAP_MOTORS
      leftMotorPower = leftMotorPower ^ rightMotorPower;
      rightMotorPower = leftMotorPower ^ rightMotorPower;
      leftMotorPower = leftMotorPower ^ rightMotorPower;
    #endif

    
    if (leftMotorPower > 10 || leftMotorPower < 10) {
      leftMotorAppliedPower = map(leftMotorPower, -100, 100, MAX_BWD_LEFT_MOTOR_POWER, MAX_FWD_LEFT_MOTOR_POWER);
      leftMotorAppliedPower = constrain(leftMotorAppliedPower, MAX_BWD_LEFT_MOTOR_POWER, MAX_FWD_LEFT_MOTOR_POWER);
      md.setM1Speed(leftMotorAppliedPower);
      stopIfFault();
    } else {
      md.setM1Speed(0);
    }

    if (rightMotorPower > 10 || rightMotorPower < 10) {
      rightMotorAppliedPower = map(rightMotorPower, -100, 100, MAX_BWD_LEFT_MOTOR_POWER, MAX_FWD_LEFT_MOTOR_POWER);
      rightMotorAppliedPower = constrain(rightMotorAppliedPower, MAX_BWD_LEFT_MOTOR_POWER, MAX_FWD_RIGHT_MOTOR_POWER);
      md.setM2Speed(rightMotorAppliedPower);
      stopIfFault();
    } else {
      md.setM2Speed(0);
    }

    int left_motor_current = md.getM1CurrentMilliamps();
    int right_motor_current = md.getM2CurrentMilliamps();

    if (motorPrintCounter >=10)
    {
      printf("[Motor Dr Thread] LMP=%4d, RMP=%4d, | LMAP=%5d, RMAP=%5d LM_CUR=%4d mA, RM_CUR=%4d mA \n", leftMotorPower, rightMotorPower, leftMotorAppliedPower, rightMotorAppliedPower, left_motor_current, right_motor_current);
      motorPrintCounter = 0;
    }
    motorPrintCounter++;
    const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);
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


    printf("[Joystick Thread] X_MID = %4d, Y_MID = %4d", X_MID, Y_MID);
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
      printf("[Joystick Thread] X=%4d, Y=%4d, Xc=%4d, Yc=%4d, Xs=%4d, Ys=%4d \n", xValue, yValue, correctedXValue, correctedYValue, scaledX, scaledY);
      joystickPrintCount = 0;
    }
    joystickPrintCount++;
    
    // Now we have the motor data. We can calculate the arcade drive values.
    maximum = max(abs(scaledY), abs(scaledX));
    total =  scaledY + scaledX;
    difference = scaledY - scaledX;

    if (scaledY >= 0)
    {
      if (scaledX >= 0)
      {
        // I quadrant
        leftMotorPower = maximum;
        rightMotorPower = difference;
      }
      else
      {            
        // II quadrant
        leftMotorPower = total;
        rightMotorPower = maximum;
      }
    }
    else
    {
        if (scaledX >= 0)
        {  
          // IV quadrant
          leftMotorPower = total;
          rightMotorPower = -maximum;
        }
        else
        {
          // III quadrant
          leftMotorPower = -maximum;
          rightMotorPower = difference;
        }
    }

    const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);
  }
}

