/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_FreeRTOS.h>

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

TaskHandle_t loop_task, blinky_task, motor_drive_task;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while (!Serial) { }

#if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
#endif

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
      motor_drive_task
    );
  
  if (rc_motor_drive != pdPASS) {
    Serial.println("Failed to create 'motor drive' thread")
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
#if defined(ARDUINO_PORTENTA_C33)
  /* Only the Portenta C33 has an RGB LED. */
  digitalWrite(LEDR, !digitalRead(LEDR));
#else
  Serial.println(millis());
#endif
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

// Motor pin definations. Should not be too different given the shield.
#define MOTOR_A_DIR_PIN 12
#define MOTOR_B_DIR_PIN 13
#define MOTOR_A_PWM_PIN 10
#define MOTOR_B_PWM_PIN 11
#define REV_LEFT_DRIVE true       // Change these if the motors are wired backwards.
#define REV_RIGHT_DRIVE true      // Change these if the motors are wired backwards.

const int MAX_FWD_LEFT_MOTOR_POWER = 128;
const int MAX_FWD_RIGHT_MOTOR_POWER = 128;
const int MAX_BWD_LEFT_MOTOR_POWER = 128;
const int MAX_BWD_RIGHT_MOTOR_POWER = 128;

// This is the motor power. A power of greater than 0 is forward.
// A power of less than 0 is backward.
int leftMotorPower = 0;
int rightMotorPower = 0;

 /*
  * This is the motor control function that is primarily responsible 
  * for setting motor values once they are calculated.
  */
void motor_drive_func(void *pvParams) 
{
  // Setup()
  const int leftMotorDirPin = MOTOR_A_DIR_PIN;
  const int leftMotorPWMPin = MOTOR_A_PWM_PIN;
  const int rightMotorDirPin = MOTOR_B_DIR_PIN;
  const int rightMotorPWMPin = MOTOR_B_PWM_PIN;
  
  leftMotorPower = 0;
  rightMotorPower = 0;

  pinMode(MOTOR_A_DIR_PIN, OUTPUT);
  pinMode(MOTOR_B_DIR_PIN, OUTPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);

  // Set all the motors to the initial value of zero.
  analogWrite(MOTOR_A_PWM_PIN, 0);
  analogWrite(MOTOR_B_PWM_PIN, 0);

  int leftMotorAppliedPower = 0;
  int rightMotorAppliedPower = 0;
  // loop()
  for(;;)
  {

    // Motor power is constrained to between -100% to 100%
    // This is then mapped to the PWM range necessary for use on the motor driver.


    leftMotorPower = constrain(leftMotorPower, -100, 100);
    rightMotorPower = constrain(rightMotorPower, -100, 100);
    
    
    if (leftMotorPower > 0) {
      leftMotorAppliedPower = map(leftMotorPower, 1, MAX_FWD_LEFT_MOTOR_POWER);
      #if REV_LEFT_DRIVE
        digitalWrite(leftMotorDirPin, LOW);
      #else
        digitalWrite(leftMotorDirPin, HIGH);
      #endif
      analogWrite(leftMotorPWMPin, leftMotorAppliedPower);
    } else if (leftMotorPower < 0) {
      leftMotorAppliedPower = map(abs(leftMotorPower), 1, MAX_BWD_LEFT_MOTOR_POWER);
      #if REV_LEFT_DRIVE
        digitalWrite(leftMotorDirPin, HIGH);
      #else
        digitalWrite(leftMotorDirPin, LOW);
      #endif
      analogWrite(leftMotorPWMPin, leftMotorAppliedPower);
    } else {
      analogWrite(leftMotorPWMPin, 0);
    }

    if (rightMotorPower > 0) {
      rightMotorAppliedPower = map(rightMotorPower, 1, MAX_FWD_RIGHT_MOTOR_POWER);
      #if REV_LEFT_DRIVE
        digitalWrite(rightMotorDirPin, LOW);
      #else
        digitalWrite(rightMotorDirPin, HIGH);
      #endif
      analogWrite(rightMotorPWMPin, rightMotorAppliedPower);
    } else if (rightMotorPower < 0) {
      rightMotorAppliedPower = constrain(abs(rightMotorPower), 1, MAX_BWD_RIGHT_MOTOR_POWER);
      #if REV_LEFT_DRIVE
        digitalWrite(rightMotorDirPin, HIGH);
      #else
        digitalWrite(rightMotorDirPin, LOW);
      #endif
      analogWrite(rightMotorPWMPin, rightMotorAppliedPower);
    } else {
      analogWrite(rightMotorPWMPin, 0);
    }

    vTaskDelay(configTICK_RATE_HZ);
  }

}



#define JOYSTICK_X_AXIS_PIN A0
#define JOYSTICK_Y_AXIS_PIN A1

#define FLIP_X_AXIS false
#define FLIP_Y_AXIS false

void joystick_func(void *pvParams)
{
  // setup()
  int xValue = 0;
  int yValue = 0;

  #if FLIP_X_AXIS
    const int xFlip = -1;
  #else
    const int xFlip = 1;
  #endif

  #if FLIP_Y_AXIS
    const int yFlip = -1;
  #else
    cosnt int yFlip = 1;
  #endif

  for(;;)
  {
    // read analog X and Y analog values
    xValue = analogRead(JOYSTICK_X_AXIS_PIN);
    yValue = analogRead(JOYSTICK_Y_AXIS_PIN);


  }
}

