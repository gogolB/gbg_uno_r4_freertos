/**************************************************************************************
 * Potentiometer + Direction Switch Control
 *
 * Simple sketch to drive both motors on the DualVNH5019 shield from a single
 * throttle potentiometer and a direction toggle switch. No FreeRTOS or BLE.
 **************************************************************************************/

#include <Arduino.h>
#include <DualVNH5019MotorShield.h>

// Hardware configuration
constexpr uint8_t POT_PIN = A0;                // Throttle input
constexpr uint8_t DIRECTION_SWITCH_PIN = 2;    // Toggle switch input (uses INPUT_PULLUP)

// Motor shield configuration
constexpr int16_t MAX_MOTOR_OUTPUT = 400;      // DualVNH5019 expects -400..+400
constexpr int16_t OUTPUT_DEADBAND = 8;         // Ignore low throttle noise
constexpr int16_t RAMP_STEP = 5;               // Max delta per loop iteration
constexpr uint16_t ANALOG_FULL_SCALE = 4095;   // UNO R4 analogRead() max value
constexpr uint16_t LOOP_DELAY_MS = 10;         // Update rate ~100 Hz

DualVNH5019MotorShield motors;

void setup() {
  pinMode(POT_PIN, INPUT);
  pinMode(DIRECTION_SWITCH_PIN, INPUT_PULLUP); // LOW = forward, HIGH = reverse

  motors.init();

  // Ensure motors start stopped
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

void loop() {
  static int16_t currentCommand = 0; // Persisted motor command for ramping

  // Read throttle input and scale to motor command range
  int32_t raw = analogRead(POT_PIN);
  if (raw < 0) {
    raw = 0;
  } else if (raw > ANALOG_FULL_SCALE) {
    raw = ANALOG_FULL_SCALE;
  }

  int16_t magnitude = static_cast<int16_t>(
      (raw * MAX_MOTOR_OUTPUT) / ANALOG_FULL_SCALE);

  // Apply a small deadband so motors idle at zero near the knob's bottom end
  if (magnitude < OUTPUT_DEADBAND) {
    magnitude = 0;
  }

  // Switch selects direction. With INPUT_PULLUP wiring, LOW means forward.
  const bool forward = digitalRead(DIRECTION_SWITCH_PIN) == LOW;
  const int16_t commandedSpeed = forward ? magnitude : -magnitude;

  // Ramp toward target to soften starts/stops
  if (commandedSpeed > currentCommand + RAMP_STEP) {
    currentCommand += RAMP_STEP;
  } else if (commandedSpeed < currentCommand - RAMP_STEP) {
    currentCommand -= RAMP_STEP;
  } else {
    currentCommand = commandedSpeed;
  }

  motors.setM1Speed(currentCommand);
  motors.setM2Speed(currentCommand);

  delay(LOOP_DELAY_MS);
}
