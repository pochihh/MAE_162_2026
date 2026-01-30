/**
 * @file test_dc_motor_pid.ino
 * @brief Test sketch for DC motor PID control with current sense feedback
 *
 * This test verifies:
 * - Encoder counting and velocity estimation
 * - Position and velocity PID control loops
 * - Current sense (CT) feedback reading from H-bridge
 * - Motor direction control
 *
 * Hardware Requirements:
 * - DC motor connected to Motor 1 port (EN, IN1, IN2, ENC_A, ENC_B, CT)
 * - H-bridge module with current sense output connected to A3
 *
 * Serial Commands:
 * - 'p' : Position mode - move to target position
 * - 'v' : Velocity mode - maintain target velocity
 * - 'd' : Disable motor
 * - '+' : Increase target (position or velocity)
 * - '-' : Decrease target (position or velocity)
 * - 'r' : Reset encoder position to zero
 * - 's' : Print current status
 * - '?' : Print help
 *
 * Expected Behavior:
 * - Motor responds to position/velocity commands
 * - Serial output shows position, velocity, PWM, and current feedback
 * - PID parameters can be verified by observing response
 *
 * Compile:
 *   arduino-cli compile --fqbn arduino:avr:mega arduino/tests/test_dc_motor_pid
 *
 * Upload:
 *   arduino-cli upload -p /dev/cu.usbserial* --fqbn arduino:avr:mega arduino/tests/test_dc_motor_pid
 */

#include "../../src/config.h"
#include "../../src/pins.h"
#include "../../src/modules/EncoderCounter.h"
#include "../../src/modules/VelocityEstimator.h"
#include "../../src/drivers/DCMotor.h"

// ============================================================================
// TEST CONFIGURATION
// ============================================================================

// Which motor to test (0-3)
#define TEST_MOTOR_ID       0

// Serial output rate (Hz)
#define PRINT_RATE_HZ       10

// Position step size (encoder ticks)
#define POSITION_STEP       720     // Half revolution (1440 PPR / 2)

// Velocity step size (ticks/sec)
#define VELOCITY_STEP       500

// Current sense configuration
// The CT output from H-bridge modules varies by module type:
// - L298N: ~0.5V per amp (varies by version)
// - BTS7960: ~0.0067V per amp (very low)
// - IBT-2: Similar to BTS7960
// Adjust this multiplier based on your H-bridge module
#define CT_MV_PER_AMP       500.0f  // mV per Amp (adjust for your H-bridge)
#define ADC_REF_MV          5000.0f // Arduino 5V reference in mV
#define ADC_RESOLUTION      1024    // 10-bit ADC

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Encoder (2x mode for this test)
EncoderCounter2x encoder;

// Velocity estimator
EdgeTimeVelocityEstimator velocityEst;

// DC Motor controller
DCMotor motor;

// Test state
enum TestMode {
  MODE_DISABLED,
  MODE_POSITION,
  MODE_VELOCITY
};

TestMode currentMode = MODE_DISABLED;
int32_t targetPosition = 0;
int32_t targetVelocity = 0;

// Timing
unsigned long lastPrintTime = 0;
unsigned long lastUpdateTime = 0;

// Current sense pin based on motor ID
const uint8_t CT_PINS[4] = { PIN_M1_CT, PIN_M2_CT, PIN_M3_CT, PIN_M4_CT };

// ============================================================================
// ENCODER ISR
// ============================================================================

void encoderISR() {
  encoder.onInterruptA();
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Read current sense value and convert to milliamps
 * @return Current in milliamps (mA)
 */
int16_t readCurrentMA() {
  int rawADC = analogRead(CT_PINS[TEST_MOTOR_ID]);

  // Convert ADC to millivolts
  float millivolts = (rawADC * ADC_REF_MV) / ADC_RESOLUTION;

  // Convert millivolts to milliamps
  float milliamps = (millivolts / CT_MV_PER_AMP) * 1000.0f;

  return (int16_t)milliamps;
}

/**
 * @brief Print current motor status
 */
void printStatus() {
  Serial.println(F("----------------------------------------"));
  Serial.print(F("Mode: "));
  switch (currentMode) {
    case MODE_DISABLED:
      Serial.println(F("DISABLED"));
      break;
    case MODE_POSITION:
      Serial.print(F("POSITION (target: "));
      Serial.print(targetPosition);
      Serial.println(F(" ticks)"));
      break;
    case MODE_VELOCITY:
      Serial.print(F("VELOCITY (target: "));
      Serial.print(targetVelocity);
      Serial.println(F(" ticks/sec)"));
      break;
  }

  Serial.print(F("Position:  "));
  Serial.print(motor.getPosition());
  Serial.println(F(" ticks"));

  Serial.print(F("Velocity:  "));
  Serial.print(motor.getVelocity());
  Serial.println(F(" ticks/sec"));

  Serial.print(F("PWM Out:   "));
  Serial.println(motor.getPWMOutput());

  Serial.print(F("Current:   "));
  Serial.print(readCurrentMA());
  Serial.println(F(" mA"));

  Serial.print(F("CT Raw:    "));
  Serial.print(analogRead(CT_PINS[TEST_MOTOR_ID]));
  Serial.print(F(" ("));
  Serial.print((analogRead(CT_PINS[TEST_MOTOR_ID]) * ADC_REF_MV) / ADC_RESOLUTION);
  Serial.println(F(" mV)"));

  Serial.println(F("----------------------------------------"));
}

/**
 * @brief Print help message
 */
void printHelp() {
  Serial.println(F("\n=== DC Motor PID Test with Current Sense ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  p - Position control mode"));
  Serial.println(F("  v - Velocity control mode"));
  Serial.println(F("  d - Disable motor"));
  Serial.println(F("  + - Increase target"));
  Serial.println(F("  - - Decrease target"));
  Serial.println(F("  r - Reset encoder to zero"));
  Serial.println(F("  s - Print status"));
  Serial.println(F("  ? - Print this help"));
  Serial.println();
}

/**
 * @brief Print continuous telemetry line
 */
void printTelemetry() {
  // Format: time_ms, position, velocity, pwm, current_mA, current_raw
  Serial.print(millis());
  Serial.print(F(", "));
  Serial.print(motor.getPosition());
  Serial.print(F(", "));
  Serial.print(motor.getVelocity());
  Serial.print(F(", "));
  Serial.print(motor.getPWMOutput());
  Serial.print(F(", "));
  Serial.print(readCurrentMA());
  Serial.print(F(", "));
  Serial.println(analogRead(CT_PINS[TEST_MOTOR_ID]));
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize serial
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  Serial.println(F("\n========================================"));
  Serial.println(F("  DC Motor PID Test with Current Sense"));
  Serial.println(F("========================================\n"));

  // Print test configuration
  Serial.print(F("Testing Motor: "));
  Serial.println(TEST_MOTOR_ID + 1);
  Serial.print(F("Current Sense Pin: A"));
  Serial.println(CT_PINS[TEST_MOTOR_ID] - A0);
  Serial.println();

  // Initialize encoder
  uint8_t encAPins[4] = { PIN_M1_ENC_A, PIN_M2_ENC_A, PIN_M3_ENC_A, PIN_M4_ENC_A };
  uint8_t encBPins[4] = { PIN_M1_ENC_B, PIN_M2_ENC_B, PIN_M3_ENC_B, PIN_M4_ENC_B };

  encoder.init(encAPins[TEST_MOTOR_ID], encBPins[TEST_MOTOR_ID], false);
  Serial.println(F("[Init] Encoder initialized"));

  // Initialize velocity estimator
  uint16_t countsPerRev = ENCODER_PPR * encoder.getResolutionMultiplier();
  velocityEst.init(countsPerRev);
  velocityEst.setFilterSize(4);
  velocityEst.setZeroTimeout(50000);  // 50ms timeout for zero velocity
  Serial.print(F("[Init] Velocity estimator: "));
  Serial.print(countsPerRev);
  Serial.println(F(" counts/rev"));

  // Initialize motor controller
  uint8_t enPins[4] = { PIN_M1_EN, PIN_M2_EN, PIN_M3_EN, PIN_M4_EN };
  uint8_t in1Pins[4] = { PIN_M1_IN1, PIN_M2_IN1, PIN_M3_IN1, PIN_M4_IN1 };
  uint8_t in2Pins[4] = { PIN_M1_IN2, PIN_M2_IN2, PIN_M3_IN2, PIN_M4_IN2 };

  motor.init(TEST_MOTOR_ID, &encoder, &velocityEst, false);
  motor.setPins(enPins[TEST_MOTOR_ID], in1Pins[TEST_MOTOR_ID], in2Pins[TEST_MOTOR_ID]);
  motor.setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  motor.setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  Serial.println(F("[Init] Motor controller initialized"));

  // Attach encoder interrupt
  attachInterrupt(digitalPinToInterrupt(encAPins[TEST_MOTOR_ID]), encoderISR, CHANGE);
  Serial.println(F("[Init] Encoder ISR attached"));

  // Configure current sense pin as input
  pinMode(CT_PINS[TEST_MOTOR_ID], INPUT);
  Serial.println(F("[Init] Current sense pin configured"));

  // Print telemetry header
  Serial.println(F("\n[Telemetry] time_ms, position, velocity, pwm, current_mA, ct_raw"));

  printHelp();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  // ------------------------------------------------------------------------
  // Motor PID Update (200 Hz)
  // ------------------------------------------------------------------------
  if (now - lastUpdateTime >= 5) {  // 5ms = 200Hz
    lastUpdateTime = now;
    motor.update();
  }

  // ------------------------------------------------------------------------
  // Serial Output (configurable rate)
  // ------------------------------------------------------------------------
  if (now - lastPrintTime >= (1000 / PRINT_RATE_HZ)) {
    lastPrintTime = now;

    if (currentMode != MODE_DISABLED) {
      printTelemetry();
    }
  }

  // ------------------------------------------------------------------------
  // Serial Command Processing
  // ------------------------------------------------------------------------
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'p':  // Position mode
        currentMode = MODE_POSITION;
        targetPosition = motor.getPosition();  // Start from current position
        motor.enable();
        motor.setTargetPosition(targetPosition);
        Serial.println(F("\n[Mode] Position control enabled"));
        printStatus();
        break;

      case 'v':  // Velocity mode
        currentMode = MODE_VELOCITY;
        targetVelocity = 0;
        motor.enable();
        motor.setTargetVelocity(targetVelocity);
        Serial.println(F("\n[Mode] Velocity control enabled"));
        printStatus();
        break;

      case 'd':  // Disable
        currentMode = MODE_DISABLED;
        motor.disable();
        Serial.println(F("\n[Mode] Motor disabled"));
        printStatus();
        break;

      case '+':  // Increase target
        if (currentMode == MODE_POSITION) {
          targetPosition += POSITION_STEP;
          motor.setTargetPosition(targetPosition);
          Serial.print(F("\n[Target] Position: "));
          Serial.println(targetPosition);
        } else if (currentMode == MODE_VELOCITY) {
          targetVelocity += VELOCITY_STEP;
          motor.setTargetVelocity(targetVelocity);
          Serial.print(F("\n[Target] Velocity: "));
          Serial.println(targetVelocity);
        }
        break;

      case '-':  // Decrease target
        if (currentMode == MODE_POSITION) {
          targetPosition -= POSITION_STEP;
          motor.setTargetPosition(targetPosition);
          Serial.print(F("\n[Target] Position: "));
          Serial.println(targetPosition);
        } else if (currentMode == MODE_VELOCITY) {
          targetVelocity -= VELOCITY_STEP;
          motor.setTargetVelocity(targetVelocity);
          Serial.print(F("\n[Target] Velocity: "));
          Serial.println(targetVelocity);
        }
        break;

      case 'r':  // Reset encoder
        encoder.resetCount();
        targetPosition = 0;
        Serial.println(F("\n[Reset] Encoder position reset to 0"));
        break;

      case 's':  // Status
        printStatus();
        break;

      case '?':  // Help
        printHelp();
        break;

      case '\n':
      case '\r':
        // Ignore newlines
        break;

      default:
        Serial.print(F("\n[Error] Unknown command: "));
        Serial.println(cmd);
        break;
    }
  }
}
