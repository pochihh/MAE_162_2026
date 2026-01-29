/**
 * @file test_dc_motor_pwm.ino
 * @brief Test sketch for direct PWM motor control (Phase 3)
 *
 * This test verifies DC motor hardware interface by:
 * 1. Configuring motor control pins (EN, IN1, IN2)
 * 2. Sending direct PWM commands to motors
 * 3. Testing forward, reverse, and brake modes
 * 4. Verifying encoder feedback during motor operation
 *
 * Expected Behavior:
 * - Motor spins forward when commanded
 * - Motor spins backward when commanded
 * - Motor stops when brake commanded
 * - Encoder count changes during motor rotation
 * - Direction matches encoder sign
 *
 * How to Test:
 * 1. Upload this sketch
 * 2. Open Serial Monitor @ 115200 baud
 * 3. Motors will cycle through: Forward → Brake → Reverse → Brake
 * 4. Verify motor movement matches expected direction
 * 5. Verify encoder counts increase/decrease accordingly
 *
 * Hardware Setup:
 * - Connect motor drivers (EN, IN1, IN2 pins to H-bridge)
 * - Connect motor power supply
 * - Connect encoders for feedback
 * - Ensure motor shafts are free to spin
 *
 * Safety:
 * - Start with low PWM values (25%) to avoid mechanical damage
 * - Ensure nothing obstructs motor rotation
 * - Monitor current draw to detect stall conditions
 *
 * Verification:
 * 1. Compile success
 * 2. Motors spin when PWM applied
 * 3. Direction control works correctly
 * 4. Encoders track motion
 * 5. Motors stop on brake command
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/modules/EncoderCounter.h"

// ============================================================================
// ENCODER INSTANCES
// ============================================================================

#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif

#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif

// ============================================================================
// MOTOR CONTROL STATE
// ============================================================================

enum MotorState {
    STATE_FORWARD,
    STATE_BRAKE_AFTER_FWD,
    STATE_REVERSE,
    STATE_BRAKE_AFTER_REV
};

MotorState currentState = STATE_FORWARD;
uint32_t lastStateChange = 0;
const uint32_t STATE_DURATION_MS = 2000;  // 2 seconds per state

// Test PWM values (start conservative)
const int16_t TEST_PWM_FORWARD = 64;   // 25% duty cycle
const int16_t TEST_PWM_REVERSE = -64;  // 25% duty cycle (negative = reverse)

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

void encoderISR_M1() {
    encoder1.onInterruptA();
}

void encoderISR_M2() {
    encoder2.onInterruptA();
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Set motor PWM and direction
 *
 * @param motorId Motor index (0-3)
 * @param pwm PWM value (-255 to +255, negative = reverse)
 */
void setMotorPWM(uint8_t motorId, int16_t pwm) {
    uint8_t pinEN, pinIN1, pinIN2, dir;

    // Select pins based on motor ID
    switch (motorId) {
        case 0:
            pinEN = PIN_M1_EN;
            pinIN1 = PIN_M1_IN1;
            pinIN2 = PIN_M1_IN2;
            dir = DC_MOTOR_1_DIR_INVERTED;
            break;
        case 1:
            pinEN = PIN_M2_EN;
            pinIN1 = PIN_M2_IN1;
            pinIN2 = PIN_M2_IN2;
            dir = DC_MOTOR_2_DIR_INVERTED;
            break;
#if DC_MOTOR_3_ENABLED
        case 2:
            pinEN = PIN_M3_EN;
            pinIN1 = PIN_M3_IN1;
            pinIN2 = PIN_M3_IN2;
            dir = DC_MOTOR_3_DIR_INVERTED;
            break;
#endif
#if DC_MOTOR_4_ENABLED
        case 3:
            pinEN = PIN_M4_EN;
            pinIN1 = PIN_M4_IN1;
            pinIN2 = PIN_M4_IN2;
            dir = DC_MOTOR_4_DIR_INVERTED;
            break;
#endif
        default:
            return;  // Invalid motor ID
    }

    // Clamp PWM to valid range
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;

    pwm = dir ? -pwm : pwm;  // Apply direction inversion if needed
    
    // Set direction and PWM
    if (pwm > 0) {
        // Forward
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
        analogWrite(pinEN, (uint8_t)pwm);
    } else if (pwm < 0) {
        // Reverse
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
        analogWrite(pinEN, (uint8_t)(-pwm));
    } else {
        // Brake (short motor terminals)
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);
        analogWrite(pinEN, 0);
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize Debug Serial
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000); // Wait for connection

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("  DC Motor PWM Test - Phase 3"));
    DEBUG_SERIAL.println(F("========================================"));

    // Configure motor control pins
    pinMode(PIN_M1_EN, OUTPUT);
    pinMode(PIN_M1_IN1, OUTPUT);
    pinMode(PIN_M1_IN2, OUTPUT);

    pinMode(PIN_M2_EN, OUTPUT);
    pinMode(PIN_M2_IN1, OUTPUT);
    pinMode(PIN_M2_IN2, OUTPUT);

#if DC_MOTOR_3_ENABLED
    pinMode(PIN_M3_EN, OUTPUT);
    pinMode(PIN_M3_IN1, OUTPUT);
    pinMode(PIN_M3_IN2, OUTPUT);
#endif

#if DC_MOTOR_4_ENABLED
    pinMode(PIN_M4_EN, OUTPUT);
    pinMode(PIN_M4_IN1, OUTPUT);
    pinMode(PIN_M4_IN2, OUTPUT);
#endif

    // Initialize all motors to stopped state
    setMotorPWM(0, 0);
    setMotorPWM(1, 0);
#if DC_MOTOR_3_ENABLED
    setMotorPWM(2, 0);
#endif
#if DC_MOTOR_4_ENABLED
    setMotorPWM(3, 0);
#endif

    DEBUG_SERIAL.println(F("[Setup] Motor control pins configured"));

    // Initialize encoders (with direction flags from config.h)
    encoder1.init(PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED);
    encoder2.init(PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED);

    // Attach interrupt handlers
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2, CHANGE);

    DEBUG_SERIAL.println(F("[Setup] Encoders initialized"));

    // Print test sequence
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("Test Sequence (2s per state):"));
    DEBUG_SERIAL.print(F("  1. Forward @ PWM "));
    DEBUG_SERIAL.println(TEST_PWM_FORWARD);
    DEBUG_SERIAL.println(F("  2. Brake"));
    DEBUG_SERIAL.print(F("  3. Reverse @ PWM "));
    DEBUG_SERIAL.println(TEST_PWM_REVERSE);
    DEBUG_SERIAL.println(F("  4. Brake"));
    DEBUG_SERIAL.println(F("  (Repeat)"));
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("Safety: Starting with 25% PWM"));
    DEBUG_SERIAL.println(F("========================================"));

    lastStateChange = millis();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    uint32_t currentTime = millis();

    // Check if it's time to change state
    if (currentTime - lastStateChange >= STATE_DURATION_MS) {
        lastStateChange = currentTime;

        // Advance to next state
        switch (currentState) {
            case STATE_FORWARD:
                currentState = STATE_BRAKE_AFTER_FWD;
                setMotorPWM(0, 0);
                setMotorPWM(1, 0);
                DEBUG_SERIAL.println(F("[State] BRAKE (after forward)"));
                break;

            case STATE_BRAKE_AFTER_FWD:
                currentState = STATE_REVERSE;
                setMotorPWM(0, TEST_PWM_REVERSE);
                setMotorPWM(1, TEST_PWM_REVERSE);
                DEBUG_SERIAL.print(F("[State] REVERSE @ PWM "));
                DEBUG_SERIAL.println(TEST_PWM_REVERSE);
                break;

            case STATE_REVERSE:
                currentState = STATE_BRAKE_AFTER_REV;
                setMotorPWM(0, 0);
                setMotorPWM(1, 0);
                DEBUG_SERIAL.println(F("[State] BRAKE (after reverse)"));
                break;

            case STATE_BRAKE_AFTER_REV:
                currentState = STATE_FORWARD;
                setMotorPWM(0, TEST_PWM_FORWARD);
                setMotorPWM(1, TEST_PWM_FORWARD);
                DEBUG_SERIAL.print(F("[State] FORWARD @ PWM "));
                DEBUG_SERIAL.println(TEST_PWM_FORWARD);
                break;
        }
    }

    // Print encoder feedback every 200ms
    static uint32_t lastPrint = 0;
    if (currentTime - lastPrint >= 200) {
        lastPrint = currentTime;

        int32_t count1 = encoder1.getCount();
        int32_t count2 = encoder2.getCount();

        DEBUG_SERIAL.print(F("  Encoder M1: "));
        DEBUG_SERIAL.print(count1);
        DEBUG_SERIAL.print(F("  |  M2: "));
        DEBUG_SERIAL.println(count2);
    }
}
