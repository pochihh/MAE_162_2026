/**
 * @file test_dc_motor_pid.ino
 * @brief Test sketch for DC motor PID control (Phase 3)
 *
 * This test verifies complete closed-loop motor control by:
 * 1. Initializing DCMotor instances with encoders and velocity estimators
 * 2. Testing velocity control mode
 * 3. Testing position control mode
 * 4. Verifying PID tracking performance
 *
 * Expected Behavior:
 * - Motor velocity tracks commanded setpoint
 * - Motor position reaches target with minimal overshoot
 * - PID loop runs at 200Hz (5ms period)
 * - Velocity estimation updates smoothly
 *
 * How to Test:
 * 1. Upload this sketch
 * 2. Open Serial Monitor @ 115200 baud
 * 3. Motors will test velocity control, then position control
 * 4. Observe setpoint tracking and PID output
 * 5. Tune PID gains if needed via serial commands
 *
 * Hardware Setup:
 * - Connect motors with encoders
 * - Connect motor power supply
 * - Ensure motor shafts are free to spin
 * - No mechanical load initially (tune PID first)
 *
 * Safety:
 * - PID limits PWM output to ±255
 * - Position control limits velocity setpoint
 * - Watch for integral windup on tracking errors
 *
 * Verification:
 * 1. Compile success
 * 2. Velocity control tracks setpoint within ±10%
 * 3. Position control reaches target within ±5 ticks
 * 4. No sustained oscillations
 * 5. Steady-state error < 5% for velocity
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/modules/EncoderCounter.h"
#include "src/modules/VelocityEstimator.h"
#include "src/drivers/DCMotor.h"

// ============================================================================
// ENCODER AND VELOCITY ESTIMATOR INSTANCES
// ============================================================================

// Motor 1
#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif
EdgeTimeVelocityEstimator velocityEst1;

// Motor 2
#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif
EdgeTimeVelocityEstimator velocityEst2;

// ============================================================================
// DC MOTOR INSTANCES
// ============================================================================

DCMotor motor1;
DCMotor motor2;

// ============================================================================
// TEST STATE MACHINE
// ============================================================================

enum TestState {
    TEST_INIT,
    TEST_VELOCITY_POSITIVE,
    TEST_VELOCITY_NEGATIVE,
    TEST_VELOCITY_ZERO,
    TEST_POSITION_FORWARD,
    TEST_POSITION_REVERSE,
    TEST_DONE
};

TestState currentState = TEST_INIT;
uint32_t stateStartTime = 0;
const uint32_t STATE_DURATION_MS = 5000;  // 5 seconds per test

// Test setpoints
const float TEST_VELOCITY_POS = 500.0f;   // 500 ticks/sec forward
const float TEST_VELOCITY_NEG = -500.0f;  // 500 ticks/sec reverse
const int32_t TEST_POSITION_FWD = 720;    // 1 revolution forward (2x mode)
const int32_t TEST_POSITION_REV = -720;   // 1 revolution reverse

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
// SCHEDULER TASKS
// ============================================================================

/**
 * @brief Motor PID update task (200Hz)
 */
void taskMotorPID() {
    motor1.update();
    motor2.update();
}

/**
 * @brief Status printing task (5Hz)
 */
void taskPrintStatus() {
    DEBUG_SERIAL.print(F("M1: "));
    DEBUG_SERIAL.print(motor1.getPosition());
    DEBUG_SERIAL.print(F(" ticks, "));
    DEBUG_SERIAL.print(motor1.getVelocity());
    DEBUG_SERIAL.print(F(" t/s, PWM="));
    DEBUG_SERIAL.print(motor1.getPWMOutput());

    DEBUG_SERIAL.print(F("  |  M2: "));
    DEBUG_SERIAL.print(motor2.getPosition());
    DEBUG_SERIAL.print(F(" ticks, "));
    DEBUG_SERIAL.print(motor2.getVelocity());
    DEBUG_SERIAL.print(F(" t/s, PWM="));
    DEBUG_SERIAL.println(motor2.getPWMOutput());
}

/**
 * @brief Test state machine task (1Hz)
 */
void taskTestStateMachine() {
    uint32_t currentTime = millis();

    // Check if it's time to advance state
    if (currentTime - stateStartTime < STATE_DURATION_MS) {
        return;  // Still in current state
    }

    stateStartTime = currentTime;

    // Advance to next state
    switch (currentState) {
        case TEST_INIT:
            DEBUG_SERIAL.println(F("\n=== TEST: Velocity Control (Positive) ==="));
            motor1.enable(DC_MODE_VELOCITY);
            motor2.enable(DC_MODE_VELOCITY);
            motor1.setTargetVelocity(TEST_VELOCITY_POS);
            motor2.setTargetVelocity(TEST_VELOCITY_POS);
            currentState = TEST_VELOCITY_POSITIVE;
            break;

        case TEST_VELOCITY_POSITIVE:
            DEBUG_SERIAL.println(F("\n=== TEST: Velocity Control (Negative) ==="));
            motor1.setTargetVelocity(TEST_VELOCITY_NEG);
            motor2.setTargetVelocity(TEST_VELOCITY_NEG);
            currentState = TEST_VELOCITY_NEGATIVE;
            break;

        case TEST_VELOCITY_NEGATIVE:
            DEBUG_SERIAL.println(F("\n=== TEST: Velocity Control (Zero) ==="));
            motor1.setTargetVelocity(0);
            motor2.setTargetVelocity(0);
            currentState = TEST_VELOCITY_ZERO;
            break;

        case TEST_VELOCITY_ZERO:
            DEBUG_SERIAL.println(F("\n=== TEST: Position Control (Forward) ==="));
            motor1.enable(DC_MODE_POSITION);
            motor2.enable(DC_MODE_POSITION);
            encoder1.resetCount();
            encoder2.resetCount();
            motor1.setTargetPosition(TEST_POSITION_FWD);
            motor2.setTargetPosition(TEST_POSITION_FWD);
            currentState = TEST_POSITION_FORWARD;
            break;

        case TEST_POSITION_FORWARD:
            DEBUG_SERIAL.println(F("\n=== TEST: Position Control (Reverse) ==="));
            motor1.setTargetPosition(TEST_POSITION_REV);
            motor2.setTargetPosition(TEST_POSITION_REV);
            currentState = TEST_POSITION_REVERSE;
            break;

        case TEST_POSITION_REVERSE:
            DEBUG_SERIAL.println(F("\n=== TEST COMPLETE ==="));
            motor1.disable();
            motor2.disable();
            currentState = TEST_DONE;
            break;

        case TEST_DONE:
            // Stay in done state
            break;
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
    DEBUG_SERIAL.println(F("  DC Motor PID Control Test - Phase 3"));
    DEBUG_SERIAL.println(F("========================================"));

    // Initialize Scheduler
    Scheduler::init();
    DEBUG_SERIAL.println(F("[Setup] Scheduler initialized"));

    // Initialize encoders
    encoder1.init(PIN_M1_ENC_A, PIN_M1_ENC_B);
    encoder2.init(PIN_M2_ENC_A, PIN_M2_ENC_B);
    DEBUG_SERIAL.println(F("[Setup] Encoders initialized"));

    // Attach interrupt handlers
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2, CHANGE);
    DEBUG_SERIAL.println(F("[Setup] Encoder interrupts attached"));

    // Initialize velocity estimators
    uint16_t countsPerRev = ENCODER_PPR * encoder1.getResolutionMultiplier();
    velocityEst1.init(countsPerRev);
    velocityEst1.setFilterSize(VELOCITY_FILTER_SIZE);
    velocityEst1.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

    velocityEst2.init(countsPerRev);
    velocityEst2.setFilterSize(VELOCITY_FILTER_SIZE);
    velocityEst2.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

    DEBUG_SERIAL.println(F("[Setup] Velocity estimators initialized"));
    DEBUG_SERIAL.print(F("  - Counts per rev: "));
    DEBUG_SERIAL.println(countsPerRev);

    // Initialize motors
    motor1.init(0, &encoder1, &velocityEst1);
    motor1.setPins(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2);
    motor1.setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
    motor1.setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);

    motor2.init(1, &encoder2, &velocityEst2);
    motor2.setPins(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2);
    motor2.setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
    motor2.setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);

    DEBUG_SERIAL.println(F("[Setup] Motors initialized"));
    DEBUG_SERIAL.print(F("  - Position PID: Kp="));
    DEBUG_SERIAL.print(DEFAULT_POS_KP);
    DEBUG_SERIAL.print(F(", Ki="));
    DEBUG_SERIAL.print(DEFAULT_POS_KI);
    DEBUG_SERIAL.print(F(", Kd="));
    DEBUG_SERIAL.println(DEFAULT_POS_KD);
    DEBUG_SERIAL.print(F("  - Velocity PID: Kp="));
    DEBUG_SERIAL.print(DEFAULT_VEL_KP);
    DEBUG_SERIAL.print(F(", Ki="));
    DEBUG_SERIAL.print(DEFAULT_VEL_KI);
    DEBUG_SERIAL.print(F(", Kd="));
    DEBUG_SERIAL.println(DEFAULT_VEL_KD);

    // Register scheduler tasks
    Scheduler::registerTask(taskMotorPID, 5, 0);           // 5ms (200Hz)
    Scheduler::registerTask(taskPrintStatus, 200, 1);      // 200ms (5Hz)
    Scheduler::registerTask(taskTestStateMachine, 1000, 2); // 1000ms (1Hz)

    DEBUG_SERIAL.println(F("[Setup] Tasks registered"));
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("Test Sequence (5s per state):"));
    DEBUG_SERIAL.println(F("  1. Velocity +500 t/s"));
    DEBUG_SERIAL.println(F("  2. Velocity -500 t/s"));
    DEBUG_SERIAL.println(F("  3. Velocity 0 t/s"));
    DEBUG_SERIAL.println(F("  4. Position +720 ticks"));
    DEBUG_SERIAL.println(F("  5. Position -720 ticks"));
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println();

    stateStartTime = millis();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    Scheduler::tick();
}
