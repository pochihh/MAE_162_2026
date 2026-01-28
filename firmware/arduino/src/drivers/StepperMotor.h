/**
 * @file StepperMotor.h
 * @brief Stepper motor driver with acceleration support
 *
 * This module provides individual stepper motor control with:
 * - Trapezoidal velocity profiles (accelerate → cruise → decelerate)
 * - Absolute and relative positioning
 * - Limit switch homing support
 * - Emergency stop capability
 *
 * Control Flow:
 *   moveSteps(N) or moveToPosition(P)
 *          │
 *          ├──> Calculate motion profile (accel/cruise/decel phases)
 *          │
 *          └──> timerCallback() @ 10kHz
 *                    ├── Decrement step interval counter
 *                    ├── Toggle STEP pin when ready
 *                    ├── Update step interval for acceleration
 *                    └── Check for motion complete / limit switch
 *
 * Hardware Interface:
 * - STEP pin: Pulse output (rising edge triggers step)
 * - DIR pin: Direction control (HIGH/LOW)
 * - ENABLE pin: Driver enable (active LOW typically)
 * - Limit switch: Optional homing reference
 *
 * Usage:
 *   StepperMotor stepper;
 *   stepper.init(0);  // Stepper ID 0
 *   stepper.setPins(PIN_STEP, PIN_DIR, PIN_EN);
 *   stepper.setMaxVelocity(1000);    // steps/sec
 *   stepper.setAcceleration(500);    // steps/sec²
 *   stepper.enable();
 *   stepper.moveSteps(200);          // Move 200 steps
 *
 *   // In Timer3 ISR @ 10kHz:
 *   stepper.timerCallback();
 */

#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>
#include <stdint.h>
#include "../messages/TLV_Payloads.h"  // For StepperState enum

// StepperState enum is defined in TLV_Payloads.h:
// STEPPER_IDLE, STEPPER_ACCEL, STEPPER_CRUISE, STEPPER_DECEL, STEPPER_HOMING

// ============================================================================
// STEPPER MOTOR CLASS
// ============================================================================

/**
 * @brief Individual stepper motor with acceleration support
 *
 * Implements trapezoidal motion profiles using Bresenham-style integer math.
 * Step timing is handled by Timer3 ISR calling timerCallback() at 10kHz.
 *
 * Motion profile calculation uses the kinematic equations:
 * - v² = v₀² + 2*a*d  (velocity after distance d)
 * - d = v₀*t + 0.5*a*t²  (distance after time t)
 *
 * Step interval is computed as: interval = timer_freq / velocity
 * During acceleration, interval decreases each step.
 * During deceleration, interval increases each step.
 */
class StepperMotor {
public:
    StepperMotor();

    /**
     * @brief Initialize stepper motor
     *
     * @param stepperId Stepper identifier (0-3)
     */
    void init(uint8_t stepperId);

    /**
     * @brief Configure hardware pins
     *
     * @param pinStep Step output pin
     * @param pinDir Direction output pin
     * @param pinEnable Enable output pin (active LOW)
     */
    void setPins(uint8_t pinStep, uint8_t pinDir, uint8_t pinEnable);

    /**
     * @brief Set limit switch pin for homing
     *
     * @param pinLimit Limit switch input pin
     * @param activeState State when limit is triggered (HIGH or LOW)
     */
    void setLimitPin(uint8_t pinLimit, uint8_t activeState);

    /**
     * @brief Enable stepper driver
     *
     * Activates the driver enable pin (typically active LOW).
     */
    void enable();

    /**
     * @brief Disable stepper driver
     *
     * Deactivates the driver enable pin (motor can freewheel).
     */
    void disable();

    /**
     * @brief Check if stepper is enabled
     *
     * @return True if driver is enabled
     */
    bool isEnabled() const { return enabled_; }

    // ========================================================================
    // MOTION PARAMETERS
    // ========================================================================

    /**
     * @brief Set maximum velocity
     *
     * @param stepsPerSec Maximum velocity in steps/second
     */
    void setMaxVelocity(uint16_t stepsPerSec);

    /**
     * @brief Set acceleration rate
     *
     * @param stepsPerSecSq Acceleration in steps/second²
     */
    void setAcceleration(uint16_t stepsPerSecSq);

    /**
     * @brief Get current maximum velocity setting
     *
     * @return Maximum velocity in steps/second
     */
    uint16_t getMaxVelocity() const { return maxVelocity_; }

    /**
     * @brief Get current acceleration setting
     *
     * @return Acceleration in steps/second²
     */
    uint16_t getAcceleration() const { return acceleration_; }

    // ========================================================================
    // MOTION COMMANDS
    // ========================================================================

    /**
     * @brief Move relative number of steps
     *
     * @param steps Number of steps (positive = forward, negative = reverse)
     */
    void moveSteps(int32_t steps);

    /**
     * @brief Move to absolute position
     *
     * @param position Target position in steps
     */
    void moveToPosition(int32_t position);

    /**
     * @brief Start homing sequence
     *
     * Moves in specified direction until limit switch triggers,
     * then sets position to zero.
     *
     * @param direction Direction to move (1 = positive, -1 = negative)
     */
    void home(int8_t direction);

    /**
     * @brief Emergency stop (immediate, no deceleration)
     *
     * Stops motor immediately without deceleration ramp.
     * Use for emergency situations only.
     */
    void stop();

    /**
     * @brief Smooth stop with deceleration
     *
     * Begins deceleration ramp to stop motor smoothly.
     */
    void smoothStop();

    // ========================================================================
    // STATE QUERIES
    // ========================================================================

    /**
     * @brief Get current state
     *
     * @return Current stepper state
     */
    StepperState getState() const { return state_; }

    /**
     * @brief Check if motor is moving
     *
     * @return True if in ACCEL, CRUISE, DECEL, or HOMING state
     */
    bool isMoving() const { return state_ != STEPPER_IDLE; }

    /**
     * @brief Get current position
     *
     * @return Current position in steps
     */
    int32_t getPosition() const { return currentPosition_; }

    /**
     * @brief Get target position
     *
     * @return Target position in steps
     */
    int32_t getTargetPosition() const { return targetPosition_; }

    /**
     * @brief Get steps remaining in current move
     *
     * @return Steps remaining to target
     */
    int32_t getStepsRemaining() const { return stepsRemaining_; }

    /**
     * @brief Set current position (for homing)
     *
     * @param position New position value
     */
    void setPosition(int32_t position);

    /**
     * @brief Get stepper ID
     *
     * @return Stepper identifier (0-3)
     */
    uint8_t getStepperId() const { return stepperId_; }

    // ========================================================================
    // TIMER CALLBACK (called from Timer3 ISR @ 10kHz)
    // ========================================================================

    /**
     * @brief Timer callback for step generation
     *
     * Called from Timer3 ISR at 10kHz (100µs period).
     * Generates step pulses and updates motion profile.
     *
     * CRITICAL: Must complete in <10µs to allow 4 steppers + overhead.
     */
    void timerCallback();

private:
    // Hardware configuration
    uint8_t stepperId_;         // Stepper identifier (0-3)
    uint8_t pinStep_;           // Step output pin
    uint8_t pinDir_;            // Direction output pin
    uint8_t pinEnable_;         // Enable output pin
    uint8_t pinLimit_;          // Limit switch input pin
    uint8_t limitActiveState_;  // State when limit triggered
    bool    hasLimit_;          // Limit switch configured
    bool    enabled_;           // Driver enabled state

    // Motion parameters
    uint16_t maxVelocity_;      // Max velocity (steps/sec)
    uint16_t acceleration_;     // Acceleration (steps/sec²)

    // Motion state
    StepperState state_;        // Current motion state
    int32_t currentPosition_;   // Current position (steps)
    int32_t targetPosition_;    // Target position (steps)
    int32_t stepsRemaining_;    // Steps left in current move
    int8_t  direction_;         // Current direction (+1 or -1)

    // Step timing (using 10kHz timer)
    uint16_t stepInterval_;     // Current step interval (timer ticks)
    uint16_t stepCounter_;      // Counter for next step
    uint16_t minInterval_;      // Minimum interval (at max velocity)

    // Acceleration tracking
    uint32_t accelSteps_;       // Steps in acceleration phase
    uint32_t decelSteps_;       // Steps in deceleration phase
    uint32_t cruiseSteps_;      // Steps in cruise phase
    uint32_t stepCount_;        // Steps taken in current move
    float    currentVelocity_;  // Current velocity (for accel calc)

    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    /**
     * @brief Calculate motion profile for move
     *
     * Computes acceleration, cruise, and deceleration phases.
     *
     * @param totalSteps Total steps to move
     */
    void calculateProfile(int32_t totalSteps);

    /**
     * @brief Update step interval during acceleration
     */
    void updateAccelInterval();

    /**
     * @brief Update step interval during deceleration
     */
    void updateDecelInterval();

    /**
     * @brief Check limit switch state
     *
     * @return True if limit switch is triggered
     */
    bool isLimitTriggered() const;
};

#endif // STEPPERMOTOR_H
