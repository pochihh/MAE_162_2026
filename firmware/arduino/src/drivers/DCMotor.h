/**
 * @file DCMotor.h
 * @brief DC motor driver with cascade PID control
 *
 * This module provides closed-loop control for DC motors with quadrature
 * encoders using a cascade PID architecture:
 * - Position mode: Position PID → Velocity PID → PWM
 * - Velocity mode: Velocity PID → PWM
 *
 * Control Flow:
 *   Target Position/Velocity
 *          │
 *          ├──> [Position PID] ──> Velocity Setpoint (position mode only)
 *          │
 *          └──> [Velocity PID] ──> PWM Output
 *                    ↑
 *                    └── Velocity Feedback (from VelocityEstimator)
 *
 * Hardware Interface:
 * - PWM output (EN pin): Speed control via analogWrite()
 * - Direction outputs (2 pins): IN1/IN2 for H-bridge control
 * - Encoder inputs (2 pins): Handled by EncoderCounter module
 *
 * Usage:
 *   DCMotor motor;
 *   EncoderCounter2x encoder;
 *   EdgeTimeVelocityEstimator velocityEst;
 *
 *   motor.init(0, &encoder, &velocityEst);  // Motor ID 0
 *   motor.setVelocityPID(1.0, 0.1, 0.01);   // Tune PID gains
 *   motor.enable(DCMotor::VELOCITY);        // Enable velocity mode
 *   motor.setTargetVelocity(1000);          // 1000 ticks/sec
 *
 *   // In PID loop @ 200Hz:
 *   motor.update();
 */

#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>
#include <stdint.h>
#include "../modules/EncoderCounter.h"
#include "../modules/VelocityEstimator.h"
#include "../messages/TLV_Payloads.h"  // For DCMotorMode enum

// ============================================================================
// PID CONTROLLER
// ============================================================================

/**
 * @brief Simple PID controller with anti-windup
 *
 * Implements standard PID control with:
 * - Proportional gain (Kp)
 * - Integral gain (Ki) with anti-windup clamping
 * - Derivative gain (Kd) with derivative-on-measurement to avoid setpoint kick
 *
 * Output = Kp*error + Ki*integral + Kd*derivative
 */
class PIDController {
public:
    PIDController();

    /**
     * @brief Set PID gains
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief Set output limits (for anti-windup)
     *
     * @param min Minimum output value
     * @param max Maximum output value
     */
    void setLimits(float min, float max);

    /**
     * @brief Compute PID output
     *
     * @param setpoint Target value
     * @param measurement Current value
     * @param dt Time step in seconds
     * @return Control output (clamped to limits)
     */
    float compute(float setpoint, float measurement, float dt);

    /**
     * @brief Reset integral term and previous error
     */
    void reset();

    /**
     * @brief Get last computed output
     *
     * @return Last output value
     */
    float getOutput() const { return output_; }

private:
    float kp_;              // Proportional gain
    float ki_;              // Integral gain
    float kd_;              // Derivative gain
    float integral_;        // Accumulated integral term
    float prevMeasurement_; // Previous measurement (for derivative-on-measurement)
    float output_;          // Last computed output
    float minOutput_;       // Minimum output limit
    float maxOutput_;       // Maximum output limit
};

// ============================================================================
// DC MOTOR CLASS
// ============================================================================

/**
 * @brief DC motor with cascade PID control
 *
 * Manages a single DC motor with encoder feedback. Supports:
 * - Position control mode (cascade PID)
 * - Velocity control mode (single PID)
 * - Disabled mode (motor off)
 *
 * Motor ID is used for debug output and TLV messaging.
 */
class DCMotor {
public:
    DCMotor();

    /**
     * @brief Initialize motor
     *
     * @param motorId Motor identifier (0-3)
     * @param encoder Pointer to encoder counter instance
     * @param velocityEst Pointer to velocity estimator instance
     */
    void init(uint8_t motorId, IEncoderCounter *encoder, IVelocityEstimator *velocityEst);

    /**
     * @brief Configure hardware pins
     *
     * @param pinEN PWM pin for motor speed (must support analogWrite)
     * @param pinIN1 Direction pin 1
     * @param pinIN2 Direction pin 2
     */
    void setPins(uint8_t pinEN, uint8_t pinIN1, uint8_t pinIN2);

    /**
     * @brief Enable motor in specified control mode
     *
     * @param mode Control mode (POSITION or VELOCITY)
     */
    void enable(DCMotorMode mode);

    /**
     * @brief Disable motor (set PWM to 0)
     */
    void disable();

    /**
     * @brief Check if motor is enabled
     *
     * @return True if motor is in POSITION or VELOCITY mode
     */
    bool isEnabled() const;

    /**
     * @brief Get current control mode
     *
     * @return Current mode (DISABLED, POSITION, or VELOCITY)
     */
    DCMotorMode getMode() const { return mode_; }

    // ========================================================================
    // CONTROL SETPOINTS
    // ========================================================================

    /**
     * @brief Set target position (position mode only)
     *
     * @param position Target position in encoder ticks
     */
    void setTargetPosition(int32_t position);

    /**
     * @brief Set target velocity (position or velocity mode)
     *
     * @param velocity Target velocity in ticks/second
     */
    void setTargetVelocity(float velocity);

    // ========================================================================
    // PID TUNING
    // ========================================================================

    /**
     * @brief Set position PID gains
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPositionPID(float kp, float ki, float kd);

    /**
     * @brief Set velocity PID gains
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setVelocityPID(float kp, float ki, float kd);

    // ========================================================================
    // CONTROL UPDATE
    // ========================================================================

    /**
     * @brief Update motor control (call from scheduler @ 200Hz)
     *
     * Performs one iteration of:
     * - Velocity estimation update
     * - PID computation (position/velocity cascade)
     * - PWM output generation
     */
    void update();

    // ========================================================================
    // STATE QUERIES
    // ========================================================================

    /**
     * @brief Get current encoder position
     *
     * @return Position in encoder ticks
     */
    int32_t getPosition() const;

    /**
     * @brief Get current velocity estimate
     *
     * @return Velocity in ticks/second
     */
    float getVelocity() const;

    /**
     * @brief Get target position
     *
     * @return Target position in ticks
     */
    int32_t getTargetPosition() const { return targetPosition_; }

    /**
     * @brief Get target velocity
     *
     * @return Target velocity in ticks/second
     */
    float getTargetVelocity() const { return targetVelocity_; }

    /**
     * @brief Get current PWM output
     *
     * @return PWM value (-255 to +255, negative = reverse)
     */
    int16_t getPWMOutput() const { return pwmOutput_; }

    /**
     * @brief Get motor ID
     *
     * @return Motor identifier (0-3)
     */
    uint8_t getMotorId() const { return motorId_; }

private:
    // Hardware
    uint8_t motorId_;                       // Motor identifier (0-3)
    uint8_t pinEN_;                         // PWM pin (speed)
    uint8_t pinIN1_;                        // Direction pin 1
    uint8_t pinIN2_;                        // Direction pin 2

    // Control state
    DCMotorMode mode_;                      // Current control mode
    int32_t targetPosition_;                // Target position (ticks)
    float targetVelocity_;                  // Target velocity (ticks/sec)
    int16_t pwmOutput_;                     // Current PWM output (-255 to +255)

    // PID controllers
    PIDController positionPID_;             // Position loop (outer)
    PIDController velocityPID_;             // Velocity loop (inner)

    // Sensor interfaces
    IEncoderCounter *encoder_;              // Encoder counter instance
    IVelocityEstimator *velocityEst_;       // Velocity estimator instance

    // Timing
    uint32_t lastUpdateUs_;                 // Last update timestamp (micros())

    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    /**
     * @brief Set motor PWM and direction
     *
     * @param pwm PWM value (-255 to +255, negative = reverse)
     */
    void setPWM(int16_t pwm);
};

#endif // DCMOTOR_H
