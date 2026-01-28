/**
 * @file ServoController.h
 * @brief Wrapper for PCA9685 I2C PWM driver for servo control
 *
 * This module provides a simplified interface to control servos via
 * the PCA9685 16-channel PWM driver IC over I2C.
 *
 * Features:
 * - 16 servo channels (0-15)
 * - Pulse width control in microseconds (500-2500µs typical)
 * - Angle control in degrees (0-180° typical)
 * - Output enable/disable for power management
 * - Bulk updates for synchronized motion
 *
 * Hardware Connection:
 * - SDA/SCL: I2C bus (shared with other I2C devices)
 * - VCC: Logic power (3.3V or 5V)
 * - V+: Servo power (separate supply recommended)
 * - OE: Output enable (active LOW, optional for power control)
 *
 * Servo PWM Timing:
 * - Standard servos: 50Hz (20ms period)
 * - Pulse width: 1000-2000µs for 0-180° (varies by servo)
 * - PCA9685: 4096 steps per period (12-bit resolution)
 *
 * Usage:
 *   ServoController::init();                    // Initialize PCA9685
 *   ServoController::enable();                  // Enable outputs
 *   ServoController::setPositionUs(0, 1500);    // Servo 0 to center (1500µs)
 *   ServoController::setPositionDeg(1, 90);     // Servo 1 to 90°
 */

#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <Arduino.h>
#include <stdint.h>
#include "../../lib/PCA9685.h"
#include "../config.h"

// ============================================================================
// SERVO CONTROLLER CLASS (Static)
// ============================================================================

/**
 * @brief PCA9685-based servo controller
 *
 * Static class providing:
 * - PCA9685 initialization and configuration
 * - Pulse width and angle-based positioning
 * - Output enable control
 * - Bulk channel updates
 *
 * All methods are static - no instantiation needed.
 */
class ServoController {
public:
    /**
     * @brief Initialize PCA9685 and configure for servo control
     *
     * Configures PCA9685 for 50Hz PWM frequency (standard servos).
     * Sets output driver mode to totem-pole for external driver compatibility.
     *
     * Must be called once in setup() before using servos.
     */
    static void init();

    /**
     * @brief Enable servo outputs
     *
     * Pulls OE pin LOW to enable PWM outputs (if OE pin configured).
     * Outputs remain at their last commanded position.
     */
    static void enable();

    /**
     * @brief Disable servo outputs
     *
     * Pulls OE pin HIGH to disable PWM outputs (if OE pin configured).
     * Servos will go limp (no holding torque).
     */
    static void disable();

    /**
     * @brief Check if outputs are enabled
     *
     * @return True if outputs are enabled
     */
    static bool isEnabled();

    // ========================================================================
    // POSITION CONTROL
    // ========================================================================

    /**
     * @brief Set servo position by pulse width
     *
     * @param channel Servo channel (0-15)
     * @param pulseWidthUs Pulse width in microseconds (typically 500-2500µs)
     */
    static void setPositionUs(uint8_t channel, uint16_t pulseWidthUs);

    /**
     * @brief Set servo position by angle
     *
     * Maps angle to pulse width using configured min/max values.
     * Default mapping: 0° = 500µs, 180° = 2500µs
     *
     * @param channel Servo channel (0-15)
     * @param degrees Angle in degrees (typically 0-180°)
     */
    static void setPositionDeg(uint8_t channel, float degrees);

    /**
     * @brief Set multiple servo positions at once
     *
     * Useful for synchronized motion of multiple servos.
     *
     * @param startChannel First channel to update
     * @param numChannels Number of channels to update
     * @param pulseWidthsUs Array of pulse widths in microseconds
     */
    static void setMultiplePositionsUs(uint8_t startChannel, uint8_t numChannels,
                                        const uint16_t *pulseWidthsUs);

    /**
     * @brief Turn off PWM for a channel
     *
     * Sets channel to fully off (0% duty cycle).
     * Servo will go limp on that channel.
     *
     * @param channel Servo channel (0-15)
     */
    static void setChannelOff(uint8_t channel);

    /**
     * @brief Turn off all servo channels
     *
     * Sets all channels to fully off (0% duty cycle).
     */
    static void setAllOff();

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    /**
     * @brief Set pulse width range for angle mapping
     *
     * Configures the pulse width values that correspond to 0° and 180°.
     * Affects all subsequent setPositionDeg() calls.
     *
     * @param minUs Pulse width for 0° (default: 500µs)
     * @param maxUs Pulse width for 180° (default: 2500µs)
     */
    static void setPulseWidthRange(uint16_t minUs, uint16_t maxUs);

    /**
     * @brief Set angle range
     *
     * Configures the angle values that correspond to min/max pulse width.
     * Default: 0° to 180°
     *
     * @param minDeg Minimum angle (default: 0°)
     * @param maxDeg Maximum angle (default: 180°)
     */
    static void setAngleRange(float minDeg, float maxDeg);

    /**
     * @brief Get current pulse width for a channel
     *
     * @param channel Servo channel (0-15)
     * @return Last commanded pulse width in microseconds
     */
    static uint16_t getPositionUs(uint8_t channel);

private:
    // PCA9685 driver instance
    static PCA9685 pca9685_;

    // Configuration
    static uint16_t minPulseUs_;    // Pulse width for min angle
    static uint16_t maxPulseUs_;    // Pulse width for max angle
    static float minAngleDeg_;      // Minimum angle
    static float maxAngleDeg_;      // Maximum angle

    // State
    static bool enabled_;
    static bool initialized_;
    static uint16_t channelPulseUs_[16];  // Last commanded pulse widths

    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    /**
     * @brief Convert pulse width to PCA9685 PWM value
     *
     * PCA9685 uses 12-bit resolution (0-4095) for one PWM period (20ms @ 50Hz).
     * Formula: pwmValue = (pulseWidthUs / 20000µs) * 4096
     *
     * @param pulseWidthUs Pulse width in microseconds
     * @return PWM value (0-4095)
     */
    static uint16_t pulseToPWM(uint16_t pulseWidthUs);

    /**
     * @brief Convert angle to pulse width
     *
     * Linear interpolation from angle to pulse width.
     *
     * @param degrees Angle in degrees
     * @return Pulse width in microseconds
     */
    static uint16_t angleToPulse(float degrees);
};

#endif // SERVOCONTROLLER_H
