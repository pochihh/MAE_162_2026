/**
 * @file StepperManager.h
 * @brief Timer3-based stepper motor coordination
 *
 * This module manages Timer3 for precise step pulse generation and
 * coordinates multiple stepper motors operating simultaneously.
 *
 * Timer3 Configuration:
 * - 10kHz interrupt rate (100µs period)
 * - CTC mode with OCR3A for precise timing
 * - ISR calls timerCallback() on each enabled stepper
 *
 * Timing Budget:
 * - 100µs total period
 * - ISR overhead: ~5µs
 * - Per-stepper callback: ~10µs
 * - With 4 steppers: 5 + 4*10 = 45µs (45% utilization)
 *
 * Usage:
 *   StepperManager::init();                          // Configure Timer3
 *   StepperMotor* motor = StepperManager::getStepper(0);  // Get stepper 0
 *   motor->setPins(STEP, DIR, EN);
 *   motor->setMaxVelocity(1000);
 *   motor->enable();
 *   motor->moveSteps(200);
 *
 *   // Timer3 ISR is automatically configured by init()
 *
 * Note: A4988 Vref setup:
 *   Vref = 8 * Imax * Rset (0.05Ω typical)
 *   For 1.0A max current, Vref = 0.4V
 */

#ifndef STEPPERMANAGER_H
#define STEPPERMANAGER_H

#include <Arduino.h>
#include <stdint.h>
#include "../drivers/StepperMotor.h"
#include "../config.h"

// ============================================================================
// STEPPER MANAGER CLASS (Static)
// ============================================================================

/**
 * @brief Manages Timer3 and coordinates all stepper motors
 *
 * Static class providing:
 * - Timer3 configuration for 10kHz interrupt
 * - Stepper motor instances
 * - ISR entry point for step generation
 *
 * All methods are static - no instantiation needed.
 */
class StepperManager {
public:
    /**
     * @brief Initialize Timer3 and stepper instances
     *
     * Configures Timer3 for 10kHz CTC mode interrupt.
     * Creates stepper motor instances (not enabled by default).
     *
     * Must be called once in setup() before using steppers.
     */
    static void init();

    /**
     * @brief Get stepper motor instance by ID
     *
     * @param stepperId Stepper index (0 to NUM_STEPPERS-1)
     * @return Pointer to StepperMotor instance, or nullptr if invalid ID
     */
    static StepperMotor* getStepper(uint8_t stepperId);

    /**
     * @brief Timer3 ISR handler
     *
     * Called from Timer3 compare match ISR at 10kHz.
     * Iterates through all enabled steppers and calls timerCallback().
     *
     * CRITICAL: Must complete in <50µs for reliable operation.
     */
    static void timerISR();

    /**
     * @brief Emergency stop all steppers
     *
     * Immediately stops all stepper motors without deceleration.
     */
    static void emergencyStopAll();

    /**
     * @brief Disable all steppers
     *
     * Disables all stepper drivers (motors can freewheel).
     */
    static void disableAll();

    /**
     * @brief Check if any stepper is moving
     *
     * @return True if at least one stepper is in motion
     */
    static bool anyMoving();

private:
    // Stepper motor instances
    static StepperMotor steppers_[NUM_STEPPERS];

    // Initialization flag
    static bool initialized_;
};

// ============================================================================
// TIMER3 ISR DECLARATION
// ============================================================================

// The actual ISR is defined in the .cpp file
// ISR(TIMER3_COMPA_vect) { StepperManager::timerISR(); }

#endif // STEPPERMANAGER_H
