/**
 * @file StepperManager.cpp
 * @brief Implementation of Timer3-based stepper motor coordination
 */

#include "StepperManager.h"
#include "../pins.h"

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

StepperMotor StepperManager::steppers_[NUM_STEPPERS];
bool StepperManager::initialized_ = false;

// ============================================================================
// TIMER3 ISR
// ============================================================================

/**
 * @brief Timer3 Compare Match A Interrupt
 *
 * Fires at 10kHz (100µs period) for stepper pulse generation.
 */
ISR(TIMER3_COMPA_vect) {
    StepperManager::timerISR();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void StepperManager::init() {
    if (initialized_) return;

    // Initialize stepper instances
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers_[i].init(i);
    }

    // Configure stepper pins based on hardware configuration
#if STEPPER_1_ENABLED
    steppers_[0].setPins(PIN_ST1_STEP, PIN_ST1_DIR, PIN_ST1_EN);
#if defined(PIN_ST1_LIMIT)
    steppers_[0].setLimitPin(PIN_ST1_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
#endif
#endif

#if STEPPER_2_ENABLED
    steppers_[1].setPins(PIN_ST2_STEP, PIN_ST2_DIR, PIN_ST2_EN);
#if defined(PIN_ST2_LIMIT)
    steppers_[1].setLimitPin(PIN_ST2_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
#endif
#endif

#if STEPPER_3_ENABLED
    steppers_[2].setPins(PIN_ST3_STEP, PIN_ST3_DIR, PIN_ST3_EN);
#if defined(PIN_ST3_LIMIT)
    steppers_[2].setLimitPin(PIN_ST3_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
#endif
#endif

#if STEPPER_4_ENABLED
    steppers_[3].setPins(PIN_ST4_STEP, PIN_ST4_DIR, PIN_ST4_EN);
#if defined(PIN_ST4_LIMIT)
    steppers_[3].setLimitPin(PIN_ST4_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
#endif
#endif

    // ========================================================================
    // Configure Timer3 for 10kHz interrupt (100µs period)
    // ========================================================================

    // Disable interrupts during configuration
    noInterrupts();

    // Reset Timer3 control registers
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0;

    // Calculate OCR3A for desired frequency:
    // OCR = (F_CPU / (prescaler * freq)) - 1
    // For 10kHz with prescaler=8: OCR = (16000000 / (8 * 10000)) - 1 = 199
    OCR3A = (F_CPU / (8UL * STEPPER_TIMER_FREQ_HZ)) - 1;

    // Configure for CTC mode (Clear Timer on Compare Match)
    // WGM32 = 1 (CTC mode with OCR3A as TOP)
    TCCR3B |= (1 << WGM32);

    // Set prescaler to 8
    // CS31 = 1 (prescaler = 8)
    TCCR3B |= (1 << CS31);

    // Enable Timer3 Compare Match A interrupt
    TIMSK3 |= (1 << OCIE3A);

    // Re-enable interrupts
    interrupts();

    initialized_ = true;

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.println(F("[StepperManager] Timer3 initialized @ 10kHz"));
    DEBUG_SERIAL.print(F("  - OCR3A = "));
    DEBUG_SERIAL.println(OCR3A);
#endif
}

// ============================================================================
// STEPPER ACCESS
// ============================================================================

StepperMotor* StepperManager::getStepper(uint8_t stepperId) {
    if (stepperId >= NUM_STEPPERS) {
        return nullptr;
    }
    return &steppers_[stepperId];
}

// ============================================================================
// TIMER ISR HANDLER
// ============================================================================

void StepperManager::timerISR() {
#ifdef DEBUG_PINS_ENABLED
    digitalWrite(DEBUG_PIN_STEPPER_ISR, HIGH);
#endif

    // Call timerCallback on each enabled stepper
#if STEPPER_1_ENABLED
    steppers_[0].timerCallback();
#endif

#if STEPPER_2_ENABLED
    steppers_[1].timerCallback();
#endif

#if STEPPER_3_ENABLED
    steppers_[2].timerCallback();
#endif

#if STEPPER_4_ENABLED
    steppers_[3].timerCallback();
#endif

#ifdef DEBUG_PINS_ENABLED
    digitalWrite(DEBUG_PIN_STEPPER_ISR, LOW);
#endif
}

// ============================================================================
// CONTROL FUNCTIONS
// ============================================================================

void StepperManager::emergencyStopAll() {
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers_[i].stop();
    }

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.println(F("[StepperManager] Emergency stop ALL"));
#endif
}

void StepperManager::disableAll() {
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers_[i].disable();
    }

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.println(F("[StepperManager] Disabled ALL"));
#endif
}

bool StepperManager::anyMoving() {
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        if (steppers_[i].isMoving()) {
            return true;
        }
    }
    return false;
}
