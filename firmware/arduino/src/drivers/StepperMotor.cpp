/**
 * @file StepperMotor.cpp
 * @brief Implementation of stepper motor driver with acceleration
 */

#include "StepperMotor.h"
#include "../config.h"

// Timer frequency for step generation (from config)
#define TIMER_FREQ_HZ   STEPPER_TIMER_FREQ_HZ  // 10000 Hz

// ============================================================================
// CONSTRUCTOR AND INITIALIZATION
// ============================================================================

StepperMotor::StepperMotor()
    : stepperId_(0)
    , pinStep_(0)
    , pinDir_(0)
    , pinEnable_(0)
    , pinLimit_(0)
    , limitActiveState_(LOW)
    , hasLimit_(false)
    , enabled_(false)
    , maxVelocity_(1000)
    , acceleration_(500)
    , state_(STEPPER_IDLE)
    , currentPosition_(0)
    , targetPosition_(0)
    , stepsRemaining_(0)
    , direction_(1)
    , stepInterval_(0)
    , stepCounter_(0)
    , minInterval_(0)
    , accelSteps_(0)
    , decelSteps_(0)
    , cruiseSteps_(0)
    , stepCount_(0)
    , currentVelocity_(0.0f)
{
}

void StepperMotor::init(uint8_t stepperId) {
    stepperId_ = stepperId;
    state_ = STEPPER_IDLE;
    currentPosition_ = 0;
    targetPosition_ = 0;
    stepsRemaining_ = 0;

    // Calculate minimum interval at max velocity
    minInterval_ = TIMER_FREQ_HZ / maxVelocity_;
    if (minInterval_ < 1) minInterval_ = 1;
}

void StepperMotor::setPins(uint8_t pinStep, uint8_t pinDir, uint8_t pinEnable) {
    pinStep_ = pinStep;
    pinDir_ = pinDir;
    pinEnable_ = pinEnable;

    // Configure pins as outputs
    pinMode(pinStep_, OUTPUT);
    pinMode(pinDir_, OUTPUT);
    pinMode(pinEnable_, OUTPUT);

    // Initialize to disabled state (enable is typically active LOW)
    digitalWrite(pinStep_, LOW);
    digitalWrite(pinDir_, LOW);
    digitalWrite(pinEnable_, HIGH);  // Disabled (active LOW)
}

void StepperMotor::setLimitPin(uint8_t pinLimit, uint8_t activeState) {
    pinLimit_ = pinLimit;
    limitActiveState_ = activeState;
    hasLimit_ = true;

    // Configure as input with pullup if active LOW
    if (activeState == LOW) {
        pinMode(pinLimit_, INPUT_PULLUP);
    } else {
        pinMode(pinLimit_, INPUT);
    }
}

void StepperMotor::enable() {
    enabled_ = true;
    digitalWrite(pinEnable_, LOW);  // Active LOW

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.println(F("] Enabled"));
#endif
}

void StepperMotor::disable() {
    enabled_ = false;
    state_ = STEPPER_IDLE;
    digitalWrite(pinEnable_, HIGH);  // Disabled (active LOW)

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.println(F("] Disabled"));
#endif
}

// ============================================================================
// MOTION PARAMETERS
// ============================================================================

void StepperMotor::setMaxVelocity(uint16_t stepsPerSec) {
    maxVelocity_ = stepsPerSec;

    // Clamp to hardware limit
    if (maxVelocity_ > STEPPER_MAX_RATE_SPS) {
        maxVelocity_ = STEPPER_MAX_RATE_SPS;
    }

    // Calculate minimum interval at max velocity
    minInterval_ = TIMER_FREQ_HZ / maxVelocity_;
    if (minInterval_ < 1) minInterval_ = 1;
}

void StepperMotor::setAcceleration(uint16_t stepsPerSecSq) {
    acceleration_ = stepsPerSecSq;
    if (acceleration_ < 1) acceleration_ = 1;
}

// ============================================================================
// MOTION COMMANDS
// ============================================================================

void StepperMotor::moveSteps(int32_t steps) {
    if (steps == 0) return;
    if (!enabled_) return;

    // Set direction
    direction_ = (steps > 0) ? 1 : -1;
    digitalWrite(pinDir_, (direction_ > 0) ? HIGH : LOW);

    // Calculate target position
    targetPosition_ = currentPosition_ + steps;
    stepsRemaining_ = abs(steps);

    // Calculate motion profile
    calculateProfile(stepsRemaining_);

    // Start with initial step interval (low velocity for acceleration start)
    // Initial interval based on acceleration: v² = 2*a*s, at s=1: v = sqrt(2*a)
    float initialVelocity = sqrt(2.0f * acceleration_);
    if (initialVelocity < 100.0f) initialVelocity = 100.0f;  // Minimum start velocity
    stepInterval_ = TIMER_FREQ_HZ / initialVelocity;
    if (stepInterval_ < minInterval_) stepInterval_ = minInterval_;

    currentVelocity_ = initialVelocity;
    stepCounter_ = stepInterval_;
    stepCount_ = 0;

    state_ = STEPPER_ACCEL;

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.print(F("] Moving "));
    DEBUG_SERIAL.print(steps);
    DEBUG_SERIAL.print(F(" steps (accel="));
    DEBUG_SERIAL.print(accelSteps_);
    DEBUG_SERIAL.print(F(", cruise="));
    DEBUG_SERIAL.print(cruiseSteps_);
    DEBUG_SERIAL.print(F(", decel="));
    DEBUG_SERIAL.print(decelSteps_);
    DEBUG_SERIAL.println(F(")"));
#endif
}

void StepperMotor::moveToPosition(int32_t position) {
    int32_t steps = position - currentPosition_;
    moveSteps(steps);
}

void StepperMotor::home(int8_t direction) {
    if (!enabled_) return;
    if (!hasLimit_) return;  // No limit switch configured

    // Set direction
    direction_ = (direction > 0) ? 1 : -1;
    digitalWrite(pinDir_, (direction_ > 0) ? HIGH : LOW);

    // Set up slow constant velocity homing
    stepsRemaining_ = INT32_MAX;  // Move until limit
    stepInterval_ = TIMER_FREQ_HZ / (maxVelocity_ / 4);  // Slow speed for homing
    if (stepInterval_ < minInterval_) stepInterval_ = minInterval_;

    stepCounter_ = stepInterval_;
    state_ = STEPPER_HOMING;

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.println(F("] Homing started"));
#endif
}

void StepperMotor::stop() {
    // Immediate stop - no deceleration
    state_ = STEPPER_IDLE;
    stepsRemaining_ = 0;

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.println(F("] Emergency STOP"));
#endif
}

void StepperMotor::smoothStop() {
    if (state_ == STEPPER_IDLE) return;

    // Calculate deceleration distance from current velocity
    // s = v² / (2*a)
    uint32_t decelDistance = (uint32_t)(currentVelocity_ * currentVelocity_) / (2 * acceleration_);

    if (decelDistance < 1) {
        // Already slow enough, just stop
        state_ = STEPPER_IDLE;
        stepsRemaining_ = 0;
    } else {
        // Update steps remaining to just the deceleration distance
        stepsRemaining_ = decelDistance;
        decelSteps_ = decelDistance;
        cruiseSteps_ = 0;
        accelSteps_ = 0;
        state_ = STEPPER_DECEL;
    }
}

void StepperMotor::setPosition(int32_t position) {
    currentPosition_ = position;
}

// ============================================================================
// MOTION PROFILE CALCULATION
// ============================================================================

void StepperMotor::calculateProfile(int32_t totalSteps) {
    // Calculate acceleration distance to reach max velocity
    // Using: v² = v₀² + 2*a*s  →  s = (v² - v₀²) / (2*a)
    // Assuming v₀ ≈ 0:  s_accel = v_max² / (2*a)

    float accelDist = (float)maxVelocity_ * maxVelocity_ / (2.0f * acceleration_);
    accelSteps_ = (uint32_t)accelDist;

    // Deceleration distance is same as acceleration distance
    decelSteps_ = accelSteps_;

    // Check if we can reach max velocity (triangular profile)
    if (accelSteps_ + decelSteps_ >= (uint32_t)totalSteps) {
        // Triangular profile - no cruise phase
        // Split distance equally between accel and decel
        accelSteps_ = totalSteps / 2;
        decelSteps_ = totalSteps - accelSteps_;
        cruiseSteps_ = 0;
    } else {
        // Trapezoidal profile with cruise phase
        cruiseSteps_ = totalSteps - accelSteps_ - decelSteps_;
    }
}

// ============================================================================
// TIMER CALLBACK (called from Timer3 ISR @ 10kHz)
// ============================================================================

void StepperMotor::timerCallback() {
    if (state_ == STEPPER_IDLE) {
        return;  // Not moving
    }

    // Decrement step counter
    if (stepCounter_ > 0) {
        stepCounter_--;
        return;  // Not time for a step yet
    }

    // Time for a step - reload counter
    stepCounter_ = stepInterval_;

    // Check limit switch during homing
    if (state_ == STEPPER_HOMING) {
        if (isLimitTriggered()) {
            // Limit reached - stop and set position to zero
            state_ = STEPPER_IDLE;
            currentPosition_ = 0;
            stepsRemaining_ = 0;

#ifdef DEBUG_STEPPER
            DEBUG_SERIAL.print(F("[Stepper "));
            DEBUG_SERIAL.print(stepperId_);
            DEBUG_SERIAL.println(F("] Home position found"));
#endif
            return;
        }
    }

    // Generate step pulse
    digitalWrite(pinStep_, HIGH);
    // Small delay for step pulse width (most drivers need >1µs)
    delayMicroseconds(2);
    digitalWrite(pinStep_, LOW);

    // Update position
    currentPosition_ += direction_;
    stepsRemaining_--;
    stepCount_++;

    // Check if move is complete
    if (stepsRemaining_ <= 0) {
        state_ = STEPPER_IDLE;

#ifdef DEBUG_STEPPER
        DEBUG_SERIAL.print(F("[Stepper "));
        DEBUG_SERIAL.print(stepperId_);
        DEBUG_SERIAL.print(F("] Move complete at position "));
        DEBUG_SERIAL.println(currentPosition_);
#endif
        return;
    }

    // Update step interval based on current phase
    switch (state_) {
        case STEPPER_ACCEL:
            updateAccelInterval();
            // Check if acceleration phase complete
            if (stepCount_ >= accelSteps_) {
                if (cruiseSteps_ > 0) {
                    state_ = STEPPER_CRUISE;
                } else {
                    state_ = STEPPER_DECEL;
                }
            }
            break;

        case STEPPER_CRUISE:
            // Constant velocity - no interval change
            // Check if cruise phase complete
            if (stepCount_ >= accelSteps_ + cruiseSteps_) {
                state_ = STEPPER_DECEL;
            }
            break;

        case STEPPER_DECEL:
            updateDecelInterval();
            break;

        case STEPPER_HOMING:
            // Constant slow velocity during homing
            break;

        default:
            break;
    }
}

// ============================================================================
// ACCELERATION HELPERS
// ============================================================================

void StepperMotor::updateAccelInterval() {
    // Increase velocity: v_new = sqrt(v² + 2*a*Δs)
    // Since Δs = 1 step: v_new = sqrt(v² + 2*a)
    float vSquared = currentVelocity_ * currentVelocity_;
    currentVelocity_ = sqrt(vSquared + 2.0f * acceleration_);

    // Clamp to max velocity
    if (currentVelocity_ > maxVelocity_) {
        currentVelocity_ = maxVelocity_;
    }

    // Update step interval
    stepInterval_ = TIMER_FREQ_HZ / currentVelocity_;
    if (stepInterval_ < minInterval_) stepInterval_ = minInterval_;
}

void StepperMotor::updateDecelInterval() {
    // Decrease velocity: v_new = sqrt(v² - 2*a*Δs)
    // Since Δs = 1 step: v_new = sqrt(v² - 2*a)
    float vSquared = currentVelocity_ * currentVelocity_;
    float newVSquared = vSquared - 2.0f * acceleration_;

    if (newVSquared < 10000.0f) {  // Min velocity ~100 steps/sec
        currentVelocity_ = 100.0f;
    } else {
        currentVelocity_ = sqrt(newVSquared);
    }

    // Update step interval
    stepInterval_ = TIMER_FREQ_HZ / currentVelocity_;

    // Clamp to reasonable maximum interval
    if (stepInterval_ > TIMER_FREQ_HZ / 10) {  // Min 10 steps/sec
        stepInterval_ = TIMER_FREQ_HZ / 10;
    }
}

bool StepperMotor::isLimitTriggered() const {
    if (!hasLimit_) return false;
    return digitalRead(pinLimit_) == limitActiveState_;
}
