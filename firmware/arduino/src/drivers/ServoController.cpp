/**
 * @file ServoController.cpp
 * @brief Implementation of PCA9685-based servo controller
 */

#include "ServoController.h"
#include "../pins.h"
#include <Wire.h>

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

// PCA9685 at default I2C address (0x40)
PCA9685 ServoController::pca9685_(0x00);  // Address pins A5-A0 = 0

// Default pulse width range (500-2500µs covers most servos)
uint16_t ServoController::minPulseUs_ = 500;
uint16_t ServoController::maxPulseUs_ = 2500;

// Default angle range (0-180°)
float ServoController::minAngleDeg_ = 0.0f;
float ServoController::maxAngleDeg_ = 180.0f;

// State tracking
bool ServoController::enabled_ = false;
bool ServoController::initialized_ = false;
uint16_t ServoController::channelPulseUs_[16] = {0};

// ============================================================================
// INITIALIZATION
// ============================================================================

void ServoController::init() {
    if (initialized_) return;

    // Initialize I2C (Wire)
    Wire.begin();

    // Reset all PCA9685 devices on the bus
    pca9685_.resetDevices();

    // Initialize PCA9685
    // - TotemPole output driver mode (works with most servo setups)
    // - Normal output when OE enabled
    // - Low output when OE disabled
    pca9685_.init(
        PCA9685_OutputDriverMode_TotemPole,
        PCA9685_OutputEnabledMode_Normal,
        PCA9685_OutputDisabledMode_Low
    );

    // Set PWM frequency for servos (50Hz = 20ms period)
    pca9685_.setPWMFreqServo();

    // Configure OE pin if defined
#ifdef PIN_SERVO_OE
    pinMode(PIN_SERVO_OE, OUTPUT);
    digitalWrite(PIN_SERVO_OE, HIGH);  // Start with outputs disabled
#endif

    // Initialize all channels to off
    for (uint8_t i = 0; i < 16; i++) {
        pca9685_.setChannelOff(i);
        channelPulseUs_[i] = 0;
    }

    initialized_ = true;
    enabled_ = false;

#ifdef DEBUG_SERVO
    DEBUG_SERIAL.println(F("[ServoController] Initialized"));
    DEBUG_SERIAL.print(F("  - I2C Address: 0x"));
    DEBUG_SERIAL.println(pca9685_.getI2CAddress(), HEX);
    DEBUG_SERIAL.print(F("  - Pulse range: "));
    DEBUG_SERIAL.print(minPulseUs_);
    DEBUG_SERIAL.print(F("-"));
    DEBUG_SERIAL.print(maxPulseUs_);
    DEBUG_SERIAL.println(F("µs"));
#endif
}

// ============================================================================
// ENABLE/DISABLE
// ============================================================================

void ServoController::enable() {
#ifdef PIN_SERVO_OE
    digitalWrite(PIN_SERVO_OE, LOW);  // OE is active LOW
#endif
    enabled_ = true;

#ifdef DEBUG_SERVO
    DEBUG_SERIAL.println(F("[ServoController] Outputs enabled"));
#endif
}

void ServoController::disable() {
#ifdef PIN_SERVO_OE
    digitalWrite(PIN_SERVO_OE, HIGH);  // OE inactive
#endif
    enabled_ = false;

#ifdef DEBUG_SERVO
    DEBUG_SERIAL.println(F("[ServoController] Outputs disabled"));
#endif
}

bool ServoController::isEnabled() {
    return enabled_;
}

// ============================================================================
// POSITION CONTROL
// ============================================================================

void ServoController::setPositionUs(uint8_t channel, uint16_t pulseWidthUs) {
    if (channel >= 16) return;
    if (!initialized_) return;

    // Clamp pulse width to reasonable range
    if (pulseWidthUs < 100) pulseWidthUs = 100;
    if (pulseWidthUs > 3000) pulseWidthUs = 3000;

    // Convert to PCA9685 PWM value
    uint16_t pwmValue = pulseToPWM(pulseWidthUs);

    // Set the channel
    pca9685_.setChannelPWM(channel, pwmValue);

    // Track the pulse width
    channelPulseUs_[channel] = pulseWidthUs;

#ifdef DEBUG_SERVO
    DEBUG_SERIAL.print(F("[Servo "));
    DEBUG_SERIAL.print(channel);
    DEBUG_SERIAL.print(F("] "));
    DEBUG_SERIAL.print(pulseWidthUs);
    DEBUG_SERIAL.print(F("µs (PWM="));
    DEBUG_SERIAL.print(pwmValue);
    DEBUG_SERIAL.println(F(")"));
#endif
}

void ServoController::setPositionDeg(uint8_t channel, float degrees) {
    // Convert angle to pulse width
    uint16_t pulseUs = angleToPulse(degrees);

    // Set the position
    setPositionUs(channel, pulseUs);
}

void ServoController::setMultiplePositionsUs(uint8_t startChannel, uint8_t numChannels,
                                              const uint16_t *pulseWidthsUs) {
    if (!initialized_) return;
    if (startChannel >= 16) return;
    if (startChannel + numChannels > 16) {
        numChannels = 16 - startChannel;
    }

    // Convert all pulse widths to PWM values
    uint16_t pwmValues[16];
    for (uint8_t i = 0; i < numChannels; i++) {
        uint16_t pulseUs = pulseWidthsUs[i];
        if (pulseUs < 100) pulseUs = 100;
        if (pulseUs > 3000) pulseUs = 3000;

        pwmValues[i] = pulseToPWM(pulseUs);
        channelPulseUs_[startChannel + i] = pulseUs;
    }

    // Set all channels at once
    pca9685_.setChannelsPWM(startChannel, numChannels, pwmValues);
}

void ServoController::setChannelOff(uint8_t channel) {
    if (channel >= 16) return;
    if (!initialized_) return;

    pca9685_.setChannelOff(channel);
    channelPulseUs_[channel] = 0;
}

void ServoController::setAllOff() {
    if (!initialized_) return;

    for (uint8_t i = 0; i < 16; i++) {
        pca9685_.setChannelOff(i);
        channelPulseUs_[i] = 0;
    }
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void ServoController::setPulseWidthRange(uint16_t minUs, uint16_t maxUs) {
    minPulseUs_ = minUs;
    maxPulseUs_ = maxUs;
}

void ServoController::setAngleRange(float minDeg, float maxDeg) {
    minAngleDeg_ = minDeg;
    maxAngleDeg_ = maxDeg;
}

uint16_t ServoController::getPositionUs(uint8_t channel) {
    if (channel >= 16) return 0;
    return channelPulseUs_[channel];
}

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

uint16_t ServoController::pulseToPWM(uint16_t pulseWidthUs) {
    // PCA9685 uses 12-bit resolution (0-4095) for one PWM period
    // At 50Hz, period = 20000µs
    // PWM value = (pulseWidth / period) * 4096
    //           = (pulseWidth * 4096) / 20000
    //           = pulseWidth * 0.2048
    //
    // Using integer math: (pulseWidth * 4096 + 10000) / 20000
    // Adding 10000 for rounding

    uint32_t pwm = ((uint32_t)pulseWidthUs * 4096UL + 10000UL) / 20000UL;

    // Clamp to valid range
    if (pwm > 4095) pwm = 4095;

    return (uint16_t)pwm;
}

uint16_t ServoController::angleToPulse(float degrees) {
    // Clamp angle to range
    if (degrees < minAngleDeg_) degrees = minAngleDeg_;
    if (degrees > maxAngleDeg_) degrees = maxAngleDeg_;

    // Linear interpolation
    // pulseUs = minPulse + (angle - minAngle) * (maxPulse - minPulse) / (maxAngle - minAngle)

    float angleRange = maxAngleDeg_ - minAngleDeg_;
    if (angleRange < 0.001f) angleRange = 0.001f;  // Avoid division by zero

    float ratio = (degrees - minAngleDeg_) / angleRange;
    float pulseUs = minPulseUs_ + ratio * (maxPulseUs_ - minPulseUs_);

    return (uint16_t)pulseUs;
}
