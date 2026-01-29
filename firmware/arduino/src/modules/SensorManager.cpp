/**
 * @file SensorManager.cpp
 * @brief Implementation of sensor aggregation module
 */

#include "SensorManager.h"
#include "../pins.h"

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

IMUDriver SensorManager::imu_;

float SensorManager::batteryVoltage_ = 0.0f;
float SensorManager::rail5VVoltage_ = 0.0f;
float SensorManager::servoVoltage_ = 0.0f;

float SensorManager::accel_[3] = {0.0f, 0.0f, 0.0f};
float SensorManager::gyro_[3] = {0.0f, 0.0f, 0.0f};
float SensorManager::mag_[3] = {0.0f, 0.0f, 0.0f};
float SensorManager::imuTemp_ = 0.0f;
bool SensorManager::imuDataValid_ = false;

uint32_t SensorManager::updateCount_ = 0;
uint32_t SensorManager::lastUpdateTime_ = 0;

bool SensorManager::initialized_ = false;
bool SensorManager::imuInitialized_ = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

void SensorManager::init() {
    if (initialized_) return;

    // Initialize ADC reference voltage (default is AVCC = 5V)
    analogReference(DEFAULT);

    // Initialize IMU if enabled
#if IMU_ENABLED
    imuInitialized_ = imu_.init(IMU_I2C_ADDRESS);
    if (imuInitialized_) {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.println(F("[SensorManager] IMU initialized"));
#endif
    } else {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.println(F("[SensorManager] WARNING: IMU not detected"));
#endif
    }
#endif

    // Read initial voltage values
    updateVoltages();

    initialized_ = true;

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.println(F("[SensorManager] Initialized"));
    DEBUG_SERIAL.print(F("  - Battery: "));
    DEBUG_SERIAL.print(batteryVoltage_);
    DEBUG_SERIAL.println(F("V"));
    DEBUG_SERIAL.print(F("  - 5V Rail: "));
    DEBUG_SERIAL.print(rail5VVoltage_);
    DEBUG_SERIAL.println(F("V"));
    DEBUG_SERIAL.print(F("  - Servo Rail: "));
    DEBUG_SERIAL.print(servoVoltage_);
    DEBUG_SERIAL.println(F("V"));
#endif
}

// ============================================================================
// UPDATE
// ============================================================================

void SensorManager::update() {
    if (!initialized_) return;

    lastUpdateTime_ = millis();
    updateCount_++;

    // Update voltage readings
    updateVoltages();

    // Update IMU readings
#if IMU_ENABLED
    if (imuInitialized_) {
        updateIMU();
    }
#endif
}

// ============================================================================
// VOLTAGE MONITORING
// ============================================================================

float SensorManager::getBatteryVoltage() {
    return batteryVoltage_;
}

float SensorManager::get5VRailVoltage() {
    return rail5VVoltage_;
}

float SensorManager::getServoVoltage() {
    return servoVoltage_;
}

bool SensorManager::isBatteryLow(float threshold) {
    return (batteryVoltage_ > 0.0f && batteryVoltage_ < threshold);
}

// ============================================================================
// IMU DATA
// ============================================================================

bool SensorManager::getAcceleration(float accel[3]) {
    if (!imuDataValid_) return false;

    accel[0] = accel_[0];
    accel[1] = accel_[1];
    accel[2] = accel_[2];

    return true;
}

bool SensorManager::getAngularVelocity(float gyro[3]) {
    if (!imuDataValid_) return false;

    gyro[0] = gyro_[0];
    gyro[1] = gyro_[1];
    gyro[2] = gyro_[2];

    return true;
}

bool SensorManager::getMagneticField(float mag[3]) {
    if (!imuDataValid_) return false;

    mag[0] = mag_[0];
    mag[1] = mag_[1];
    mag[2] = mag_[2];

    return true;
}

float SensorManager::getIMUTemperature() {
    return imuTemp_;
}

bool SensorManager::isIMUAvailable() {
    return imuInitialized_ && imu_.isConnected();
}

// ============================================================================
// STATISTICS
// ============================================================================

uint32_t SensorManager::getUpdateCount() {
    return updateCount_;
}

uint32_t SensorManager::getLastUpdateTime() {
    return lastUpdateTime_;
}

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

void SensorManager::updateVoltages() {
    // Read battery voltage (1:6 divider, 0-24V range)
    uint16_t adcVBat = readADCAverage(PIN_VBAT_SENSE);
    batteryVoltage_ = adcToVoltage(adcVBat, 1.0f / 6.0f);

    // Read 5V rail voltage (1:2 divider, 0-10V range)
    uint16_t adcV5 = readADCAverage(PIN_V5_SENSE);
    rail5VVoltage_ = adcToVoltage(adcV5, 1.0f / 2.0f);

    // Read servo rail voltage (1:3 divider, 0-15V range)
    uint16_t adcVServo = readADCAverage(PIN_VSERVO_SENSE);
    servoVoltage_ = adcToVoltage(adcVServo, 1.0f / 3.0f);
}

void SensorManager::updateIMU() {
    // Read accelerometer and gyroscope
    imuDataValid_ = imu_.readAccelGyro(accel_, gyro_);

    // Read magnetometer if enabled
    if (imu_.isMagnetometerEnabled()) {
        imu_.readMagnetometer(mag_);
    }

    // Read temperature
    imuTemp_ = imu_.readTemperature();
}

uint16_t SensorManager::readADCAverage(uint8_t pin, uint8_t numSamples) {
    uint32_t sum = 0;

    for (uint8_t i = 0; i < numSamples; i++) {
        sum += analogRead(pin);
        delayMicroseconds(100);  // Small delay between samples
    }

    return (uint16_t)(sum / numSamples);
}

float SensorManager::adcToVoltage(uint16_t adcValue, float dividerRatio) {
    // ADC reference voltage (5V with DEFAULT setting)
    const float vRef = 5.0f;

    // Convert ADC value to voltage at ADC input
    float vADC = (adcValue / 1023.0f) * vRef;

    // Calculate actual voltage before divider
    float vActual = vADC / dividerRatio;

    return vActual;
}
