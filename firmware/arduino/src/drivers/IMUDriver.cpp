/**
 * @file IMUDriver.cpp
 * @brief Implementation of ICM-20948 IMU driver
 */

#include "IMUDriver.h"

#if IMU_ENABLED
  // Only include ICM_20948 library if IMU is enabled
  // Note: This requires the full ICM_20948 library with util/ subdirectory
  // Comment out the include below if library is not available
  // #include "../../lib/ICM_20948.h"
#endif

#include <Wire.h>

// ============================================================================
// CONSTRUCTOR
// ============================================================================

IMUDriver::IMUDriver()
    : initialized_(false)
    , i2cAddress_(0x68)
    , accelRange_(16)
    , gyroRange_(2000)
    , magEnabled_(false)
{
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool IMUDriver::init(uint8_t i2cAddress) {
    i2cAddress_ = i2cAddress;

#if IMU_ENABLED
    // Initialize I2C
    Wire.begin();

    // Initialize ICM-20948
    // Note: This requires full ICM_20948 library
    // bool success = imu_.begin(Wire, i2cAddress);
    bool success = false;  // Stub when library not available

    if (!success) {
#ifdef DEBUG_IMU
        DEBUG_SERIAL.println(F("[IMU] ICM-20948 library not available or init failed"));
#endif
        return false;
    }

    // Set default accelerometer range (±16g)
    setAccelRange(16);

    // Set default gyroscope range (±2000 dps)
    setGyroRange(2000);

    // Enable magnetometer
    setMagnetometerEnabled(true);

    initialized_ = true;

#ifdef DEBUG_IMU
    DEBUG_SERIAL.println(F("[IMU] ICM-20948 initialized"));
    DEBUG_SERIAL.print(F("  - I2C Address: 0x"));
    DEBUG_SERIAL.println(i2cAddress, HEX);
    DEBUG_SERIAL.print(F("  - Accel Range: ±"));
    DEBUG_SERIAL.print(accelRange_);
    DEBUG_SERIAL.println(F("g"));
    DEBUG_SERIAL.print(F("  - Gyro Range: ±"));
    DEBUG_SERIAL.print(gyroRange_);
    DEBUG_SERIAL.println(F("dps"));
#endif

    return true;
#else
    // IMU disabled in config
    return false;
#endif
}

bool IMUDriver::isConnected() {
#if IMU_ENABLED
    if (!initialized_) return false;
    // IMU library not available
    return false;
#else
    return false;
#endif
}

// ============================================================================
// DATA READING
// ============================================================================

bool IMUDriver::readAccelGyro(float accel[3], float gyro[3]) {
#if IMU_ENABLED
    // IMU library not available - return zeros
    accel[0] = accel[1] = accel[2] = 0.0f;
    gyro[0] = gyro[1] = gyro[2] = 0.0f;
    return false;
#else
    return false;
#endif
}

bool IMUDriver::readMagnetometer(float mag[3]) {
#if IMU_ENABLED
    // IMU library not available - return zeros
    mag[0] = mag[1] = mag[2] = 0.0f;
    return false;
#else
    return false;
#endif
}

float IMUDriver::readTemperature() {
#if IMU_ENABLED
    // IMU library not available
    return 0.0f;
#else
    return 0.0f;
#endif
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void IMUDriver::setAccelRange(uint8_t range) {
#if IMU_ENABLED
    accelRange_ = range;
    // IMU library not available
#endif
}

void IMUDriver::setGyroRange(uint16_t range) {
#if IMU_ENABLED
    gyroRange_ = range;
    // IMU library not available
#endif
}

void IMUDriver::setMagnetometerEnabled(bool enable) {
#if IMU_ENABLED
    magEnabled_ = enable;
    // IMU library not available
#endif
}

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

float IMUDriver::convertAccel(int16_t raw) {
    // Convert raw value to g based on current range
    // ICM-20948 uses 16-bit signed values
    float scale = (float)accelRange_ / 32768.0f;
    return raw * scale;
}

float IMUDriver::convertGyro(int16_t raw) {
    // Convert raw value to deg/sec based on current range
    // ICM-20948 uses 16-bit signed values
    float scale = (float)gyroRange_ / 32768.0f;
    return raw * scale;
}

float IMUDriver::convertMag(int16_t raw) {
    // Convert raw value to µT (microtesla)
    // AK09916 magnetometer: 0.15 µT/LSB
    return raw * 0.15f;
}
