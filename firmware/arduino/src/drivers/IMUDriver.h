/**
 * @file IMUDriver.h
 * @brief Wrapper for ICM-20948 9-axis IMU
 *
 * This module provides a simplified interface to the ICM-20948 IMU sensor.
 * The ICM-20948 combines:
 * - 3-axis accelerometer
 * - 3-axis gyroscope
 * - 3-axis magnetometer (AK09916)
 *
 * Features:
 * - I2C communication (address 0x68 or 0x69)
 * - Configurable accelerometer range (±2g, ±4g, ±8g, ±16g)
 * - Configurable gyroscope range (±250, ±500, ±1000, ±2000 dps)
 * - Digital Motion Processor (DMP) support (future)
 * - Temperature sensor
 *
 * Hardware Connection:
 * - SDA/SCL: I2C bus (shared with other I2C devices)
 * - INT: Interrupt pin (optional, for data ready signal)
 * - VCC: 3.3V or 5V (internal LDO for 3.3V logic)
 *
 * Usage:
 *   IMUDriver imu;
 *   imu.init();
 *
 *   float accel[3], gyro[3];
 *   if (imu.readAccelGyro(accel, gyro)) {
 *       // Process IMU data
 *   }
 */

#ifndef IMUDRIVER_H
#define IMUDRIVER_H

#include <Arduino.h>
#include <stdint.h>
#include "../config.h"

// Forward declaration - only include if IMU is enabled and library is available
#if IMU_ENABLED
  // Note: ICM_20948 library requires util/ICM_20948_C.h
  // If library is not fully installed, IMU functionality will be disabled
  #ifdef ICM_20948_H_
    class ICM_20948_I2C;
  #else
    // Stub class for compilation when library is missing
    class ICM_20948_I2C {};
  #endif
#else
  class ICM_20948_I2C {};  // Stub when IMU disabled
#endif

// ============================================================================
// IMU DRIVER CLASS
// ============================================================================

/**
 * @brief ICM-20948 IMU driver wrapper
 *
 * Provides simplified interface to ICM-20948 library.
 * Handles initialization, configuration, and data reading.
 */
class IMUDriver {
public:
    IMUDriver();

    /**
     * @brief Initialize IMU sensor
     *
     * Configures I2C communication and sensor settings.
     * Sets default accelerometer and gyroscope ranges.
     *
     * @param i2cAddress I2C address (0x68 or 0x69, default 0x68)
     * @return True if initialization successful
     */
    bool init(uint8_t i2cAddress = 0x68);

    /**
     * @brief Check if IMU is connected and responding
     *
     * @return True if IMU is available
     */
    bool isConnected();

    /**
     * @brief Read accelerometer and gyroscope data
     *
     * @param accel Output array for acceleration [x, y, z] in g (9.8 m/s²)
     * @param gyro Output array for angular velocity [x, y, z] in deg/sec
     * @return True if read successful
     */
    bool readAccelGyro(float accel[3], float gyro[3]);

    /**
     * @brief Read magnetometer data
     *
     * @param mag Output array for magnetic field [x, y, z] in µT (microtesla)
     * @return True if read successful
     */
    bool readMagnetometer(float mag[3]);

    /**
     * @brief Read temperature from IMU
     *
     * @return Temperature in degrees Celsius
     */
    float readTemperature();

    /**
     * @brief Set accelerometer range
     *
     * @param range Accelerometer range (2, 4, 8, or 16 for ±Xg)
     */
    void setAccelRange(uint8_t range);

    /**
     * @brief Set gyroscope range
     *
     * @param range Gyroscope range (250, 500, 1000, or 2000 for ±X dps)
     */
    void setGyroRange(uint16_t range);

    /**
     * @brief Get current accelerometer range
     *
     * @return Accelerometer range in ±g
     */
    uint8_t getAccelRange() const { return accelRange_; }

    /**
     * @brief Get current gyroscope range
     *
     * @return Gyroscope range in ±dps
     */
    uint16_t getGyroRange() const { return gyroRange_; }

    /**
     * @brief Enable/disable magnetometer
     *
     * @param enable True to enable magnetometer
     */
    void setMagnetometerEnabled(bool enable);

    /**
     * @brief Check if magnetometer is enabled
     *
     * @return True if magnetometer is enabled
     */
    bool isMagnetometerEnabled() const { return magEnabled_; }

private:
    ICM_20948_I2C imu_;         // ICM-20948 library instance
    bool initialized_;          // Initialization state
    uint8_t i2cAddress_;        // I2C address (0x68 or 0x69)
    uint8_t accelRange_;        // Accelerometer range (±g)
    uint16_t gyroRange_;        // Gyroscope range (±dps)
    bool magEnabled_;           // Magnetometer enabled flag

    /**
     * @brief Convert raw accelerometer data to g
     *
     * @param raw Raw sensor value
     * @return Acceleration in g
     */
    float convertAccel(int16_t raw);

    /**
     * @brief Convert raw gyroscope data to deg/sec
     *
     * @param raw Raw sensor value
     * @return Angular velocity in deg/sec
     */
    float convertGyro(int16_t raw);

    /**
     * @brief Convert raw magnetometer data to µT
     *
     * @param raw Raw sensor value
     * @return Magnetic field in microtesla
     */
    float convertMag(int16_t raw);
};

#endif // IMUDRIVER_H
