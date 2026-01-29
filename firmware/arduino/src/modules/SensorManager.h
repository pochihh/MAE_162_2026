/**
 * @file SensorManager.h
 * @brief Sensor data aggregation and management
 *
 * This module manages all sensor readings and provides a unified interface
 * for accessing sensor data. It handles:
 * - IMU (accelerometer, gyroscope, magnetometer)
 * - Battery and rail voltage monitoring (ADC)
 * - Temperature sensor (from IMU)
 * - Optional ultrasonic/ToF distance sensors
 *
 * The SensorManager is called periodically from the scheduler (50Hz) to
 * update all sensor readings. Other modules can query the latest data
 * without blocking on I2C or ADC operations.
 *
 * Voltage Dividers (from hardware design):
 * - Battery (VBAT): 1:6 divider (50kΩ/10kΩ) → 0-24V maps to 0-4V ADC
 * - 5V Rail (V5): 1:2 divider (10kΩ/10kΩ) → 0-10V maps to 0-5V ADC
 * - Servo Rail (VSERVO): 1:3 divider (20kΩ/10kΩ) → 0-15V maps to 0-5V ADC
 *
 * Usage:
 *   SensorManager::init();
 *
 *   // In scheduler task @ 50Hz:
 *   SensorManager::update();
 *
 *   // Query latest data:
 *   float vbat = SensorManager::getBatteryVoltage();
 */

#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include <stdint.h>
#include "../drivers/IMUDriver.h"
#include "../config.h"

// ============================================================================
// SENSOR MANAGER CLASS (Static)
// ============================================================================

/**
 * @brief Centralized sensor management
 *
 * Static class providing:
 * - Sensor initialization
 * - Periodic sensor updates
 * - Unified data access interface
 */
class SensorManager {
public:
    /**
     * @brief Initialize all enabled sensors
     *
     * Initializes IMU, configures ADC, and prepares all sensor interfaces.
     * Must be called once in setup() before using sensors.
     */
    static void init();

    /**
     * @brief Update all sensor readings
     *
     * Called from scheduler at 50Hz.
     * Reads all enabled sensors and updates internal buffers.
     */
    static void update();

    // ========================================================================
    // VOLTAGE MONITORING
    // ========================================================================

    /**
     * @brief Get battery voltage
     *
     * @return Battery voltage in volts (0-24V range)
     */
    static float getBatteryVoltage();

    /**
     * @brief Get 5V rail voltage
     *
     * @return 5V rail voltage in volts (nominal 5.0V)
     */
    static float get5VRailVoltage();

    /**
     * @brief Get servo rail voltage
     *
     * @return Servo rail voltage in volts (typically 5-10V)
     */
    static float getServoVoltage();

    /**
     * @brief Check if battery is low
     *
     * @param threshold Low battery threshold in volts (default: 10.5V for 12V NiMH)
     * @return True if battery voltage is below threshold
     */
    static bool isBatteryLow(float threshold = 10.5f);

    // ========================================================================
    // IMU DATA
    // ========================================================================

    /**
     * @brief Get latest accelerometer data
     *
     * @param accel Output array for acceleration [x, y, z] in g
     * @return True if valid data available
     */
    static bool getAcceleration(float accel[3]);

    /**
     * @brief Get latest gyroscope data
     *
     * @param gyro Output array for angular velocity [x, y, z] in deg/sec
     * @return True if valid data available
     */
    static bool getAngularVelocity(float gyro[3]);

    /**
     * @brief Get latest magnetometer data
     *
     * @param mag Output array for magnetic field [x, y, z] in µT
     * @return True if valid data available
     */
    static bool getMagneticField(float mag[3]);

    /**
     * @brief Get IMU temperature
     *
     * @return Temperature in degrees Celsius
     */
    static float getIMUTemperature();

    /**
     * @brief Check if IMU is connected and responding
     *
     * @return True if IMU is available
     */
    static bool isIMUAvailable();

    // ========================================================================
    // STATISTICS
    // ========================================================================

    /**
     * @brief Get sensor update counter
     *
     * Increments each time update() is called.
     * Useful for detecting sensor update frequency.
     *
     * @return Update counter
     */
    static uint32_t getUpdateCount();

    /**
     * @brief Get time of last sensor update
     *
     * @return Timestamp in milliseconds
     */
    static uint32_t getLastUpdateTime();

private:
    // IMU driver
    static IMUDriver imu_;

    // Voltage readings (volts)
    static float batteryVoltage_;
    static float rail5VVoltage_;
    static float servoVoltage_;

    // IMU data buffers
    static float accel_[3];         // Acceleration (g)
    static float gyro_[3];          // Angular velocity (deg/sec)
    static float mag_[3];           // Magnetic field (µT)
    static float imuTemp_;          // Temperature (°C)
    static bool imuDataValid_;      // IMU data validity flag

    // Statistics
    static uint32_t updateCount_;
    static uint32_t lastUpdateTime_;

    // Initialization flags
    static bool initialized_;
    static bool imuInitialized_;

    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    /**
     * @brief Read ADC voltage with averaging
     *
     * @param pin Analog input pin
     * @param numSamples Number of samples to average (default: 4)
     * @return Raw ADC value (0-1023)
     */
    static uint16_t readADCAverage(uint8_t pin, uint8_t numSamples = 4);

    /**
     * @brief Convert ADC reading to voltage
     *
     * @param adcValue Raw ADC value (0-1023)
     * @param dividerRatio Voltage divider ratio (output / input)
     * @return Actual voltage in volts
     */
    static float adcToVoltage(uint16_t adcValue, float dividerRatio);

    /**
     * @brief Update voltage readings
     */
    static void updateVoltages();

    /**
     * @brief Update IMU readings
     */
    static void updateIMU();
};

#endif // SENSORMANAGER_H
