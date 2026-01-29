/**
 * @file test_i2c_scanner.ino
 * @brief I2C bus scanner for debugging
 *
 * Scans I2C bus and reports all detected devices.
 * Useful for verifying PCA9685, IMU, and other I2C peripherals.
 *
 * Hardware:
 * - Arduino Mega 2560
 * - I2C devices connected to SDA (pin 20) and SCL (pin 21)
 * - Pull-up resistors (usually built into modules)
 *
 * Expected Devices:
 * - 0x40: PCA9685 PWM controller (if installed)
 * - 0x68: ICM-20948 IMU (if installed)
 * - 0x70: PCA9685 ALL_CALL address (broadcast, normal)
 */

#include <Wire.h>

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        ; // Wait for serial port to connect
    }

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  I2C Bus Scanner"));
    Serial.println(F("========================================"));
    Serial.println();

    Wire.begin();

    Serial.println(F("Scanning I2C bus (0x01-0x7F)..."));
    Serial.println();

    uint8_t deviceCount = 0;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print(F("Device found at 0x"));
            if (address < 16) Serial.print(F("0"));
            Serial.print(address, HEX);
            Serial.print(F("  "));

            // Identify common devices
            switch (address) {
                case 0x40:
                    Serial.println(F("(PCA9685 PWM Controller)"));
                    break;
                case 0x68:
                    Serial.println(F("(ICM-20948 IMU or MPU-6050)"));
                    break;
                case 0x69:
                    Serial.println(F("(ICM-20948 IMU alternate)"));
                    break;
                case 0x70:
                    Serial.println(F("(PCA9685 ALL_CALL broadcast)"));
                    break;
                default:
                    Serial.println(F("(Unknown device)"));
                    break;
            }

            deviceCount++;
        } else if (error == 4) {
            Serial.print(F("Unknown error at address 0x"));
            if (address < 16) Serial.print(F("0"));
            Serial.println(address, HEX);
        }
    }

    Serial.println();
    if (deviceCount == 0) {
        Serial.println(F("No I2C devices found!"));
        Serial.println(F("Check:"));
        Serial.println(F("  - SDA/SCL connections (pins 20/21)"));
        Serial.println(F("  - Pull-up resistors (4.7kΩ)"));
        Serial.println(F("  - Device power supply"));
    } else {
        Serial.print(F("Found "));
        Serial.print(deviceCount);
        Serial.println(F(" device(s)"));
    }

    Serial.println();
    Serial.println(F("Scan complete. Type any key to rescan."));
    Serial.println(F("========================================"));
}

void loop() {
    if (Serial.available() > 0) {
        Serial.read();  // Clear buffer
        Serial.println(F("\nRescanning..."));
        delay(100);
        setup();  // Re-run scan
    }
}
