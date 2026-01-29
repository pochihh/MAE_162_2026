/**
 * @file test_voltage.ino
 * @brief Test sketch for voltage monitoring (Phase 5)
 *
 * This test validates:
 * - Battery voltage monitoring (A0 with 1:6 divider)
 * - 5V rail voltage monitoring (A1 with 1:2 divider)
 * - Servo rail voltage monitoring (A2 with 1:3 divider)
 * - ADC accuracy and calibration
 * - Low battery threshold detection
 *
 * Hardware Requirements:
 * - Arduino Mega 2560
 * - Voltage dividers on A0, A1, A2 (as per PCB specs)
 * - Battery connected (12V NiMH or 7.4-24V LiPo)
 * - 5V and servo power rails active
 *
 * Expected Behavior:
 * - Voltage readings update every second
 * - Values should match actual voltages (±5%)
 * - Low battery warning appears when battery < 10.5V
 * - ADC readings are stable (minimal noise)
 *
 * Voltage Divider Ratios (from config.h):
 * - Battery: 1:6 (50kΩ + 10kΩ) → 0-24V → 0-4V ADC
 * - 5V Rail: 1:2 → 0-10V → 0-5V ADC
 * - Servo Rail: 1:3 → 0-15V → 0-5V ADC
 *
 * Calibration:
 * - Compare readings to multimeter measurements
 * - Adjust divider ratios in config.h if needed
 * - Verify reference voltage (ADC_VREF = 5.0V)
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/modules/SensorManager.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

uint32_t lastPrintTime = 0;
const uint32_t PRINT_PERIOD_MS = 1000;  // 1Hz update rate

// Statistics for noise measurement
const uint8_t SAMPLE_COUNT = 10;
float batteryReadings[SAMPLE_COUNT];
uint8_t sampleIndex = 0;
bool samplesReady = false;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {
        ; // Wait for serial port to connect
    }

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  Voltage Monitoring Test - Phase 5"));
    Serial.println(F("========================================"));
    Serial.println();

    // Print configuration
    Serial.println(F("[Config] Voltage Divider Settings:"));
    Serial.print(F("  - Battery (A0): 1:"));
    Serial.print(VBAT_DIVIDER_RATIO, 1);
    Serial.print(F(" ("));
    Serial.print(VBAT_DIVIDER_R1 / 1000.0f, 0);
    Serial.print(F("kΩ + "));
    Serial.print(VBAT_DIVIDER_R2 / 1000.0f, 0);
    Serial.println(F("kΩ)"));

    Serial.print(F("  - 5V Rail (A1): 1:"));
    Serial.println(V5_DIVIDER_RATIO, 1);

    Serial.print(F("  - Servo Rail (A2): 1:"));
    Serial.println(VSERVO_DIVIDER_RATIO, 1);

    Serial.print(F("  - ADC Reference: "));
    Serial.print(ADC_VREF, 1);
    Serial.println(F("V"));

    Serial.print(F("  - ADC Resolution: "));
    Serial.print(ADC_RESOLUTION);
    Serial.println(F(" bits (10-bit)"));

    Serial.print(F("  - Low Battery Threshold: "));
    Serial.print(VBAT_LOW_THRESHOLD, 1);
    Serial.println(F("V"));
    Serial.println();

    // Initialize SensorManager
    Serial.println(F("[Setup] Initializing SensorManager..."));
    SensorManager::init();
    Serial.println(F("  - Voltage monitoring initialized"));
    Serial.println();

    // Configure analog pins explicitly
#if VBAT_ENABLED
    pinMode(PIN_VBAT_SENSE, INPUT);
#endif
#if V5_ENABLED
    pinMode(PIN_V5_SENSE, INPUT);
#endif
#if VSERVO_ENABLED
    pinMode(PIN_VSERVO_SENSE, INPUT);
#endif

    Serial.println(F("[Setup] Initialization complete!"));
    Serial.println();
    Serial.println(F("Reading voltages every 1 second..."));
    Serial.println(F("Compare with multimeter for calibration."));
    Serial.println(F("========================================"));
    Serial.println();

    // Print header
    printHeader();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Update sensor readings
    SensorManager::update();

    // Print readings periodically
    if (millis() - lastPrintTime >= PRINT_PERIOD_MS) {
        lastPrintTime = millis();

        // Get voltage readings
        float vbat = SensorManager::getBatteryVoltage();
        float v5 = SensorManager::get5VRailVoltage();
        float vservo = SensorManager::getServoVoltage();
        bool lowBattery = SensorManager::isBatteryLow();

        // Store battery reading for statistics
        batteryReadings[sampleIndex] = vbat;
        sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
        if (sampleIndex == 0) {
            samplesReady = true;
        }

        // Print readings
        printReadings(vbat, v5, vservo, lowBattery);

        // Print statistics every 10 samples
        if (samplesReady && sampleIndex == 0) {
            printStatistics();
        }

        // Check for low battery
        if (lowBattery) {
            Serial.println(F("*** LOW BATTERY WARNING! ***"));
        }
    }

    // Check for serial commands
    if (Serial.available() > 0) {
        char c = Serial.read();

        switch (c) {
            case 'h':
            case 'H':
            case '?':
                printHelp();
                break;

            case 'c':
            case 'C':
                printCalibration();
                break;

            case 'r':
            case 'R':
                printRawADC();
                break;

            case 's':
            case 'S':
                printStatistics();
                break;

            case '\n':
            case '\r':
                // Ignore newlines
                break;

            default:
                Serial.print(F("Unknown command: "));
                Serial.println(c);
                Serial.println(F("Type 'h' for help"));
                break;
        }
    }
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

void printHeader() {
    Serial.println(F("Time(s) | Battery  | 5V Rail  | Servo    | Status"));
    Serial.println(F("--------|----------|----------|----------|--------"));
}

void printReadings(float vbat, float v5, float vservo, bool lowBat) {
    // Time since startup
    uint32_t seconds = millis() / 1000;
    if (seconds < 100) Serial.print(F(" "));
    if (seconds < 10) Serial.print(F(" "));
    Serial.print(seconds);
    Serial.print(F("     | "));

    // Battery voltage
    if (vbat < 10.0f) Serial.print(F(" "));
    Serial.print(vbat, 2);
    Serial.print(F("V  | "));

    // 5V rail voltage
    Serial.print(v5, 2);
    Serial.print(F("V  | "));

    // Servo voltage
    if (vservo < 10.0f) Serial.print(F(" "));
    Serial.print(vservo, 2);
    Serial.print(F("V  | "));

    // Status
    if (lowBat) {
        Serial.println(F("LOW!"));
    } else {
        Serial.println(F("OK"));
    }
}

void printRawADC() {
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("Raw ADC Readings:"));
    Serial.println(F("========================================"));

#if VBAT_ENABLED
    int rawBat = analogRead(PIN_VBAT_SENSE);
    Serial.print(F("Battery (A0):  "));
    if (rawBat < 1000) Serial.print(F(" "));
    if (rawBat < 100) Serial.print(F(" "));
    if (rawBat < 10) Serial.print(F(" "));
    Serial.print(rawBat);
    Serial.print(F(" / 1023  ("));
    Serial.print((rawBat * 100.0f) / 1023.0f, 1);
    Serial.println(F("%)"));
#endif

#if V5_ENABLED
    int raw5V = analogRead(PIN_V5_SENSE);
    Serial.print(F("5V Rail (A1):  "));
    if (raw5V < 1000) Serial.print(F(" "));
    if (raw5V < 100) Serial.print(F(" "));
    if (raw5V < 10) Serial.print(F(" "));
    Serial.print(raw5V);
    Serial.print(F(" / 1023  ("));
    Serial.print((raw5V * 100.0f) / 1023.0f, 1);
    Serial.println(F("%)"));
#endif

#if VSERVO_ENABLED
    int rawServo = analogRead(PIN_VSERVO_SENSE);
    Serial.print(F("Servo (A2):    "));
    if (rawServo < 1000) Serial.print(F(" "));
    if (rawServo < 100) Serial.print(F(" "));
    if (rawServo < 10) Serial.print(F(" "));
    Serial.print(rawServo);
    Serial.print(F(" / 1023  ("));
    Serial.print((rawServo * 100.0f) / 1023.0f, 1);
    Serial.println(F("%)"));
#endif

    Serial.println(F("========================================"));
    Serial.println();
}

void printStatistics() {
    if (!samplesReady) {
        Serial.println(F("Not enough samples yet..."));
        return;
    }

    // Calculate statistics
    float sum = 0.0f;
    float min = batteryReadings[0];
    float max = batteryReadings[0];

    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
        float v = batteryReadings[i];
        sum += v;
        if (v < min) min = v;
        if (v > max) max = v;
    }

    float avg = sum / SAMPLE_COUNT;
    float range = max - min;

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("Battery Voltage Statistics (10 samples):"));
    Serial.println(F("========================================"));
    Serial.print(F("  Average: "));
    Serial.print(avg, 3);
    Serial.println(F("V"));
    Serial.print(F("  Minimum: "));
    Serial.print(min, 3);
    Serial.println(F("V"));
    Serial.print(F("  Maximum: "));
    Serial.print(max, 3);
    Serial.println(F("V"));
    Serial.print(F("  Range:   "));
    Serial.print(range, 3);
    Serial.print(F("V ("));
    Serial.print((range / avg) * 100.0f, 2);
    Serial.println(F("%)"));

    if (range > 0.2f) {
        Serial.println(F("  Warning: High voltage noise detected!"));
    } else if (range > 0.1f) {
        Serial.println(F("  Note: Moderate voltage noise"));
    } else {
        Serial.println(F("  Noise level: Acceptable"));
    }

    Serial.println(F("========================================"));
    Serial.println();
}

void printCalibration() {
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("Calibration Instructions:"));
    Serial.println(F("========================================"));
    Serial.println(F("1. Measure actual voltages with multimeter:"));
    Serial.println(F("   - Battery voltage (main power input)"));
    Serial.println(F("   - 5V rail (Arduino VCC)"));
    Serial.println(F("   - Servo rail (if applicable)"));
    Serial.println();
    Serial.println(F("2. Compare with readings above"));
    Serial.println();
    Serial.println(F("3. Calculate correction factor:"));
    Serial.println(F("   New Ratio = Old Ratio × (Actual / Measured)"));
    Serial.println();
    Serial.println(F("4. Update config.h divider ratios:"));
    Serial.println(F("   - VBAT_DIVIDER_RATIO"));
    Serial.println(F("   - V5_DIVIDER_RATIO"));
    Serial.println(F("   - VSERVO_DIVIDER_RATIO"));
    Serial.println();
    Serial.println(F("5. Recompile and verify accuracy"));
    Serial.println(F("========================================"));
    Serial.println();
}

void printHelp() {
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("Voltage Monitoring Test Commands:"));
    Serial.println(F("========================================"));
    Serial.println(F("  h or ? - Show this help"));
    Serial.println(F("  r      - Print raw ADC values"));
    Serial.println(F("  s      - Print voltage statistics"));
    Serial.println(F("  c      - Show calibration instructions"));
    Serial.println(F("========================================"));
    Serial.println();
}
