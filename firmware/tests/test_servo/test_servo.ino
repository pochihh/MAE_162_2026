/**
 * @file test_servo.ino
 * @brief Test sketch for servo controller (Phase 4)
 *
 * This test validates:
 * - PCA9685 I2C PWM controller initialization
 * - 16-channel servo control
 * - Pulse width control (microseconds)
 * - Angle control (degrees)
 * - Output enable/disable functionality
 * - Bulk servo updates for synchronized motion
 *
 * Hardware Requirements:
 * - Arduino Mega 2560
 * - PCA9685 16-channel PWM controller
 * - I2C connection (SDA=20, SCL=21)
 * - At least one servo motor connected
 * - Servo power supply (5-10V depending on servos)
 *
 * Expected Behavior:
 * - Servos respond to position commands
 * - Pulse width range: 500-2500µs (typical)
 * - Angle range: 0-180° (typical)
 * - Smooth motion on multiple servos
 *
 * Serial Commands:
 *   e              - Enable servo outputs
 *   d              - Disable servo outputs
 *   s<ch>,<us>     - Set servo pulse width (e.g., "s0,1500" = channel 0 to 1500µs)
 *   a<ch>,<deg>    - Set servo angle (e.g., "a0,90" = channel 0 to 90°)
 *   c<ch>          - Center servo (1500µs or 90°)
 *   w<ch>          - Sweep servo back and forth
 *   W              - Sweep all servos
 *   0-9, A-F       - Quick set channel (0-15) to center position
 *   ?              - Print status
 *   help           - Show command list
 *
 * Test Sequence:
 * 1. Enable outputs: "e"
 * 2. Center servo 0: "c0"
 * 3. Sweep to min: "a0,0"
 * 4. Sweep to max: "a0,180"
 * 5. Test pulse width: "s0,1000" then "s0,2000"
 * 6. Sweep test: "w0"
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/drivers/ServoController.h"
#include <Wire.h>

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Command buffer for serial input
char commandBuffer[64];
uint8_t bufferIndex = 0;

// Status update period
uint32_t lastStatusTime = 0;
const uint32_t STATUS_PERIOD_MS = 5000;

// Sweep animation state
bool sweepActive = false;
uint8_t sweepChannel = 0;
bool sweepAllChannels = false;
uint32_t sweepLastUpdate = 0;
const uint32_t SWEEP_STEP_MS = 20;
float sweepAngle = 90.0f;
float sweepDirection = 1.0f;
const float SWEEP_STEP = 2.0f;

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
    Serial.println(F("  Servo Controller Test - Phase 4"));
    Serial.println(F("  PCA9685 16-Channel PWM"));
    Serial.println(F("========================================"));
    Serial.println();

    // Initialize ServoController
    Serial.println(F("[Setup] Initializing servo controller..."));
    ServoController::init();

    // Configure pulse width and angle ranges
    ServoController::setPulseWidthRange(500, 2500);  // Standard servo range
    ServoController::setAngleRange(0.0f, 180.0f);    // Standard angle range

    Serial.println(F("  - PCA9685 initialized"));
    Serial.println(F("  - I2C Address: 0x40 (default)"));
    Serial.println(F("  - PWM Frequency: 50Hz (servo standard)"));
    Serial.println(F("  - Pulse Range: 500-2500µs"));
    Serial.println(F("  - Angle Range: 0-180°"));
    Serial.println();

    Serial.println(F("[Setup] Initialization complete!"));
    Serial.println(F("[Setup] Note: Servo outputs are DISABLED by default"));
    Serial.println(F("[Setup] Use 'e' command to enable outputs"));
    Serial.println();
    Serial.println(F("Type 'help' for command list"));
    Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Process serial commands
    processSerialCommands();

    // Update sweep animation if active
    if (sweepActive) {
        updateSweep();
    }

    // Print status periodically
    if (millis() - lastStatusTime >= STATUS_PERIOD_MS) {
        lastStatusTime = millis();
        printStatus();
    }
}

// ============================================================================
// SERIAL COMMAND PROCESSING
// ============================================================================

void processSerialCommands() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        // Handle newline/carriage return - execute command
        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                commandBuffer[bufferIndex] = '\0';
                executeCommand(commandBuffer);
                bufferIndex = 0;
            }
        }
        // Add character to buffer
        else if (bufferIndex < sizeof(commandBuffer) - 1) {
            commandBuffer[bufferIndex++] = c;
        }
        // Buffer overflow - reset
        else {
            Serial.println(F("ERROR: Command too long"));
            bufferIndex = 0;
        }
    }
}

void executeCommand(const char* cmd) {
    // Skip empty commands
    if (cmd[0] == '\0') return;

    // Help command
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
        return;
    }

    // Enable outputs
    if (strcmp(cmd, "e") == 0) {
        Serial.println(F(">>> Enable servo outputs"));
        ServoController::enable();
        return;
    }

    // Disable outputs
    if (strcmp(cmd, "d") == 0) {
        Serial.println(F(">>> Disable servo outputs"));
        ServoController::disable();
        sweepActive = false;
        return;
    }

    // Sweep all channels
    if (strcmp(cmd, "W") == 0) {
        Serial.println(F(">>> Sweep ALL channels (press any key to stop)"));
        sweepActive = true;
        sweepAllChannels = true;
        sweepAngle = 90.0f;
        sweepDirection = 1.0f;
        return;
    }

    // Quick center commands (0-9, A-F for channels 0-15)
    if (strlen(cmd) == 1) {
        char c = cmd[0];
        uint8_t channel = 255;

        if (c >= '0' && c <= '9') {
            channel = c - '0';
        } else if (c >= 'A' && c <= 'F') {
            channel = 10 + (c - 'A');
        } else if (c >= 'a' && c <= 'f') {
            channel = 10 + (c - 'a');
        }

        if (channel < 16) {
            Serial.print(F(">>> Center channel "));
            Serial.println(channel);
            ServoController::setPositionDeg(channel, 90.0f);
            return;
        }
    }

    // Parse command character and channel
    char cmdChar = cmd[0];
    uint8_t channel = 255;
    int32_t value = 0;

    // Parse channel and value
    int parsed = sscanf(cmd + 1, "%hhu,%ld", &channel, &value);

    // Validate channel
    if (channel >= 16) {
        Serial.print(F("ERROR: Invalid channel "));
        Serial.println(channel);
        return;
    }

    // Execute command
    switch (cmdChar) {
        case 's': // Set pulse width (microseconds)
            if (parsed >= 2) {
                Serial.print(F(">>> Set channel "));
                Serial.print(channel);
                Serial.print(F(" to "));
                Serial.print(value);
                Serial.println(F("µs"));
                ServoController::setPositionUs(channel, (uint16_t)value);
                sweepActive = false;
            } else {
                Serial.println(F("ERROR: Format: s<ch>,<us>"));
            }
            break;

        case 'a': // Set angle (degrees)
            if (parsed >= 2) {
                Serial.print(F(">>> Set channel "));
                Serial.print(channel);
                Serial.print(F(" to "));
                Serial.print(value);
                Serial.println(F("°"));
                ServoController::setPositionDeg(channel, (float)value);
                sweepActive = false;
            } else {
                Serial.println(F("ERROR: Format: a<ch>,<deg>"));
            }
            break;

        case 'c': // Center servo
            if (parsed >= 1) {
                Serial.print(F(">>> Center channel "));
                Serial.println(channel);
                ServoController::setPositionDeg(channel, 90.0f);
                sweepActive = false;
            } else {
                Serial.println(F("ERROR: Format: c<ch>"));
            }
            break;

        case 'w': // Sweep single channel
            if (parsed >= 1) {
                Serial.print(F(">>> Sweep channel "));
                Serial.print(channel);
                Serial.println(F(" (press any key to stop)"));
                sweepActive = true;
                sweepChannel = channel;
                sweepAllChannels = false;
                sweepAngle = 90.0f;
                sweepDirection = 1.0f;
            } else {
                Serial.println(F("ERROR: Format: w<ch>"));
            }
            break;

        default:
            Serial.print(F("ERROR: Unknown command '"));
            Serial.print(cmdChar);
            Serial.println(F("'"));
            break;
    }
}

// ============================================================================
// SWEEP ANIMATION
// ============================================================================

void updateSweep() {
    if (millis() - sweepLastUpdate < SWEEP_STEP_MS) {
        return;
    }
    sweepLastUpdate = millis();

    // Update angle
    sweepAngle += sweepDirection * SWEEP_STEP;

    // Reverse direction at limits
    if (sweepAngle >= 180.0f) {
        sweepAngle = 180.0f;
        sweepDirection = -1.0f;
    } else if (sweepAngle <= 0.0f) {
        sweepAngle = 0.0f;
        sweepDirection = 1.0f;
    }

    // Update servo(s)
    if (sweepAllChannels) {
        for (uint8_t ch = 0; ch < 16; ch++) {
            ServoController::setPositionDeg(ch, sweepAngle);
        }
    } else {
        ServoController::setPositionDeg(sweepChannel, sweepAngle);
    }

    // Check for stop command
    if (Serial.available() > 0) {
        Serial.read();  // Consume character
        Serial.println(F(">>> Sweep stopped"));
        sweepActive = false;
    }
}

// ============================================================================
// STATUS DISPLAY
// ============================================================================

void printStatus() {
    Serial.println(F("----------------------------------------"));
    Serial.println(F("Servo Controller Status:"));
    Serial.print(F("  Outputs: "));
    Serial.println(ServoController::isEnabled() ? F("ENABLED") : F("DISABLED"));
    Serial.println();
    Serial.println(F("Channel Positions (µs):"));

    for (uint8_t ch = 0; ch < 16; ch++) {
        uint16_t pulseUs = ServoController::getPositionUs(ch);

        Serial.print(F("  Ch"));
        if (ch < 10) Serial.print(F(" "));
        Serial.print(ch);
        Serial.print(F(": "));

        if (pulseUs == 0) {
            Serial.println(F("OFF"));
        } else {
            Serial.print(pulseUs);
            Serial.println(F("µs"));
        }
    }

    Serial.println(F("----------------------------------------"));
}

void printHelp() {
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  Servo Controller Test Commands"));
    Serial.println(F("========================================"));
    Serial.println(F("Control:"));
    Serial.println(F("  e              - Enable servo outputs"));
    Serial.println(F("  d              - Disable servo outputs"));
    Serial.println(F("  s<ch>,<us>     - Set pulse width (µs)"));
    Serial.println(F("  a<ch>,<deg>    - Set angle (degrees)"));
    Serial.println(F("  c<ch>          - Center servo (90° / 1500µs)"));
    Serial.println(F("  w<ch>          - Sweep single channel"));
    Serial.println(F("  W              - Sweep all channels"));
    Serial.println();
    Serial.println(F("Quick Commands:"));
    Serial.println(F("  0-9, A-F       - Center channel 0-15"));
    Serial.println();
    Serial.println(F("Info:"));
    Serial.println(F("  ?              - Print status (auto every 5s)"));
    Serial.println(F("  help           - Show this help"));
    Serial.println();
    Serial.println(F("Examples:"));
    Serial.println(F("  e              - Enable outputs"));
    Serial.println(F("  c0             - Center servo on channel 0"));
    Serial.println(F("  a0,0           - Move channel 0 to 0°"));
    Serial.println(F("  a0,180         - Move channel 0 to 180°"));
    Serial.println(F("  s0,1000        - Set channel 0 to 1000µs"));
    Serial.println(F("  s0,2000        - Set channel 0 to 2000µs"));
    Serial.println(F("  w0             - Sweep channel 0"));
    Serial.println(F("  W              - Sweep all channels"));
    Serial.println(F("  0              - Quick center channel 0"));
    Serial.println(F("  5              - Quick center channel 5"));
    Serial.println(F("  A              - Quick center channel 10"));
    Serial.println(F("  F              - Quick center channel 15"));
    Serial.println(F("========================================"));
    Serial.println();
}
