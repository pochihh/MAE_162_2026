/**
 * @file test_stepper.ino
 * @brief Test sketch for stepper motor control (Phase 4)
 *
 * This test validates:
 * - Timer3 @ 10kHz configuration
 * - Stepper motor initialization and pin configuration
 * - Motion profiles (acceleration, cruise, deceleration)
 * - Relative and absolute positioning
 * - Velocity and acceleration settings
 * - Emergency stop functionality
 * - Limit switch homing (if configured)
 *
 * Hardware Requirements:
 * - Arduino Mega 2560
 * - At least one stepper motor driver (A4988/DRV8825)
 * - Stepper motor connected to driver
 * - Optional: Limit switch for homing tests
 *
 * Expected Behavior:
 * - Stepper motors respond to serial commands
 * - Motion follows trapezoidal velocity profile
 * - Position tracking is accurate
 * - Limit switches stop motion when triggered
 *
 * Serial Commands:
 *   e<id>          - Enable stepper (id: 0-3)
 *   d<id>          - Disable stepper (id: 0-3)
 *   m<id>,<steps>  - Move relative steps (e.g., "m0,200" moves stepper 0 by 200 steps)
 *   p<id>,<pos>    - Move to absolute position (e.g., "p0,1000")
 *   v<id>,<vel>    - Set max velocity (e.g., "v0,500" sets 500 steps/sec)
 *   a<id>,<accel>  - Set acceleration (e.g., "a0,300" sets 300 steps/sec²)
 *   h<id>,<dir>    - Home stepper (dir: 1 or -1)
 *   s<id>          - Emergency stop stepper
 *   S              - Stop all steppers
 *   ?              - Print status of all steppers
 *   help           - Show command list
 *
 * Test Sequence:
 * 1. Enable stepper 0: "e0"
 * 2. Set velocity: "v0,500"
 * 3. Set acceleration: "a0,300"
 * 4. Move 200 steps: "m0,200"
 * 5. Wait for completion
 * 6. Move to position 0: "p0,0"
 * 7. Check status: "?"
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/modules/StepperManager.h"
#include "src/drivers/StepperMotor.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Command buffer for serial input
char commandBuffer[64];
uint8_t bufferIndex = 0;

// Status update period
uint32_t lastStatusTime = 0;
const uint32_t STATUS_PERIOD_MS = 2000;

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
    Serial.println(F("  Stepper Motor Test - Phase 4"));
    Serial.println(F("========================================"));
    Serial.println();

    // Initialize StepperManager (configures Timer3 @ 10kHz)
    Serial.println(F("[Setup] Initializing stepper motors..."));
    StepperManager::init();

    // Configure stepper 0 (always enabled in config)
#if STEPPER_1_ENABLED
    StepperMotor* stepper0 = StepperManager::getStepper(0);
    if (stepper0) {
        stepper0->setPins(PIN_ST1_STEP, PIN_ST1_DIR, PIN_ST1_EN);
#if defined(PIN_ST1_LIMIT)
        stepper0->setLimitPin(PIN_ST1_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
        Serial.println(F("  - Stepper 0: Limit switch configured"));
#endif
        stepper0->setMaxVelocity(1000);     // 1000 steps/sec default
        stepper0->setAcceleration(500);     // 500 steps/sec² default
        Serial.println(F("  - Stepper 0: Initialized"));
    }
#endif

    // Configure stepper 1 (if enabled)
#if STEPPER_2_ENABLED
    StepperMotor* stepper1 = StepperManager::getStepper(1);
    if (stepper1) {
        stepper1->setPins(PIN_ST2_STEP, PIN_ST2_DIR, PIN_ST2_EN);
#if defined(PIN_ST2_LIMIT)
        stepper1->setLimitPin(PIN_ST2_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
        Serial.println(F("  - Stepper 1: Limit switch configured"));
#endif
        stepper1->setMaxVelocity(1000);
        stepper1->setAcceleration(500);
        Serial.println(F("  - Stepper 1: Initialized"));
    }
#endif

    // Configure stepper 2 (if enabled)
#if STEPPER_3_ENABLED
    StepperMotor* stepper2 = StepperManager::getStepper(2);
    if (stepper2) {
        stepper2->setPins(PIN_ST3_STEP, PIN_ST3_DIR, PIN_ST3_EN);
#if defined(PIN_ST3_LIMIT)
        stepper2->setLimitPin(PIN_ST3_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
        Serial.println(F("  - Stepper 2: Limit switch configured"));
#endif
        stepper2->setMaxVelocity(1000);
        stepper2->setAcceleration(500);
        Serial.println(F("  - Stepper 2: Initialized"));
    }
#endif

    // Configure stepper 3 (if enabled)
#if STEPPER_4_ENABLED
    StepperMotor* stepper3 = StepperManager::getStepper(3);
    if (stepper3) {
        stepper3->setPins(PIN_ST4_STEP, PIN_ST4_DIR, PIN_ST4_EN);
#if defined(PIN_ST4_LIMIT)
        stepper3->setLimitPin(PIN_ST4_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
        Serial.println(F("  - Stepper 3: Limit switch configured"));
#endif
        stepper3->setMaxVelocity(1000);
        stepper3->setAcceleration(500);
        Serial.println(F("  - Stepper 3: Initialized"));
    }
#endif

    Serial.println();
    Serial.println(F("[Setup] Initialization complete!"));
    Serial.println(F("[Setup] Timer3 running @ 10kHz for step generation"));
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

    // Emergency stop all
    if (strcmp(cmd, "S") == 0) {
        Serial.println(F(">>> Emergency stop ALL steppers"));
        StepperManager::emergencyStopAll();
        return;
    }

    // Parse command character and stepper ID
    char cmdChar = cmd[0];
    uint8_t stepperId = 255;
    int32_t value1 = 0;
    int32_t value2 = 0;

    // Parse stepper ID and values
    int parsed = sscanf(cmd + 1, "%hhu,%ld,%ld", &stepperId, &value1, &value2);
    if (parsed < 1) {
        parsed = sscanf(cmd + 1, "%hhu", &stepperId);
    }

    // Validate stepper ID
    if (stepperId >= NUM_STEPPERS) {
        Serial.print(F("ERROR: Invalid stepper ID "));
        Serial.println(stepperId);
        return;
    }

    StepperMotor* stepper = StepperManager::getStepper(stepperId);
    if (!stepper) {
        Serial.println(F("ERROR: Stepper not initialized"));
        return;
    }

    // Execute command
    switch (cmdChar) {
        case 'e': // Enable
            Serial.print(F(">>> Enable stepper "));
            Serial.println(stepperId);
            stepper->enable();
            break;

        case 'd': // Disable
            Serial.print(F(">>> Disable stepper "));
            Serial.println(stepperId);
            stepper->disable();
            break;

        case 'm': // Move relative
            if (parsed >= 2) {
                Serial.print(F(">>> Move stepper "));
                Serial.print(stepperId);
                Serial.print(F(" by "));
                Serial.print(value1);
                Serial.println(F(" steps"));
                stepper->moveSteps(value1);
            } else {
                Serial.println(F("ERROR: Format: m<id>,<steps>"));
            }
            break;

        case 'p': // Move absolute
            if (parsed >= 2) {
                Serial.print(F(">>> Move stepper "));
                Serial.print(stepperId);
                Serial.print(F(" to position "));
                Serial.println(value1);
                stepper->moveToPosition(value1);
            } else {
                Serial.println(F("ERROR: Format: p<id>,<position>"));
            }
            break;

        case 'v': // Set velocity
            if (parsed >= 2 && value1 > 0) {
                Serial.print(F(">>> Set stepper "));
                Serial.print(stepperId);
                Serial.print(F(" velocity to "));
                Serial.print(value1);
                Serial.println(F(" steps/sec"));
                stepper->setMaxVelocity((uint16_t)value1);
            } else {
                Serial.println(F("ERROR: Format: v<id>,<vel> (vel > 0)"));
            }
            break;

        case 'a': // Set acceleration
            if (parsed >= 2 && value1 > 0) {
                Serial.print(F(">>> Set stepper "));
                Serial.print(stepperId);
                Serial.print(F(" acceleration to "));
                Serial.print(value1);
                Serial.println(F(" steps/sec²"));
                stepper->setAcceleration((uint16_t)value1);
            } else {
                Serial.println(F("ERROR: Format: a<id>,<accel> (accel > 0)"));
            }
            break;

        case 'h': // Home
            if (parsed >= 2 && (value1 == 1 || value1 == -1)) {
                Serial.print(F(">>> Home stepper "));
                Serial.print(stepperId);
                Serial.print(F(" in direction "));
                Serial.println(value1);
                stepper->home((int8_t)value1);
            } else {
                Serial.println(F("ERROR: Format: h<id>,<dir> (dir: 1 or -1)"));
            }
            break;

        case 's': // Stop
            Serial.print(F(">>> Emergency stop stepper "));
            Serial.println(stepperId);
            stepper->stop();
            break;

        default:
            Serial.print(F("ERROR: Unknown command '"));
            Serial.print(cmdChar);
            Serial.println(F("'"));
            break;
    }
}

// ============================================================================
// STATUS DISPLAY
// ============================================================================

void printStatus() {
    Serial.println(F("----------------------------------------"));
    Serial.println(F("Stepper Status:"));

    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        StepperMotor* stepper = StepperManager::getStepper(i);
        if (!stepper) continue;

        Serial.print(F("  Stepper "));
        Serial.print(i);
        Serial.print(F(": "));

        if (!stepper->isEnabled()) {
            Serial.println(F("DISABLED"));
            continue;
        }

        Serial.print(F("EN "));

        // Print state
        StepperState state = stepper->getState();
        switch (state) {
            case STEPPER_IDLE:
                Serial.print(F("[IDLE]    "));
                break;
            case STEPPER_ACCEL:
                Serial.print(F("[ACCEL]   "));
                break;
            case STEPPER_CRUISE:
                Serial.print(F("[CRUISE]  "));
                break;
            case STEPPER_DECEL:
                Serial.print(F("[DECEL]   "));
                break;
            case STEPPER_HOMING:
                Serial.print(F("[HOMING]  "));
                break;
            case STEPPER_FAULT:
                Serial.print(F("[FAULT]   "));
                break;
            default:
                Serial.print(F("[UNKNOWN] "));
                break;
        }

        // Print position
        Serial.print(F("Pos: "));
        Serial.print(stepper->getPosition());

        if (stepper->isMoving()) {
            Serial.print(F(" → "));
            Serial.print(stepper->getTargetPosition());
            Serial.print(F(" ("));
            Serial.print(stepper->getStepsRemaining());
            Serial.print(F(" left)"));
        }

        Serial.print(F("  V: "));
        Serial.print(stepper->getMaxVelocity());
        Serial.print(F(" A: "));
        Serial.print(stepper->getAcceleration());

        Serial.println();
    }
    Serial.println(F("----------------------------------------"));
}

void printHelp() {
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  Stepper Motor Test Commands"));
    Serial.println(F("========================================"));
    Serial.println(F("Control:"));
    Serial.println(F("  e<id>          - Enable stepper (id: 0-3)"));
    Serial.println(F("  d<id>          - Disable stepper"));
    Serial.println(F("  m<id>,<steps>  - Move relative steps"));
    Serial.println(F("  p<id>,<pos>    - Move to absolute position"));
    Serial.println(F("  v<id>,<vel>    - Set max velocity (steps/sec)"));
    Serial.println(F("  a<id>,<accel>  - Set acceleration (steps/sec²)"));
    Serial.println(F("  h<id>,<dir>    - Home stepper (dir: 1 or -1)"));
    Serial.println(F("  s<id>          - Emergency stop stepper"));
    Serial.println(F("  S              - Stop all steppers"));
    Serial.println();
    Serial.println(F("Info:"));
    Serial.println(F("  ?              - Print status (auto every 2s)"));
    Serial.println(F("  help           - Show this help"));
    Serial.println();
    Serial.println(F("Examples:"));
    Serial.println(F("  e0             - Enable stepper 0"));
    Serial.println(F("  v0,500         - Set stepper 0 to 500 steps/sec"));
    Serial.println(F("  a0,300         - Set stepper 0 to 300 steps/sec²"));
    Serial.println(F("  m0,200         - Move stepper 0 forward 200 steps"));
    Serial.println(F("  m0,-200        - Move stepper 0 backward 200 steps"));
    Serial.println(F("  p0,0           - Return stepper 0 to position 0"));
    Serial.println(F("  h0,1           - Home stepper 0 (positive direction)"));
    Serial.println(F("========================================"));
    Serial.println();
}
