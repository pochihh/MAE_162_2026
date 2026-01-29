/**
 * @file test_user_io.ino
 * @brief Test sketch for user I/O (Phase 5)
 *
 * This test validates:
 * - Button reading (10 buttons: BTN1-BTN10)
 * - LED control (5 LEDs: RED, GREEN, BLUE, ORANGE, PURPLE)
 * - LED modes (ON, OFF, BLINK, BREATHE, PWM)
 * - NeoPixel RGB LED (WS2812B)
 * - System status indication
 * - Debouncing and state tracking
 *
 * Hardware Requirements:
 * - Arduino Mega 2560
 * - 10 push buttons (BTN1-BTN10)
 * - 5 LEDs (Red, Green, Blue, Orange, Purple) with current-limiting resistors
 * - 1 NeoPixel WS2812B LED (optional)
 * - Connections as per pins.h
 *
 * Expected Behavior:
 * - Buttons register presses with debouncing
 * - LEDs respond to mode commands
 * - NeoPixel shows system status colors
 * - Blink and breathe modes animate smoothly
 * - No false triggers or bouncing
 *
 * Serial Commands:
 *   LED Control:
 *   r0/r1, g0/g1, b0/b1, o0/o1, p0/p1 - LED off/on (Red/Green/Blue/Orange/Purple)
 *   rb/gb/bb/ob/pb - Blink LED
 *   rt/gt/bt/ot/pt - Breathe LED (pulse)
 *
 *   NeoPixel:
 *   n<0-7>       - Set NeoPixel (0=off, 1=red, 2=green, 3=blue, 4=yellow, 5=cyan, 6=magenta, 7=white)
 *
 *   Status:
 *   s<0-3>       - Set system status (0=OK, 1=BUSY, 2=WARNING, 3=ERROR)
 *   ?            - Print button/LED status
 *   h            - Show help
 *
 * Interactive Test:
 * - Press BTN1-5: Toggles corresponding LED (1=RED, 2=GREEN, 3=BLUE, 4=ORANGE, 5=PURPLE)
 * - Press BTN6-9: Cycles NeoPixel through status colors
 * - Press BTN10: Turns all LEDs off
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/modules/UserIO.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

uint32_t lastStatusTime = 0;
const uint32_t STATUS_PERIOD_MS = 2000;  // Print status every 2 seconds

// Button state tracking for interactive test (10 buttons)
bool btnWasPressed[10] = {false};
uint8_t neoPixelColorIndex = 0;

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
    Serial.println(F("  User I/O Test - Phase 5"));
    Serial.println(F("  Buttons, LEDs, NeoPixel"));
    Serial.println(F("========================================"));
    Serial.println();

    // Initialize UserIO
    Serial.println(F("[Setup] Initializing User I/O..."));
    UserIO::init();
    Serial.println(F("  - Buttons initialized"));
    Serial.println(F("  - LEDs initialized"));
    Serial.println(F("  - NeoPixel initialized"));
    Serial.println();

    Serial.println(F("[Setup] Initialization complete!"));
    Serial.println();

    // Demo LED modes
    Serial.println(F("[Demo] Testing LED modes..."));

    // Test each LED color
    Serial.println(F("  - RED LED on"));
    UserIO::setLED(LED_RED, LED_ON, 255, 0);
    delay(500);
    UserIO::setLED(LED_RED, LED_OFF, 0, 0);

    Serial.println(F("  - GREEN LED on"));
    UserIO::setLED(LED_GREEN, LED_ON, 255, 0);
    delay(500);
    UserIO::setLED(LED_GREEN, LED_OFF, 0, 0);

    Serial.println(F("  - BLUE LED on"));
    UserIO::setLED(LED_BLUE, LED_ON, 255, 0);
    delay(500);
    UserIO::setLED(LED_BLUE, LED_OFF, 0, 0);

    Serial.println(F("  - ORANGE LED on"));
    UserIO::setLED(LED_ORANGE, LED_ON, 255, 0);
    delay(500);
    UserIO::setLED(LED_ORANGE, LED_OFF, 0, 0);

    Serial.println(F("  - PURPLE LED on"));
    UserIO::setLED(LED_PURPLE, LED_ON, 255, 0);
    delay(500);
    UserIO::setLED(LED_PURPLE, LED_OFF, 0, 0);

    // Test NeoPixel colors
    Serial.println(F("  - NeoPixel RED"));
    UserIO::setSystemStatus(STATUS_ERROR);
    UserIO::update();
    delay(500);

    Serial.println(F("  - NeoPixel GREEN"));
    UserIO::setSystemStatus(STATUS_OK);
    UserIO::update();
    delay(500);

    Serial.println(F("  - NeoPixel BLUE"));
    UserIO::setSystemStatus(STATUS_BUSY);
    UserIO::update();
    delay(500);

    Serial.println(F("  - NeoPixel YELLOW"));
    UserIO::setSystemStatus(STATUS_WARNING);
    UserIO::update();
    delay(500);

    Serial.println(F("  - NeoPixel OFF"));
    UserIO::setSystemStatus(STATUS_OK);
    UserIO::update();

    Serial.println();
    Serial.println(F("[Demo] LED test complete!"));
    Serial.println();
    Serial.println(F("Interactive Test:"));
    Serial.println(F("  - Press BTN1-5: Toggle LED (RED/GREEN/BLUE/ORANGE/PURPLE)"));
    Serial.println(F("  - Press BTN6-9: Cycle NeoPixel status colors"));
    Serial.println(F("  - Press BTN10: Turn all LEDs off"));
    Serial.println();
    Serial.println(F("Type 'h' for command help"));
    Serial.println(F("========================================"));
    Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Update UserIO (handles button debouncing, LED animations)
    UserIO::update();

    // Handle button presses for interactive test
    handleButtons();

    // Print status periodically
    if (millis() - lastStatusTime >= STATUS_PERIOD_MS) {
        lastStatusTime = millis();
        printStatus();
    }

    // Handle serial commands
    processSerialCommands();
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void handleButtons() {
    // Read all button states
    bool btnPressed[10];
    for (uint8_t i = 0; i < 10; i++) {
        btnPressed[i] = UserIO::isButtonPressed(i);
    }

    // BTN1-5: Toggle corresponding LED
    const LEDId ledMap[] = {LED_RED, LED_GREEN, LED_BLUE, LED_ORANGE, LED_PURPLE};
    const char* ledNames[] = {"RED", "GREEN", "BLUE", "ORANGE", "PURPLE"};
    static bool ledOn[5] = {false};

    for (uint8_t i = 0; i < 5; i++) {
        if (btnPressed[i] && !btnWasPressed[i]) {
            ledOn[i] = !ledOn[i];
            Serial.print(F("[BTN"));
            Serial.print(i + 1);
            Serial.print(F("] Pressed - Toggle "));
            Serial.print(ledNames[i]);
            Serial.println(F(" LED"));
            UserIO::setLED(ledMap[i], ledOn[i] ? LED_ON : LED_OFF, 255, 0);
        }
    }

    // BTN6-9: Cycle NeoPixel status colors
    for (uint8_t i = 5; i < 9; i++) {
        if (btnPressed[i] && !btnWasPressed[i]) {
            const uint32_t statusColors[] = {STATUS_OK, STATUS_BUSY, STATUS_WARNING, STATUS_ERROR};
            const char* colorNames[] = {"GREEN (OK)", "CYAN (BUSY)", "YELLOW (WARNING)", "RED (ERROR)"};

            neoPixelColorIndex = (neoPixelColorIndex + 1) % 4;

            Serial.print(F("[BTN"));
            Serial.print(i + 1);
            Serial.print(F("] Pressed - NeoPixel: "));
            Serial.println(colorNames[neoPixelColorIndex]);

            UserIO::setSystemStatus(statusColors[neoPixelColorIndex]);
        }
    }

    // BTN10: Turn all LEDs off
    if (btnPressed[9] && !btnWasPressed[9]) {
        Serial.println(F("[BTN10] Pressed - All LEDs OFF"));
        UserIO::setAllLEDsOff();
        for (uint8_t i = 0; i < 5; i++) {
            ledOn[i] = false;
        }
    }

    // Update previous button states
    for (uint8_t i = 0; i < 10; i++) {
        btnWasPressed[i] = btnPressed[i];
    }
}

// ============================================================================
// SERIAL COMMAND PROCESSING
// ============================================================================

void processSerialCommands() {
    if (Serial.available() == 0) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) return;

    // Parse command
    char cmdChar1 = cmd.charAt(0);
    char cmdChar2 = cmd.length() > 1 ? cmd.charAt(1) : '\0';

    // Help command
    if (cmdChar1 == 'h' || cmdChar1 == 'H' || cmdChar1 == '?') {
        printHelp();
        return;
    }

    // LED commands
    if (cmdChar1 == 'r' || cmdChar1 == 'R') {
        handleLEDCommand(LED_RED, "RED", cmdChar2);
    } else if (cmdChar1 == 'g' || cmdChar1 == 'G') {
        handleLEDCommand(LED_GREEN, "GREEN", cmdChar2);
    } else if (cmdChar1 == 'b' || cmdChar1 == 'B') {
        handleLEDCommand(LED_BLUE, "BLUE", cmdChar2);
    } else if (cmdChar1 == 'o' || cmdChar1 == 'O') {
        handleLEDCommand(LED_ORANGE, "ORANGE", cmdChar2);
    } else if (cmdChar1 == 'p' || cmdChar1 == 'P') {
        handleLEDCommand(LED_PURPLE, "PURPLE", cmdChar2);
    }
    // NeoPixel command
    else if (cmdChar1 == 'n' || cmdChar1 == 'N') {
        if (cmdChar2 >= '0' && cmdChar2 <= '7') {
            uint8_t color = cmdChar2 - '0';
            handleNeoPixelCommand(color);
        } else {
            Serial.println(F("ERROR: Format n<0-7>"));
        }
    }
    // System status command
    else if (cmdChar1 == 's' || cmdChar1 == 'S') {
        if (cmdChar2 >= '0' && cmdChar2 <= '3') {
            uint8_t statusIdx = cmdChar2 - '0';
            const uint32_t statusColors[] = {STATUS_OK, STATUS_BUSY, STATUS_WARNING, STATUS_ERROR};
            const char* statusNames[] = {"OK", "BUSY", "WARNING", "ERROR"};
            Serial.print(F(">>> System status: "));
            Serial.println(statusNames[statusIdx]);
            UserIO::setSystemStatus(statusColors[statusIdx]);
        } else {
            Serial.println(F("ERROR: Format s<0-3>"));
        }
    } else {
        Serial.print(F("ERROR: Unknown command '"));
        Serial.print(cmdChar1);
        Serial.println(F("'"));
    }
}

void handleLEDCommand(LEDId led, const char* name, char mode) {
    switch (mode) {
        case '0':  // Off
            Serial.print(F(">>> "));
            Serial.print(name);
            Serial.println(F(" LED: OFF"));
            UserIO::setLED(led, LED_OFF, 0, 0);
            break;

        case '1':  // On
            Serial.print(F(">>> "));
            Serial.print(name);
            Serial.println(F(" LED: ON"));
            UserIO::setLED(led, LED_ON, 255, 0);
            break;

        case 'b':  // Blink
        case 'B':
            Serial.print(F(">>> "));
            Serial.print(name);
            Serial.println(F(" LED: BLINK"));
            UserIO::setLED(led, LED_BLINK, 255, 500);
            break;

        case 't':  // Breathe (pulse)
        case 'T':
            Serial.print(F(">>> "));
            Serial.print(name);
            Serial.println(F(" LED: BREATHE"));
            UserIO::setLED(led, LED_BREATHE, 255, 2000);
            break;

        default:
            Serial.println(F("ERROR: Mode 0=off, 1=on, b=blink, t=breathe"));
            break;
    }
}

void handleNeoPixelCommand(uint8_t color) {
    const char* colorNames[] = {"OFF", "RED", "GREEN", "BLUE", "YELLOW", "CYAN", "MAGENTA", "WHITE"};

    Serial.print(F(">>> NeoPixel: "));
    Serial.println(colorNames[color]);

    // Map color index to RGB values
    switch (color) {
        case 0:  // Off
            UserIO::setNeoPixelColor(0, 0, 0);
            break;
        case 1:  // Red
            UserIO::setNeoPixelColor(255, 0, 0);
            break;
        case 2:  // Green
            UserIO::setNeoPixelColor(0, 255, 0);
            break;
        case 3:  // Blue
            UserIO::setNeoPixelColor(0, 0, 255);
            break;
        case 4:  // Yellow
            UserIO::setNeoPixelColor(255, 255, 0);
            break;
        case 5:  // Cyan
            UserIO::setNeoPixelColor(0, 255, 255);
            break;
        case 6:  // Magenta
            UserIO::setNeoPixelColor(255, 0, 255);
            break;
        case 7:  // White
            UserIO::setNeoPixelColor(255, 255, 255);
            break;
    }
}

// ============================================================================
// STATUS DISPLAY
// ============================================================================

void printStatus() {
    Serial.println(F("----------------------------------------"));
    Serial.println(F("User I/O Status:"));
    Serial.println();

    // Button states (all 10 buttons)
    Serial.println(F("  Buttons:"));
    uint16_t buttonStates = UserIO::getButtonStates();
    for (uint8_t i = 0; i < 10; i++) {
        Serial.print(F("    BTN"));
        if (i < 9) Serial.print(F(" "));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.println((buttonStates & (1 << i)) ? F("PRESSED") : F("Released"));
    }

    Serial.println();
    Serial.println(F("  LEDs: Use commands to control"));
    Serial.println(F("    r/g/b/o/p + 0/1/b/t"));
    Serial.println();
    Serial.println(F("  NeoPixel: Showing system status"));
    Serial.println(F("    n<0-7> or s<0-3>"));

    Serial.println(F("----------------------------------------"));
}

void printHelp() {
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("User I/O Test Commands:"));
    Serial.println(F("========================================"));
    Serial.println(F("LED Control (5 LEDs):"));
    Serial.println(F("  r0/r1 - RED LED off/on"));
    Serial.println(F("  g0/g1 - GREEN LED off/on"));
    Serial.println(F("  b0/b1 - BLUE LED off/on"));
    Serial.println(F("  o0/o1 - ORANGE LED off/on"));
    Serial.println(F("  p0/p1 - PURPLE LED off/on"));
    Serial.println();
    Serial.println(F("  rb/gb/bb/ob/pb - Blink LED"));
    Serial.println(F("  rt/gt/bt/ot/pt - Breathe LED"));
    Serial.println();
    Serial.println(F("NeoPixel:"));
    Serial.println(F("  n0 - OFF       n1 - RED"));
    Serial.println(F("  n2 - GREEN     n3 - BLUE"));
    Serial.println(F("  n4 - YELLOW    n5 - CYAN"));
    Serial.println(F("  n6 - MAGENTA   n7 - WHITE"));
    Serial.println();
    Serial.println(F("System Status:"));
    Serial.println(F("  s0 - OK (green)"));
    Serial.println(F("  s1 - BUSY (cyan)"));
    Serial.println(F("  s2 - WARNING (yellow)"));
    Serial.println(F("  s3 - ERROR (red)"));
    Serial.println();
    Serial.println(F("Info:"));
    Serial.println(F("  ?  - Print all button/LED status"));
    Serial.println(F("  h  - Show this help"));
    Serial.println();
    Serial.println(F("Interactive (10 Buttons):"));
    Serial.println(F("  BTN1-5:  Toggle LED (R/G/B/O/P)"));
    Serial.println(F("  BTN6-9:  Cycle NeoPixel colors"));
    Serial.println(F("  BTN10:   All LEDs off"));
    Serial.println(F("========================================"));
    Serial.println();
}
