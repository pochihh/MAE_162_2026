/**
 * @file UserIO.h
 * @brief User interface I/O management (buttons, LEDs, NeoPixels)
 *
 * This module manages all user-facing I/O:
 * - 10 user buttons (active low with internal pullup)
 * - 8 limit switches (shared pins with buttons 3-10)
 * - Status LEDs (red, green) and user LEDs (blue, orange, purple)
 * - NeoPixel RGB LED for system status indication
 *
 * LED Modes:
 * - OFF: LED is off
 * - ON: LED is constantly on at specified brightness
 * - PWM: LED is on with PWM dimming (0-255)
 * - BLINK: LED blinks at specified period
 * - BREATHE: LED fades in and out (breathing effect)
 *
 * NeoPixel System Status Colors:
 * - GREEN: System OK, ready
 * - BLUE: Idle, waiting for commands
 * - CYAN: Busy, processing
 * - YELLOW: Warning (low battery, sensor fault)
 * - RED: Error (communication timeout, critical fault)
 *
 * Usage:
 *   UserIO::init();
 *
 *   // In scheduler task @ 20Hz:
 *   UserIO::update();
 *
 *   // Query buttons:
 *   if (UserIO::isButtonPressed(0)) { ... }
 *
 *   // Set LED:
 *   UserIO::setLED(LED_RED, LED_MODE_BLINK, 255, 500);
 *
 *   // Set status:
 *   UserIO::setSystemStatus(STATUS_OK);
 */

#ifndef USERIO_H
#define USERIO_H

#include <Arduino.h>
#include <stdint.h>
#include "../drivers/NeoPixelDriver.h"
#include "../messages/TLV_Payloads.h"  // For LEDMode enum
#include "../config.h"

// LEDMode enum is defined in TLV_Payloads.h:
// LED_OFF, LED_ON, LED_PWM, LED_BLINK, LED_BREATHE

// ============================================================================
// LED IDENTIFIERS
// ============================================================================

enum LEDId {
    LED_RED = 0,        // Status LED red (error/low battery)
    LED_GREEN = 1,      // Status LED green (system OK)
    LED_BLUE = 2,       // User LED blue
    LED_ORANGE = 3,     // User LED orange
    LED_PURPLE = 4,     // User LED purple (non-PWM)
    LED_COUNT = 5       // Total number of LEDs
};

// ============================================================================
// USER I/O CLASS (Static)
// ============================================================================

/**
 * @brief User interface I/O manager
 *
 * Static class providing:
 * - Button and limit switch reading
 * - LED control with multiple modes
 * - NeoPixel system status indication
 */
class UserIO {
public:
    /**
     * @brief Initialize all user I/O
     *
     * Configures buttons, LEDs, and NeoPixel.
     * Must be called once in setup() before using I/O.
     */
    static void init();

    /**
     * @brief Update LED animations and button states
     *
     * Called from scheduler at 20Hz.
     * Updates LED patterns (blink, breathe) and reads button states.
     */
    static void update();

    // ========================================================================
    // BUTTON INTERFACE
    // ========================================================================

    /**
     * @brief Check if button is currently pressed
     *
     * Buttons are active low (pressed = LOW, released = HIGH).
     *
     * @param buttonId Button index (0-9)
     * @return True if button is pressed
     */
    static bool isButtonPressed(uint8_t buttonId);

    /**
     * @brief Get all button states as bitmask
     *
     * Bit 0 = button 0, bit 1 = button 1, etc.
     * Bit value 1 = pressed, 0 = released.
     *
     * @return 10-bit button state bitmask
     */
    static uint16_t getButtonStates();

    /**
     * @brief Check if button was just pressed (rising edge)
     *
     * Detects transition from released to pressed.
     *
     * @param buttonId Button index (0-9)
     * @return True if button was just pressed this update cycle
     */
    static bool wasButtonPressed(uint8_t buttonId);

    // ========================================================================
    // LIMIT SWITCH INTERFACE
    // ========================================================================

    /**
     * @brief Check if limit switch is triggered
     *
     * Limit switches share pins with buttons 3-10.
     * Active state depends on LIMIT_ACTIVE_LOW setting.
     *
     * @param limitId Limit switch index (0-7)
     * @return True if limit switch is triggered
     */
    static bool isLimitTriggered(uint8_t limitId);

    /**
     * @brief Get all limit switch states as bitmask
     *
     * Bit 0 = limit 0, bit 1 = limit 1, etc.
     * Bit value 1 = triggered, 0 = not triggered.
     *
     * @return 8-bit limit switch state bitmask
     */
    static uint8_t getLimitStates();

    // ========================================================================
    // LED CONTROL
    // ========================================================================

    /**
     * @brief Set LED mode and parameters
     *
     * @param ledId LED identifier (LED_RED, LED_GREEN, etc.)
     * @param mode LED mode (OFF, ON, PWM, BLINK, BREATHE)
     * @param brightness Brightness (0-255, only for ON/PWM modes)
     * @param periodMs Period in milliseconds (only for BLINK/BREATHE modes)
     */
    static void setLED(LEDId ledId, LEDMode mode, uint8_t brightness = 255, uint16_t periodMs = 1000);

    /**
     * @brief Turn off all LEDs
     */
    static void setAllLEDsOff();

    // ========================================================================
    // NEOPIXEL SYSTEM STATUS
    // ========================================================================

    /**
     * @brief Set system status color on NeoPixel
     *
     * Sets the NeoPixel to a predefined status color.
     *
     * @param status Status color (STATUS_OK, STATUS_WARNING, STATUS_ERROR, etc.)
     */
    static void setSystemStatus(uint32_t status);

    /**
     * @brief Set custom NeoPixel color
     *
     * @param color 32-bit RGB color (0x00RRGGBB)
     */
    static void setNeoPixelColor(uint32_t color);

    /**
     * @brief Set NeoPixel color from RGB components
     *
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     */
    static void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set NeoPixel brightness
     *
     * @param brightness Brightness (0-255)
     */
    static void setNeoPixelBrightness(uint8_t brightness);

private:
    // NeoPixel driver
    static NeoPixelDriver neopixel_;

    // LED state tracking
    struct LEDState {
        LEDMode mode;
        uint8_t pin;
        uint8_t brightness;
        uint16_t periodMs;
        uint32_t lastToggle;
        bool state;
        uint8_t breathePhase;   // For breathe mode (0-255)
    };
    static LEDState leds_[LED_COUNT];

    // Button state tracking
    static uint16_t buttonStates_;      // Current button states
    static uint16_t prevButtonStates_;  // Previous button states

    // Limit switch state tracking
    static uint8_t limitStates_;

    // Initialization flag
    static bool initialized_;

    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    /**
     * @brief Update LED animations
     */
    static void updateLEDs();

    /**
     * @brief Update button states
     */
    static void updateButtons();

    /**
     * @brief Update limit switch states
     */
    static void updateLimitSwitches();

    /**
     * @brief Update single LED based on mode
     *
     * @param led LED state structure
     */
    static void updateLED(LEDState& led);
};

#endif // USERIO_H
