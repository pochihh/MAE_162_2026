/**
 * @file NeoPixelDriver.h
 * @brief Abstraction layer for WS2812B addressable RGB LEDs
 *
 * This module provides a simplified interface for controlling WS2812B NeoPixel LEDs.
 * It wraps the underlying NeoPixel library (Adafruit_NeoPixel by default) to allow
 * easy swapping of implementations.
 *
 * Features:
 * - Individual pixel color control (R, G, B)
 * - Brightness control
 * - Color utilities (HSV to RGB conversion)
 * - Predefined color constants
 *
 * Hardware Connection:
 * - DATA: Digital output pin (defined in pins.h)
 * - VCC: 5V (3.3V logic level compatible with 5V LEDs)
 * - GND: Ground
 *
 * Timing Considerations:
 * - show() disables interrupts for ~30µs per LED
 * - Call show() only when needed (not every loop iteration)
 * - Avoid calling show() in interrupt context
 *
 * Usage:
 *   NeoPixelDriver neopixel;
 *   neopixel.init(PIN_NEOPIXEL, 1);  // 1 LED
 *   neopixel.setPixel(0, 255, 0, 0); // Red
 *   neopixel.show();
 */

#ifndef NEOPIXELDRIVER_H
#define NEOPIXELDRIVER_H

#include <Arduino.h>
#include <stdint.h>

// Forward declaration - we'll include the actual library in .cpp
class Adafruit_NeoPixel;

// ============================================================================
// COLOR CONSTANTS
// ============================================================================

// Predefined colors (R, G, B format)
const uint32_t COLOR_OFF       = 0x000000;
const uint32_t COLOR_RED        = 0xFF0000;
const uint32_t COLOR_GREEN      = 0x00FF00;
const uint32_t COLOR_BLUE       = 0x0000FF;
const uint32_t COLOR_YELLOW     = 0xFFFF00;
const uint32_t COLOR_CYAN       = 0x00FFFF;
const uint32_t COLOR_MAGENTA    = 0xFF00FF;
const uint32_t COLOR_WHITE      = 0xFFFFFF;
const uint32_t COLOR_ORANGE     = 0xFF8000;
const uint32_t COLOR_PURPLE     = 0x8000FF;

// System status colors
const uint32_t STATUS_OK        = COLOR_GREEN;
const uint32_t STATUS_WARNING   = COLOR_YELLOW;
const uint32_t STATUS_ERROR     = COLOR_RED;
const uint32_t STATUS_IDLE      = COLOR_BLUE;
const uint32_t STATUS_BUSY      = COLOR_CYAN;

// ============================================================================
// NEOPIXEL DRIVER CLASS
// ============================================================================

/**
 * @brief WS2812B NeoPixel driver
 *
 * Wraps Adafruit_NeoPixel library for easier use and
 * allows swapping to alternative libraries in the future.
 */
class NeoPixelDriver {
public:
    NeoPixelDriver();
    ~NeoPixelDriver();

    /**
     * @brief Initialize NeoPixel strip
     *
     * @param pin Data output pin
     * @param numPixels Number of LEDs in strip
     * @param brightness Initial brightness (0-255, default 128)
     */
    void init(uint8_t pin, uint16_t numPixels, uint8_t brightness = 128);

    /**
     * @brief Set individual pixel color
     *
     * Color is buffered. Call show() to update LEDs.
     *
     * @param index Pixel index (0 to numPixels-1)
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     */
    void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set individual pixel color from 32-bit value
     *
     * Color is buffered. Call show() to update LEDs.
     *
     * @param index Pixel index (0 to numPixels-1)
     * @param color 32-bit color (0x00RRGGBB format)
     */
    void setPixel(uint16_t index, uint32_t color);

    /**
     * @brief Set all pixels to the same color
     *
     * Color is buffered. Call show() to update LEDs.
     *
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     */
    void setAll(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set all pixels to the same color from 32-bit value
     *
     * Color is buffered. Call show() to update LEDs.
     *
     * @param color 32-bit color (0x00RRGGBB format)
     */
    void setAll(uint32_t color);

    /**
     * @brief Clear all pixels (set to off)
     *
     * Clears buffer. Call show() to update LEDs.
     */
    void clear();

    /**
     * @brief Update LEDs with buffered colors
     *
     * IMPORTANT: Disables interrupts for ~30µs per LED.
     * Do not call from interrupt context.
     */
    void show();

    /**
     * @brief Set brightness for all pixels
     *
     * @param brightness Brightness (0-255, 0=off, 255=max)
     */
    void setBrightness(uint8_t brightness);

    /**
     * @brief Get current brightness setting
     *
     * @return Brightness (0-255)
     */
    uint8_t getBrightness() const;

    /**
     * @brief Get number of pixels
     *
     * @return Number of pixels in strip
     */
    uint16_t getNumPixels() const;

    // ========================================================================
    // COLOR UTILITIES
    // ========================================================================

    /**
     * @brief Convert HSV to RGB color
     *
     * Useful for color animations and smooth color transitions.
     *
     * @param hue Hue (0-65535, wraps around)
     * @param sat Saturation (0-255, 0=white, 255=full color)
     * @param val Value/brightness (0-255, 0=black, 255=bright)
     * @return 32-bit RGB color (0x00RRGGBB)
     */
    static uint32_t colorHSV(uint16_t hue, uint8_t sat, uint8_t val);

    /**
     * @brief Create 32-bit color from R, G, B components
     *
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     * @return 32-bit color (0x00RRGGBB)
     */
    static uint32_t color(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Dim color by factor
     *
     * @param color Input color
     * @param factor Dimming factor (0-255, 0=black, 255=unchanged)
     * @return Dimmed color
     */
    static uint32_t dimColor(uint32_t color, uint8_t factor);

private:
    Adafruit_NeoPixel* strip_;  // Pointer to NeoPixel library instance
    bool initialized_;          // Initialization state
};

#endif // NEOPIXELDRIVER_H
