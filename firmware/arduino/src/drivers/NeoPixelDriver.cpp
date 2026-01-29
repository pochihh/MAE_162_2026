/**
 * @file NeoPixelDriver.cpp
 * @brief Implementation of NeoPixel driver
 */

#include "NeoPixelDriver.h"
// #include "../../lib/Adafruit_NeoPixel.h"
#include <Adafruit_NeoPixel.h>

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

NeoPixelDriver::NeoPixelDriver()
    : strip_(nullptr)
    , initialized_(false)
{
}

NeoPixelDriver::~NeoPixelDriver() {
    if (strip_) {
        delete strip_;
        strip_ = nullptr;
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void NeoPixelDriver::init(uint8_t pin, uint16_t numPixels, uint8_t brightness) {
    if (initialized_) {
        return;
    }

    // Create NeoPixel instance
    // NEO_GRB + NEO_KHZ800: WS2812B standard timing and color order
    strip_ = new Adafruit_NeoPixel(numPixels, pin, NEO_GRB + NEO_KHZ800);

    if (!strip_) {
        return;
    }

    // Initialize strip
    strip_->begin();
    strip_->setBrightness(brightness);
    strip_->clear();
    strip_->show();  // Initialize all pixels to 'off'

    initialized_ = true;

#ifdef DEBUG_NEOPIXEL
    DEBUG_SERIAL.println(F("[NeoPixel] Initialized"));
    DEBUG_SERIAL.print(F("  - Pin: "));
    DEBUG_SERIAL.println(pin);
    DEBUG_SERIAL.print(F("  - Pixels: "));
    DEBUG_SERIAL.println(numPixels);
    DEBUG_SERIAL.print(F("  - Brightness: "));
    DEBUG_SERIAL.println(brightness);
#endif
}

// ============================================================================
// PIXEL CONTROL
// ============================================================================

void NeoPixelDriver::setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (!initialized_ || !strip_) return;
    if (index >= strip_->numPixels()) return;

    strip_->setPixelColor(index, r, g, b);
}

void NeoPixelDriver::setPixel(uint16_t index, uint32_t color) {
    if (!initialized_ || !strip_) return;
    if (index >= strip_->numPixels()) return;

    strip_->setPixelColor(index, color);
}

void NeoPixelDriver::setAll(uint8_t r, uint8_t g, uint8_t b) {
    if (!initialized_ || !strip_) return;

    uint32_t color = strip_->Color(r, g, b);
    for (uint16_t i = 0; i < strip_->numPixels(); i++) {
        strip_->setPixelColor(i, color);
    }
}

void NeoPixelDriver::setAll(uint32_t color) {
    if (!initialized_ || !strip_) return;

    for (uint16_t i = 0; i < strip_->numPixels(); i++) {
        strip_->setPixelColor(i, color);
    }
}

void NeoPixelDriver::clear() {
    if (!initialized_ || !strip_) return;

    strip_->clear();
}

void NeoPixelDriver::show() {
    if (!initialized_ || !strip_) return;

    strip_->show();
}

// ============================================================================
// BRIGHTNESS CONTROL
// ============================================================================

void NeoPixelDriver::setBrightness(uint8_t brightness) {
    if (!initialized_ || !strip_) return;

    strip_->setBrightness(brightness);
}

uint8_t NeoPixelDriver::getBrightness() const {
    if (!initialized_ || !strip_) return 0;

    return strip_->getBrightness();
}

uint16_t NeoPixelDriver::getNumPixels() const {
    if (!initialized_ || !strip_) return 0;

    return strip_->numPixels();
}

// ============================================================================
// COLOR UTILITIES
// ============================================================================

uint32_t NeoPixelDriver::colorHSV(uint16_t hue, uint8_t sat, uint8_t val) {
    // Use Adafruit's built-in HSV conversion
    return Adafruit_NeoPixel::ColorHSV(hue, sat, val);
}

uint32_t NeoPixelDriver::color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint32_t NeoPixelDriver::dimColor(uint32_t inputColor, uint8_t factor) {
    // Extract RGB components
    uint8_t r = (inputColor >> 16) & 0xFF;
    uint8_t g = (inputColor >> 8) & 0xFF;
    uint8_t b = inputColor & 0xFF;

    // Dim each component
    r = (r * factor) / 255;
    g = (g * factor) / 255;
    b = (b * factor) / 255;

    return NeoPixelDriver::color(r, g, b);
}
