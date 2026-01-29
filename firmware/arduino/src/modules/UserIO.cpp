/**
 * @file UserIO.cpp
 * @brief Implementation of user I/O management
 */

#include "UserIO.h"
#include "../pins.h"

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

NeoPixelDriver UserIO::neopixel_;

UserIO::LEDState UserIO::leds_[LED_COUNT];

uint16_t UserIO::buttonStates_ = 0;
uint16_t UserIO::prevButtonStates_ = 0;
uint8_t UserIO::limitStates_ = 0;

bool UserIO::initialized_ = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

void UserIO::init() {
    if (initialized_) return;

    // Initialize buttons (INPUT_PULLUP for active-low buttons)
    for (uint8_t i = 0; i < 10; i++) {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }

    // Initialize LEDs
    leds_[LED_RED].pin = PIN_LED_RED;
    leds_[LED_RED].mode = LED_OFF;
    pinMode(PIN_LED_RED, OUTPUT);
    digitalWrite(PIN_LED_RED, LOW);

    leds_[LED_GREEN].pin = PIN_LED_GREEN;
    leds_[LED_GREEN].mode = LED_OFF;
    pinMode(PIN_LED_GREEN, OUTPUT);
    digitalWrite(PIN_LED_GREEN, LOW);

    leds_[LED_BLUE].pin = PIN_LED_BLUE;
    leds_[LED_BLUE].mode = LED_OFF;
    pinMode(PIN_LED_BLUE, OUTPUT);
    digitalWrite(PIN_LED_BLUE, LOW);

    leds_[LED_ORANGE].pin = PIN_LED_ORANGE;
    leds_[LED_ORANGE].mode = LED_OFF;
    pinMode(PIN_LED_ORANGE, OUTPUT);
    digitalWrite(PIN_LED_ORANGE, LOW);

    leds_[LED_PURPLE].pin = PIN_LED_PURPLE;
    leds_[LED_PURPLE].mode = LED_OFF;
    pinMode(PIN_LED_PURPLE, OUTPUT);
    digitalWrite(PIN_LED_PURPLE, LOW);

    // Initialize all LED states
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        leds_[i].brightness = 255;
        leds_[i].periodMs = 1000;
        leds_[i].lastToggle = 0;
        leds_[i].state = false;
        leds_[i].breathePhase = 0;
    }

    // Initialize NeoPixel
    neopixel_.init(PIN_NEOPIXEL, 1, 128);  // 1 LED, 50% brightness
    neopixel_.setPixel(0, STATUS_IDLE);    // Start with blue (idle)
    neopixel_.show();

    // Read initial button states
    updateButtons();
    updateLimitSwitches();

    initialized_ = true;

#ifdef DEBUG_USERIO
    DEBUG_SERIAL.println(F("[UserIO] Initialized"));
    DEBUG_SERIAL.println(F("  - 10 buttons configured"));
    DEBUG_SERIAL.println(F("  - 5 LEDs configured"));
    DEBUG_SERIAL.println(F("  - NeoPixel initialized"));
#endif
}

// ============================================================================
// UPDATE
// ============================================================================

void UserIO::update() {
    if (!initialized_) return;

    updateButtons();
    updateLimitSwitches();
    updateLEDs();
}

// ============================================================================
// BUTTON INTERFACE
// ============================================================================

bool UserIO::isButtonPressed(uint8_t buttonId) {
    if (buttonId >= 10) return false;

    return (buttonStates_ & (1 << buttonId)) != 0;
}

uint16_t UserIO::getButtonStates() {
    return buttonStates_;
}

bool UserIO::wasButtonPressed(uint8_t buttonId) {
    if (buttonId >= 10) return false;

    bool current = (buttonStates_ & (1 << buttonId)) != 0;
    bool previous = (prevButtonStates_ & (1 << buttonId)) != 0;

    return (current && !previous);  // Rising edge (released to pressed)
}

// ============================================================================
// LIMIT SWITCH INTERFACE
// ============================================================================

bool UserIO::isLimitTriggered(uint8_t limitId) {
    if (limitId >= 8) return false;

    return (limitStates_ & (1 << limitId)) != 0;
}

uint8_t UserIO::getLimitStates() {
    return limitStates_;
}

// ============================================================================
// LED CONTROL
// ============================================================================

void UserIO::setLED(LEDId ledId, LEDMode mode, uint8_t brightness, uint16_t periodMs) {
    if (ledId >= LED_COUNT) return;

    LEDState& led = leds_[ledId];
    led.mode = mode;
    led.brightness = brightness;
    led.periodMs = periodMs;
    led.lastToggle = millis();
    led.breathePhase = 0;

    // Immediately apply state for OFF and ON modes
    if (mode == LED_OFF) {
        digitalWrite(led.pin, LOW);
        led.state = false;
    } else if (mode == LED_ON) {
        digitalWrite(led.pin, HIGH);
        led.state = true;
    } else if (mode == LED_PWM) {
        analogWrite(led.pin, brightness);
        led.state = true;
    }
}

void UserIO::setAllLEDsOff() {
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        setLED((LEDId)i, LED_OFF);
    }
}

// ============================================================================
// NEOPIXEL SYSTEM STATUS
// ============================================================================

void UserIO::setSystemStatus(uint32_t status) {
    neopixel_.setPixel(0, status);
    neopixel_.show();
}

void UserIO::setNeoPixelColor(uint32_t color) {
    neopixel_.setPixel(0, color);
    neopixel_.show();
}

void UserIO::setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b) {
    neopixel_.setPixel(0, r, g, b);
    neopixel_.show();
}

void UserIO::setNeoPixelBrightness(uint8_t brightness) {
    neopixel_.setBrightness(brightness);
    neopixel_.show();
}

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

void UserIO::updateButtons() {
    prevButtonStates_ = buttonStates_;
    buttonStates_ = 0;

    // Read all button states
    for (uint8_t i = 0; i < 10; i++) {
        // Buttons are active LOW (pressed = LOW)
        if (digitalRead(BUTTON_PINS[i]) == LOW) {
            buttonStates_ |= (1 << i);
        }
    }
}

void UserIO::updateLimitSwitches() {
    limitStates_ = 0;

    // Read all limit switch states
    for (uint8_t i = 0; i < 8; i++) {
        // Limit switches share pins with buttons 3-10
        uint8_t activeState = LIMIT_ACTIVE_LOW ? LOW : HIGH;
        if (digitalRead(LIMIT_PINS[i]) == activeState) {
            limitStates_ |= (1 << i);
        }
    }
}

void UserIO::updateLEDs() {
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        updateLED(leds_[i]);
    }
}

void UserIO::updateLED(LEDState& led) {
    uint32_t now = millis();

    switch (led.mode) {
        case LED_OFF:
            // Already handled in setLED()
            break;

        case LED_ON:
        case LED_PWM:
            // Already handled in setLED()
            break;

        case LED_BLINK:
            // Toggle LED at specified period
            if (now - led.lastToggle >= led.periodMs / 2) {
                led.lastToggle = now;
                led.state = !led.state;
                digitalWrite(led.pin, led.state ? HIGH : LOW);
            }
            break;

        case LED_BREATHE: {
            // Smooth breathing effect using sine wave approximation
            uint32_t elapsed = now - led.lastToggle;
            uint32_t phase = (elapsed * 255) / led.periodMs;

            if (phase >= 255) {
                led.lastToggle = now;
                phase = 0;
            }

            // Simple triangle wave for breathing
            uint8_t brightness;
            if (phase < 128) {
                brightness = phase * 2;  // Fade in
            } else {
                brightness = (255 - phase) * 2;  // Fade out
            }

            // Apply brightness scaling
            brightness = (brightness * led.brightness) / 255;

            // PIN_LED_PURPLE doesn't support PWM, so use ON/OFF only
            if (led.pin == PIN_LED_PURPLE) {
                digitalWrite(led.pin, (brightness > 128) ? HIGH : LOW);
            } else {
                analogWrite(led.pin, brightness);
            }
            break;
        }

        default:
            break;
    }
}
