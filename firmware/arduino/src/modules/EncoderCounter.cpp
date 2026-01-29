/**
 * @file EncoderCounter.cpp
 * @brief Implementation of encoder counting module
 */

#include "EncoderCounter.h"

// ============================================================================
// 2X RESOLUTION ENCODER IMPLEMENTATION
// ============================================================================

EncoderCounter2x::EncoderCounter2x()
    : count_(0)
    , lastEdgeUs_(0)
    , invertDir_(false)
    , pinA_(0)
    , pinB_(0)
{
}

void EncoderCounter2x::init(uint8_t pinA, uint8_t pinB, bool invertDir) {
    pinA_ = pinA;
    pinB_ = pinB;
    invertDir_ = invertDir;

    // Configure pins
    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);

    // Reset state
    count_ = 0;
    lastEdgeUs_ = micros();
}

int32_t EncoderCounter2x::getCount() const {
    // Reading 32-bit volatile is atomic on AVR
    return count_;
}

void EncoderCounter2x::resetCount() {
    noInterrupts();
    count_ = 0;
    interrupts();
}

void EncoderCounter2x::setCount(int32_t count) {
    noInterrupts();
    count_ = count;
    interrupts();
}

void EncoderCounter2x::onInterruptA() {
    // CRITICAL: This runs in ISR context, must be FAST (<10 cycles)
    //
    // 2x Encoder Logic:
    // - Phase A changes (rising or falling edge)
    // - Read phase B to determine direction
    // - If A and B are different → forward (count++)
    // - If A and B are same → reverse (count--)
    //
    // Truth table for forward rotation (clockwise):
    //   A=0, B=1 → count++  (A rising when B high)
    //   A=1, B=0 → count++  (A falling when B low)
    //
    // Truth table for reverse rotation (counter-clockwise):
    //   A=0, B=0 → count--  (A falling when B low)
    //   A=1, B=1 → count--  (A rising when B high)

    uint8_t a = digitalRead(pinA_);
    uint8_t b = digitalRead(pinB_);

    // Determine direction: A^B gives forward direction
    bool forward = (a ^ b);

    // Apply direction with inversion flag
    count_ += invertDir_ ? (forward ? -1 : 1) : (forward ? 1 : -1);

    lastEdgeUs_ = micros();
}

void EncoderCounter2x::onInterruptB() {
    // Not used in 2x mode
}

uint32_t EncoderCounter2x::getLastEdgeUs() const {
    return lastEdgeUs_;
}

uint8_t EncoderCounter2x::getResolutionMultiplier() const {
    return 2;
}

// ============================================================================
// 4X RESOLUTION ENCODER IMPLEMENTATION
// ============================================================================

EncoderCounter4x::EncoderCounter4x()
    : count_(0)
    , lastEdgeUs_(0)
    , prevState_(0)
    , invertDir_(false)
    , pinA_(0)
    , pinB_(0)
{
}

void EncoderCounter4x::init(uint8_t pinA, uint8_t pinB, bool invertDir) {
    pinA_ = pinA;
    pinB_ = pinB;
    invertDir_ = invertDir;

    // Configure pins
    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);

    // Read initial state
    uint8_t a = digitalRead(pinA_);
    uint8_t b = digitalRead(pinB_);
    prevState_ = (a << 1) | b;

    // Reset counters
    count_ = 0;
    lastEdgeUs_ = micros();
}

int32_t EncoderCounter4x::getCount() const {
    return count_;
}

void EncoderCounter4x::resetCount() {
    noInterrupts();
    count_ = 0;
    interrupts();
}

void EncoderCounter4x::setCount(int32_t count) {
    noInterrupts();
    count_ = count;
    interrupts();
}

void EncoderCounter4x::onInterruptA() {
    processEdge();
}

void EncoderCounter4x::onInterruptB() {
    processEdge();
}

void EncoderCounter4x::processEdge() {
    // CRITICAL: ISR context, must be FAST (~15-20 cycles)
    //
    // 4x Quadrature Decoding State Machine:
    // - Tracks both phase A and phase B edges
    // - Previous state: prevState_ (2 bits: [A|B])
    // - Current state: (A << 1) | B
    //
    // State transitions for forward (CW) rotation:
    //   00 → 01 → 11 → 10 → 00 (count increments by 1 each transition)
    //
    // State transitions for reverse (CCW) rotation:
    //   00 → 10 → 11 → 01 → 00 (count decrements by 1 each transition)
    //
    // State transition lookup table:
    //   Encoding: (prevState << 2) | currState
    //   Valid transitions: 0=no change, +1=forward, -1=reverse

    uint8_t a = digitalRead(pinA_);
    uint8_t b = digitalRead(pinB_);
    uint8_t currState = (a << 1) | b;

    // Lookup table indexed by (prevState << 2) | currState
    // Values: 0=invalid/no-change, 1=forward, -1=reverse (stored as 255)
    static const int8_t stateTable[16] = {
        0,  // 00 → 00 (no change)
        1,  // 00 → 01 (forward)
        -1, // 00 → 10 (reverse)
        0,  // 00 → 11 (invalid)
        -1, // 01 → 00 (reverse)
        0,  // 01 → 01 (no change)
        0,  // 01 → 10 (invalid)
        1,  // 01 → 11 (forward)
        1,  // 10 → 00 (forward)
        0,  // 10 → 01 (invalid)
        0,  // 10 → 10 (no change)
        -1, // 10 → 11 (reverse)
        0,  // 11 → 00 (invalid)
        -1, // 11 → 01 (reverse)
        1,  // 11 → 10 (forward)
        0   // 11 → 11 (no change)
    };

    uint8_t index = (prevState_ << 2) | currState;
    int8_t delta = stateTable[index];

    // Apply direction with inversion flag
    count_ += invertDir_ ? -delta : delta;

    prevState_ = currState;
    lastEdgeUs_ = micros();
}

uint32_t EncoderCounter4x::getLastEdgeUs() const {
    return lastEdgeUs_;
}

uint8_t EncoderCounter4x::getResolutionMultiplier() const {
    return 4;
}
