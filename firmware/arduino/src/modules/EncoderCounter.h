/**
 * @file EncoderCounter.h
 * @brief Modular encoder counting with 2x and 4x resolution support
 *
 * This module provides interrupt-driven quadrature encoder counting with
 * swappable resolution modes:
 * - 2x mode: Interrupt on phase A only (current hardware)
 * - 4x mode: Interrupts on both phases (requires additional hardware INT pins)
 *
 * Hardware Requirements:
 * - Phase A pins MUST be on hardware interrupt pins (INT0-INT5)
 * - Phase B pins can be any digital pin (2x mode) or INT pins (4x mode)
 * - With 1440 PPR encoders:
 *   - 2x mode: 720 counts/rev (rising + falling on A)
 *   - 4x mode: 1440 counts/rev (all edges on A and B)
 *
 * Usage:
 *   EncoderCounter2x encoder;
 *   encoder.init(PIN_M1_ENC_A, PIN_M1_ENC_B);
 *   attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
 *
 *   // In ISR:
 *   void encoderISR_M1() { encoder.onInterruptA(); }
 *
 *   // In main code:
 *   int32_t position = encoder.getCount();
 */

#ifndef ENCODERCOUNTER_H
#define ENCODERCOUNTER_H

#include <Arduino.h>
#include <stdint.h>

// ============================================================================
// ENCODER MODE CONFIGURATION
// ============================================================================

#define ENCODER_2X  0   // 2x resolution (interrupt on phase A only)
#define ENCODER_4X  1   // 4x resolution (interrupts on both phases)

// ============================================================================
// ENCODER COUNTER INTERFACE
// ============================================================================

/**
 * @brief Abstract interface for encoder counting implementations
 *
 * Allows swapping between 2x and 4x resolution modes without changing
 * application code.
 */
class IEncoderCounter {
public:
    virtual ~IEncoderCounter() {}

    /**
     * @brief Initialize encoder pins
     *
     * @param pinA Phase A pin (must support hardware interrupts)
     * @param pinB Phase B pin
     * @param invertDir Direction inversion flag (false=normal, true=inverted)
     */
    virtual void init(uint8_t pinA, uint8_t pinB, bool invertDir = false) = 0;

    /**
     * @brief Get current encoder count
     *
     * @return Position in encoder ticks (signed)
     */
    virtual int32_t getCount() const = 0;

    /**
     * @brief Reset encoder count to zero
     */
    virtual void resetCount() = 0;

    /**
     * @brief Set encoder count to specific value
     *
     * @param count New count value
     */
    virtual void setCount(int32_t count) = 0;

    /**
     * @brief Phase A interrupt handler
     *
     * Called from ISR when phase A edge is detected.
     * MUST be fast (<10 CPU cycles).
     */
    virtual void onInterruptA() = 0;

    /**
     * @brief Phase B interrupt handler (4x mode only)
     *
     * Called from ISR when phase B edge is detected.
     * Not used in 2x mode.
     */
    virtual void onInterruptB() = 0;

    /**
     * @brief Get timestamp of last encoder edge
     *
     * Used by velocity estimator to compute edge timing.
     *
     * @return Timestamp in microseconds (from micros())
     */
    virtual uint32_t getLastEdgeUs() const = 0;

    /**
     * @brief Get resolution multiplier
     *
     * @return 2 for 2x mode, 4 for 4x mode
     */
    virtual uint8_t getResolutionMultiplier() const = 0;
};

// ============================================================================
// 2X RESOLUTION ENCODER (Current Hardware)
// ============================================================================

/**
 * @brief 2x resolution encoder counter
 *
 * Counts both rising and falling edges on phase A (CHANGE interrupt mode).
 * Reads phase B state to determine direction.
 *
 * Resolution: 2 edges per encoder pulse
 * - 1440 PPR encoder → 720 counts/rev
 *
 * ISR Time: ~10 CPU cycles (1 digitalRead, 1 increment, 1 timestamp capture)
 *
 * Hardware Requirements:
 * - Phase A on hardware INT pin (INT0-INT5)
 * - Phase B on any digital pin
 */
class EncoderCounter2x : public IEncoderCounter {
public:
    EncoderCounter2x();

    void init(uint8_t pinA, uint8_t pinB, bool invertDir = false) override;
    int32_t getCount() const override;
    void resetCount() override;
    void setCount(int32_t count) override;
    void onInterruptA() override;
    void onInterruptB() override;  // Not used in 2x mode
    uint32_t getLastEdgeUs() const override;
    uint8_t getResolutionMultiplier() const override;

private:
    volatile int32_t count_;         // Current encoder count
    volatile uint32_t lastEdgeUs_;   // Timestamp of last edge (micros())
    bool invertDir_;                 // Direction inversion flag
    uint8_t pinA_;                   // Phase A pin number
    uint8_t pinB_;                   // Phase B pin number
};

// ============================================================================
// 4X RESOLUTION ENCODER (Future Hardware)
// ============================================================================

/**
 * @brief 4x resolution encoder counter
 *
 * Counts all edges on both phase A and phase B using state machine decoding.
 * Requires both phases connected to hardware interrupt pins.
 *
 * Resolution: 4 edges per encoder pulse
 * - 1440 PPR encoder → 1440 counts/rev
 *
 * ISR Time: ~15-20 CPU cycles (state lookup, increment, timestamp)
 *
 * Hardware Requirements:
 * - Phase A on hardware INT pin
 * - Phase B on hardware INT pin
 *
 * State Machine:
 *   00 → 01: +1    01 → 11: +1    11 → 10: +1    10 → 00: +1  (forward)
 *   00 → 10: -1    10 → 11: -1    11 → 01: -1    01 → 00: -1  (reverse)
 */
class EncoderCounter4x : public IEncoderCounter {
public:
    EncoderCounter4x();

    void init(uint8_t pinA, uint8_t pinB, bool invertDir = false) override;
    int32_t getCount() const override;
    void resetCount() override;
    void setCount(int32_t count) override;
    void onInterruptA() override;
    void onInterruptB() override;
    uint32_t getLastEdgeUs() const override;
    uint8_t getResolutionMultiplier() const override;

private:
    volatile int32_t count_;         // Current encoder count
    volatile uint32_t lastEdgeUs_;   // Timestamp of last edge (micros())
    volatile uint8_t prevState_;     // Previous state (2 bits: A|B)
    bool invertDir_;                 // Direction inversion flag
    uint8_t pinA_;                   // Phase A pin number
    uint8_t pinB_;                   // Phase B pin number

    /**
     * @brief Common interrupt handler for both phases
     *
     * Called by both onInterruptA() and onInterruptB().
     */
    void processEdge();
};

#endif // ENCODERCOUNTER_H
