/**
 * @file VelocityEstimator.h
 * @brief Modular velocity estimation with swappable algorithms
 *
 * This module provides velocity estimation from encoder edges using different
 * algorithms optimized for various speed ranges:
 * - EdgeTimeVelocityEstimator: Best for low-medium speeds (default)
 * - PulseCountVelocityEstimator: Best for high speeds
 *
 * Data Flow:
 *   Encoder ISR              PID Loop (200Hz)
 *       │                         │
 *       ├─> Store timestamp  ────>├─> update()
 *       ├─> Increment count       ├─> Compute velocity
 *       └─> (exit ISR)            └─> getVelocity()
 *
 * Why compute in PID loop, not ISR:
 * - Keeps ISR minimal (~10 cycles vs ~50+ for velocity math)
 * - 200Hz update rate is adequate for control
 * - Floating point math in ISR would block other interrupts
 *
 * Usage:
 *   EdgeTimeVelocityEstimator velocityEstimator;
 *   velocityEstimator.init(720);  // 720 counts/rev for 2x mode
 *
 *   // In encoder ISR:
 *   encoder.onInterruptA();  // Encoder handles timestamp internally
 *
 *   // In PID loop @ 200Hz:
 *   velocityEstimator.update(encoder.getLastEdgeUs(), encoder.getCount());
 *   float velocity = velocityEstimator.getVelocity();  // ticks/sec
 */

#ifndef VELOCITYESTIMATOR_H
#define VELOCITYESTIMATOR_H

#include <Arduino.h>
#include <stdint.h>

// ============================================================================
// VELOCITY ESTIMATOR INTERFACE
// ============================================================================

/**
 * @brief Abstract interface for velocity estimation algorithms
 *
 * Allows swapping between different velocity computation methods without
 * changing application code.
 */
class IVelocityEstimator {
public:
    virtual ~IVelocityEstimator() {}

    /**
     * @brief Initialize velocity estimator
     *
     * @param countsPerRev Encoder counts per revolution (resolution dependent)
     */
    virtual void init(uint16_t countsPerRev) = 0;

    /**
     * @brief Update velocity estimate
     *
     * Called from PID loop at 200Hz. Computes velocity based on recent
     * encoder edge timestamps and position changes.
     *
     * @param currentUs Current time in microseconds (from micros())
     * @param currentCount Current encoder position
     */
    virtual void update(uint32_t currentUs, int32_t currentCount) = 0;

    /**
     * @brief Get estimated velocity
     *
     * @return Velocity in ticks/second (signed, positive = forward)
     */
    virtual float getVelocity() const = 0;

    /**
     * @brief Reset velocity estimator state
     */
    virtual void reset() = 0;
};

// ============================================================================
// EDGE-TIME VELOCITY ESTIMATOR (Default)
// ============================================================================

/**
 * @brief Velocity estimator using time between encoder edges
 *
 * Algorithm:
 * - Tracks timestamps of recent encoder edges
 * - Computes velocity from time between edges: v = 1 / Δt
 * - Uses moving average filter to smooth noisy measurements
 * - Zero-velocity detection via timeout (no edges for >50ms)
 *
 * Best for: Low to medium speeds (10-500 RPM)
 * Advantages: Smooth velocity estimate, good low-speed resolution
 * Disadvantages: Noisy at very high speeds, latency at very low speeds
 *
 * Configuration:
 * - Filter size: 2-8 samples (trade-off between smoothness and latency)
 * - Zero timeout: 50ms (configurable, depends on min expected speed)
 */
class EdgeTimeVelocityEstimator : public IVelocityEstimator {
public:
    EdgeTimeVelocityEstimator();

    void init(uint16_t countsPerRev) override;
    void update(uint32_t currentUs, int32_t currentCount) override;
    float getVelocity() const override;
    void reset() override;

    /**
     * @brief Set moving average filter size
     *
     * @param size Number of samples to average (2-8)
     */
    void setFilterSize(uint8_t size);

    /**
     * @brief Set zero-velocity timeout
     *
     * @param timeoutMs Time without edges before velocity = 0 (milliseconds)
     */
    void setZeroTimeout(uint16_t timeoutMs);

private:
    uint16_t countsPerRev_;          // Encoder counts per revolution
    int32_t prevCount_;              // Previous encoder count
    uint32_t prevEdgeUs_;            // Previous edge timestamp
    float velocity_;                 // Current velocity estimate (ticks/sec)
    uint16_t zeroTimeoutMs_;         // Zero-velocity timeout (milliseconds)

    // Moving average filter
    static const uint8_t MAX_FILTER_SIZE = 8;
    float filterBuffer_[MAX_FILTER_SIZE];
    uint8_t filterSize_;
    uint8_t filterIndex_;
    uint8_t filterCount_;

    /**
     * @brief Add sample to moving average filter
     *
     * @param sample Velocity sample to add
     */
    void addFilterSample(float sample);

    /**
     * @brief Get filtered velocity
     *
     * @return Average of samples in filter buffer
     */
    float getFilteredVelocity() const;
};

// ============================================================================
// PULSE-COUNT VELOCITY ESTIMATOR (Alternative)
// ============================================================================

/**
 * @brief Velocity estimator using pulse counting over fixed time window
 *
 * Algorithm:
 * - Counts encoder edges over fixed time window (e.g., 20ms)
 * - Computes velocity: v = ΔCount / Δt
 * - Updates at fixed rate (e.g., 50Hz)
 *
 * Best for: High speeds (>500 RPM)
 * Advantages: Simple, works well at high speeds, low CPU usage
 * Disadvantages: Poor low-speed resolution, quantization noise
 *
 * Configuration:
 * - Time window: 10-50ms (shorter = faster response, more noise)
 */
class PulseCountVelocityEstimator : public IVelocityEstimator {
public:
    PulseCountVelocityEstimator();

    void init(uint16_t countsPerRev) override;
    void update(uint32_t currentUs, int32_t currentCount) override;
    float getVelocity() const override;
    void reset() override;

    /**
     * @brief Set time window for pulse counting
     *
     * @param windowMs Time window in milliseconds (10-50ms typical)
     */
    void setTimeWindow(uint16_t windowMs);

private:
    uint16_t countsPerRev_;          // Encoder counts per revolution
    int32_t windowStartCount_;       // Count at start of time window
    uint32_t windowStartUs_;         // Timestamp at start of time window
    float velocity_;                 // Current velocity estimate (ticks/sec)
    uint32_t timeWindowUs_;          // Time window in microseconds
};

#endif // VELOCITYESTIMATOR_H
