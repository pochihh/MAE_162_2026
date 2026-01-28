/**
 * @file VelocityEstimator.cpp
 * @brief Implementation of velocity estimation algorithms
 */

#include "VelocityEstimator.h"
#include <string.h>  // For memset

// ============================================================================
// EDGE-TIME VELOCITY ESTIMATOR IMPLEMENTATION
// ============================================================================

EdgeTimeVelocityEstimator::EdgeTimeVelocityEstimator()
    : countsPerRev_(0)
    , prevCount_(0)
    , prevEdgeUs_(0)
    , velocity_(0.0f)
    , zeroTimeoutMs_(50)
    , filterSize_(4)
    , filterIndex_(0)
    , filterCount_(0)
{
    memset(filterBuffer_, 0, sizeof(filterBuffer_));
}

void EdgeTimeVelocityEstimator::init(uint16_t countsPerRev) {
    countsPerRev_ = countsPerRev;
    reset();
}

void EdgeTimeVelocityEstimator::reset() {
    prevCount_ = 0;
    prevEdgeUs_ = micros();
    velocity_ = 0.0f;
    filterIndex_ = 0;
    filterCount_ = 0;
    memset(filterBuffer_, 0, sizeof(filterBuffer_));
}

void EdgeTimeVelocityEstimator::update(uint32_t currentUs, int32_t currentCount) {
    // Check for zero velocity timeout
    uint32_t timeSinceLastEdgeUs = currentUs - prevEdgeUs_;

    if (timeSinceLastEdgeUs > (zeroTimeoutMs_ * 1000UL)) {
        // No encoder edges for timeout period → velocity is zero
        velocity_ = 0.0f;
        addFilterSample(0.0f);
        return;
    }

    // Check if encoder count changed
    int32_t deltaCount = currentCount - prevCount_;

    if (deltaCount == 0) {
        // No movement since last update
        // Keep previous velocity estimate (don't add to filter yet)
        return;
    }

    // Compute time between edges
    uint32_t deltaTimeUs = currentUs - prevEdgeUs_;

    if (deltaTimeUs == 0) {
        // Avoid division by zero (shouldn't happen in practice)
        return;
    }

    // Compute instantaneous velocity: ticks/microsecond → ticks/second
    // velocity = deltaCount / (deltaTimeUs / 1000000)
    //          = (deltaCount * 1000000) / deltaTimeUs
    float instantVelocity = (float)deltaCount * 1000000.0f / (float)deltaTimeUs;

    // Add to moving average filter
    addFilterSample(instantVelocity);

    // Get filtered velocity
    velocity_ = getFilteredVelocity();

    // Update previous values for next iteration
    prevCount_ = currentCount;
    prevEdgeUs_ = currentUs;
}

float EdgeTimeVelocityEstimator::getVelocity() const {
    return velocity_;
}

void EdgeTimeVelocityEstimator::setFilterSize(uint8_t size) {
    if (size < 2) size = 2;
    if (size > MAX_FILTER_SIZE) size = MAX_FILTER_SIZE;

    filterSize_ = size;

    // Reset filter if size changed
    filterIndex_ = 0;
    filterCount_ = 0;
    memset(filterBuffer_, 0, sizeof(filterBuffer_));
}

void EdgeTimeVelocityEstimator::setZeroTimeout(uint16_t timeoutMs) {
    zeroTimeoutMs_ = timeoutMs;
}

void EdgeTimeVelocityEstimator::addFilterSample(float sample) {
    filterBuffer_[filterIndex_] = sample;
    filterIndex_ = (filterIndex_ + 1) % filterSize_;

    if (filterCount_ < filterSize_) {
        filterCount_++;
    }
}

float EdgeTimeVelocityEstimator::getFilteredVelocity() const {
    if (filterCount_ == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < filterCount_; i++) {
        sum += filterBuffer_[i];
    }

    return sum / (float)filterCount_;
}

// ============================================================================
// PULSE-COUNT VELOCITY ESTIMATOR IMPLEMENTATION
// ============================================================================

PulseCountVelocityEstimator::PulseCountVelocityEstimator()
    : countsPerRev_(0)
    , windowStartCount_(0)
    , windowStartUs_(0)
    , velocity_(0.0f)
    , timeWindowUs_(20000)  // Default 20ms window
{
}

void PulseCountVelocityEstimator::init(uint16_t countsPerRev) {
    countsPerRev_ = countsPerRev;
    reset();
}

void PulseCountVelocityEstimator::reset() {
    windowStartCount_ = 0;
    windowStartUs_ = micros();
    velocity_ = 0.0f;
}

void PulseCountVelocityEstimator::update(uint32_t currentUs, int32_t currentCount) {
    // Check if time window has elapsed
    uint32_t deltaTimeUs = currentUs - windowStartUs_;

    if (deltaTimeUs < timeWindowUs_) {
        // Still accumulating counts in current window
        return;
    }

    // Time window elapsed - compute velocity
    int32_t deltaCount = currentCount - windowStartCount_;

    if (deltaTimeUs == 0) {
        // Avoid division by zero
        velocity_ = 0.0f;
    } else {
        // Compute velocity: ticks/second
        velocity_ = (float)deltaCount * 1000000.0f / (float)deltaTimeUs;
    }

    // Start new window
    windowStartCount_ = currentCount;
    windowStartUs_ = currentUs;
}

float PulseCountVelocityEstimator::getVelocity() const {
    return velocity_;
}

void PulseCountVelocityEstimator::setTimeWindow(uint16_t windowMs) {
    timeWindowUs_ = (uint32_t)windowMs * 1000UL;
}
