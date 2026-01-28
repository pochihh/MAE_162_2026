/**
 * @file MessageCenter.h
 * @brief Central message handling for TLV protocol communication
 *
 * This module manages all UART communication between Arduino and Raspberry Pi
 * using the TLV (Type-Length-Value) protocol. It handles:
 * - Receiving and decoding incoming commands from RPi
 * - Dispatching commands to appropriate subsystems
 * - Queueing and sending outgoing sensor/status data
 * - Tracking heartbeat for safety timeout
 *
 * Communication: Serial2 @ 921600 baud (pins 16/17 with level shifter)
 *
 * Usage:
 *   MessageCenter::init();                       // Initialize UART and TLV codec
 *   // In scheduler task @ 100Hz:
 *   MessageCenter::processingTick();             // Process incoming/outgoing messages
 *
 *   // Queue outgoing data:
 *   MessageCenter::sendSystemStatus();
 *   MessageCenter::sendEncoderData();
 *   MessageCenter::sendVoltage Data();
 */

#ifndef MESSAGECENTER_H
#define MESSAGECENTER_H

#include <Arduino.h>
#include <stdint.h>

// C library for TLV codec
extern "C" {
#include "../../lib/tlvcodec.h"
}

#include "TLV_TypeDefs.h"
#include "TLV_Payloads.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define MSG_SERIAL_BUFFER_SIZE  2048    // Serial read buffer size
#define MSG_TLV_BUFFER_SIZE     1024    // TLV encode/decode buffer size
#define MSG_MAX_PENDING_MSGS    16      // Maximum queued outgoing messages

// ============================================================================
// MESSAGE CENTER CLASS
// ============================================================================

class MessageCenter {
public:
    /**
     * @brief Initialize UART communication and TLV codec
     *
     * Configures Serial2 @ 921600 baud and initializes encoder/decoder.
     * Must be called once in setup() before use.
     */
    static void init();

    /**
     * @brief Process incoming and outgoing messages
     *
     * Call this from scheduler task at 100Hz. It will:
     * - Read available bytes from Serial2
     * - Decode TLV frames and dispatch commands
     * - Send queued outgoing messages
     * - Check heartbeat timeout
     */
    static void processingTick();

    /**
     * @brief Check if heartbeat is valid (safety timeout)
     *
     * Returns true if heartbeat received within HEARTBEAT_TIMEOUT_MS.
     * If false, all motors should be disabled for safety.
     *
     * @return True if heartbeat valid, false if timeout
     */
    static bool isHeartbeatValid();

    /**
     * @brief Get time since last heartbeat (milliseconds)
     *
     * @return Milliseconds since last SYS_HEARTBEAT received
     */
    static uint32_t timeSinceHeartbeat();

    // ========================================================================
    // COMMUNICATION STATISTICS
    // ========================================================================

    /**
     * @brief Get total number of frames successfully received
     *
     * @return Total frames with no decode errors
     */
    static uint32_t getFramesReceived();

    /**
     * @brief Get total number of decode errors
     *
     * @return Total frames with any decode error
     */
    static uint32_t getDecodeErrors();

    /**
     * @brief Get total number of CRC errors
     *
     * @return Total frames with CRC mismatch
     */
    static uint32_t getCRCErrors();

    /**
     * @brief Get total number of messages sent
     *
     * @return Total TLV frames transmitted
     */
    static uint32_t getMessagesSent();

    /**
     * @brief Reset all communication statistics
     */
    static void resetStatistics();

    // ========================================================================
    // OUTGOING MESSAGE QUEUE (Arduino → RPi)
    // ========================================================================

    /**
     * @brief Send system status message
     *
     * Queues a SYS_STATUS message with uptime, voltages, error flags, etc.
     */
    static void sendSystemStatus();

    /**
     * @brief Send DC motor status message
     *
     * @param motorId Motor index (0-3)
     */
    static void sendDCMotorStatus(uint8_t motorId);

    /**
     * @brief Send all DC motor statuses
     *
     * Sends DC_STATUS for all enabled motors.
     */
    static void sendAllDCMotorStatus();

    /**
     * @brief Send stepper motor status message
     *
     * @param stepperId Stepper index (0-3)
     */
    static void sendStepperStatus(uint8_t stepperId);

    /**
     * @brief Send voltage sensor data
     *
     * Queues SENSOR_VOLTAGE message with battery, 5V, and servo rail voltages.
     */
    static void sendVoltageData();

    /**
     * @brief Send encoder data for all motors
     *
     * Queues SENSOR_ENCODER message with positions and velocities.
     */
    static void sendEncoderData();

    /**
     * @brief Send current sensor data
     *
     * Queues SENSOR_CURRENT message with motor currents.
     */
    static void sendCurrentData();

    /**
     * @brief Send IMU sensor data
     *
     * Queues SENSOR_IMU message with accel/gyro/mag readings.
     */
    static void sendIMUData();

    /**
     * @brief Send range sensor data
     *
     * @param sensorId Sensor index
     * @param distanceMm Distance in millimeters
     * @param status Sensor status (0=valid, 1=out of range, 2=error)
     */
    static void sendRangeData(uint8_t sensorId, uint16_t distanceMm, uint8_t status);

    /**
     * @brief Send button state
     *
     * @param buttonMask Bitmask of pressed buttons
     * @param changedMask Bitmask of changed buttons
     */
    static void sendButtonState(uint16_t buttonMask, uint16_t changedMask);

    /**
     * @brief Send limit switch state
     *
     * @param limitMask Bitmask of triggered limits
     * @param changedMask Bitmask of changed limits
     */
    static void sendLimitState(uint8_t limitMask, uint8_t changedMask);

private:
    // ========================================================================
    // TLV CODEC
    // ========================================================================

    static TlvDecodeDescriptor decoder;     // TLV decoder
    static TlvEncodeDescriptor encoder;     // TLV encoder

    static uint8_t serialBuffer[MSG_SERIAL_BUFFER_SIZE];    // Serial read buffer
    static size_t  serialBufferIndex;                       // Current index in buffer

    static uint8_t pendingMessageCount;     // Number of messages queued in encoder

    // ========================================================================
    // HEARTBEAT TRACKING
    // ========================================================================

    static uint32_t lastHeartbeatTime;      // millis() when last heartbeat received
    static bool     heartbeatReceived;      // Has any heartbeat been received?

    // ========================================================================
    // COMMUNICATION STATISTICS
    // ========================================================================

    static uint32_t framesReceived;         // Total frames successfully decoded
    static uint32_t decodeErrors;           // Total decode errors (any type)
    static uint32_t crcErrors;              // Total CRC errors specifically
    static uint32_t messagesSent;           // Total frames transmitted

    // ========================================================================
    // TLV DECODE CALLBACK (called by tlvcodec library)
    // ========================================================================

    /**
     * @brief TLV decode callback function
     *
     * Called by tlvcodec library when a complete frame is decoded.
     * Dispatches individual TLV messages to command handlers.
     *
     * @param error Error code (NoError if successful)
     * @param frameHeader Frame header information
     * @param tlvHeaders Array of TLV headers
     * @param tlvData Array of TLV payload pointers
     */
    static void decodeCallback(
        DecodeErrorCode *error,
        const FrameHeader *frameHeader,
        TlvHeader *tlvHeaders,
        uint8_t **tlvData
    );

    // ========================================================================
    // COMMAND HANDLERS (RPi → Arduino)
    // ========================================================================

    // System commands
    static void handleHeartbeat(const PayloadHeartbeat *payload);
    static void handleSetPID(const PayloadSetPID *payload);
    static void handleGetPID(const PayloadGetPID *payload);

    // DC motor commands
    static void handleDCEnable(const PayloadDCEnable *payload);
    static void handleDCSetPosition(const PayloadDCSetPosition *payload);
    static void handleDCSetVelocity(const PayloadDCSetVelocity *payload);

    // Stepper commands
    static void handleStepEnable(const PayloadStepEnable *payload);
    static void handleStepSetAccel(const PayloadStepSetAccel *payload);
    static void handleStepSetVel(const PayloadStepSetVel *payload);
    static void handleStepMove(const PayloadStepMove *payload);
    static void handleStepHome(const PayloadStepHome *payload);

    // Servo commands
    static void handleServoEnable(const PayloadServoEnable *payload);
    static void handleServoSet(const PayloadServoSetSingle *payload);

    // User I/O commands
    static void handleSetLED(const PayloadSetLED *payload);
    static void handleSetNeoPixel(const PayloadSetNeoPixel *payload);

    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    /**
     * @brief Add a message to the outgoing queue
     *
     * @param tlvType TLV type constant
     * @param payload Pointer to payload struct
     * @param payloadSize Size of payload in bytes
     */
    static void queueMessage(uint32_t tlvType, const void *payload, uint32_t payloadSize);

    /**
     * @brief Flush queued messages to Serial2
     *
     * Wraps up encoder buffer and sends via UART.
     */
    static void flushMessages();
};

#endif // MESSAGECENTER_H
