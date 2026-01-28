/**
 * @file MessageCenter.cpp
 * @brief Implementation of central message handling for TLV protocol
 */

#include "MessageCenter.h"
#include "../config.h"

#include <string.h>  // Provides memcpy, memset needed by tlvcodec.c
extern "C" {
#include "../../lib/tlvcodec.c"
}

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

TlvDecodeDescriptor MessageCenter::decoder;
TlvEncodeDescriptor MessageCenter::encoder;
uint8_t MessageCenter::serialBuffer[MSG_SERIAL_BUFFER_SIZE];
size_t  MessageCenter::serialBufferIndex = 0;
uint8_t MessageCenter::pendingMessageCount = 0;
uint32_t MessageCenter::lastHeartbeatTime = 0;
bool MessageCenter::heartbeatReceived = false;

// Communication statistics
uint32_t MessageCenter::framesReceived = 0;
uint32_t MessageCenter::decodeErrors = 0;
uint32_t MessageCenter::crcErrors = 0;
uint32_t MessageCenter::messagesSent = 0;

// ============================================================================
// INITIALIZATION
// ============================================================================

void MessageCenter::init() {
    // Initialize UART (Serial2 @ 921600 baud, pins 16/17)
    RPI_SERIAL.begin(RPI_BAUD_RATE);

    // Initialize TLV decoder with callback
    initDecodeDescriptor(
        &decoder,
        MSG_TLV_BUFFER_SIZE,
        ENABLE_CRC_CHECK,  // CRC enabled or disabled
        MessageCenter::decodeCallback
    );

    // Initialize TLV encoder
    initEncodeDescriptor(
        &encoder,
        MSG_TLV_BUFFER_SIZE,
        DEVICE_ID,
        ENABLE_CRC_CHECK   // CRC enabled or disabled
    );

    serialBufferIndex = 0;
    pendingMessageCount = 0;
    lastHeartbeatTime = millis();
    heartbeatReceived = false;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.println(F("[MessageCenter] Initialized"));
    DEBUG_SERIAL.print(F("  - UART: Serial2 @ "));
    DEBUG_SERIAL.print(RPI_BAUD_RATE);
    DEBUG_SERIAL.println(F(" baud"));
    DEBUG_SERIAL.print(F("  - Device ID: 0x"));
    DEBUG_SERIAL.println(DEVICE_ID, HEX);
#endif
}

// ============================================================================
// PROCESSING TICK (called at 100Hz from scheduler)
// ============================================================================

void MessageCenter::processingTick() {
    // ========================================================================
    // 1. Read available bytes from Serial2
    // ========================================================================

    while (RPI_SERIAL.available() > 0) {
        uint8_t byte = RPI_SERIAL.read();
        serialBuffer[serialBufferIndex++] = byte;

        // Prevent buffer overflow
        if (serialBufferIndex >= MSG_SERIAL_BUFFER_SIZE) {
#ifdef DEBUG_TLV_PACKETS
            DEBUG_SERIAL.println(F("[MessageCenter] ERROR: Serial buffer overflow!"));
#endif
            serialBufferIndex = 0;  // Reset buffer
        }
    }

    // ========================================================================
    // 2. Decode TLV frames from buffer
    // ========================================================================

    if (serialBufferIndex > 0) {
        // Pass accumulated bytes to decoder
        // The decoder will call decodeCallback() when a complete frame is received
        decode(&decoder, serialBuffer, serialBufferIndex);
        serialBufferIndex = 0;  // Reset buffer after processing
    }

    // ========================================================================
    // 3. Send queued outgoing messages
    // ========================================================================

    if (pendingMessageCount > 0) {
        flushMessages();
    }
}

// ============================================================================
// HEARTBEAT VALIDATION
// ============================================================================

bool MessageCenter::isHeartbeatValid() {
    if (!heartbeatReceived) {
        return false;  // No heartbeat ever received
    }

    return (millis() - lastHeartbeatTime) < HEARTBEAT_TIMEOUT_MS;
}

uint32_t MessageCenter::timeSinceHeartbeat() {
    return millis() - lastHeartbeatTime;
}

// ============================================================================
// COMMUNICATION STATISTICS
// ============================================================================

uint32_t MessageCenter::getFramesReceived() {
    return framesReceived;
}

uint32_t MessageCenter::getDecodeErrors() {
    return decodeErrors;
}

uint32_t MessageCenter::getCRCErrors() {
    return crcErrors;
}

uint32_t MessageCenter::getMessagesSent() {
    return messagesSent;
}

void MessageCenter::resetStatistics() {
    framesReceived = 0;
    decodeErrors = 0;
    crcErrors = 0;
    messagesSent = 0;
}

// ============================================================================
// TLV DECODE CALLBACK (dispatches incoming messages)
// ============================================================================

void MessageCenter::decodeCallback(
    DecodeErrorCode *error,
    const FrameHeader *frameHeader,
    TlvHeader *tlvHeaders,
    uint8_t **tlvData
) {
    // Track decode statistics
    if (*error != NoError) {
        decodeErrors++;

        // Track CRC errors specifically
        if (*error == CrcError) {
            crcErrors++;
        }

#ifdef DEBUG_TLV_PACKETS
        DEBUG_SERIAL.print(F("[MessageCenter] Decode error: "));
        DEBUG_SERIAL.println(*error);
#endif
        return;
    }

    // Successfully decoded frame
    framesReceived++;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[MessageCenter] Frame received: "));
    DEBUG_SERIAL.print(frameHeader->numTlvs);
    DEBUG_SERIAL.println(F(" TLVs"));
#endif

    // Process each TLV in the frame
    for (size_t i = 0; i < frameHeader->numTlvs; i++) {
        uint32_t tlvType = tlvHeaders[i].tlvType;
        uint8_t *payload = tlvData[i];

#ifdef DEBUG_TLV_PACKETS
        DEBUG_SERIAL.print(F("  - TLV Type: "));
        DEBUG_SERIAL.print(tlvType);
        DEBUG_SERIAL.print(F(", Length: "));
        DEBUG_SERIAL.println(tlvHeaders[i].tlvLen);
#endif

        // Dispatch to appropriate handler based on TLV type
        switch (tlvType) {
            // ================================================================
            // SYSTEM COMMANDS
            // ================================================================
            case SYS_HEARTBEAT:
                handleHeartbeat((PayloadHeartbeat *)payload);
                break;

            case SYS_SET_PID:
                handleSetPID((PayloadSetPID *)payload);
                break;

            case SYS_GET_PID:
                handleGetPID((PayloadGetPID *)payload);
                break;

            // ================================================================
            // DC MOTOR COMMANDS
            // ================================================================
            case DC_ENABLE:
                handleDCEnable((PayloadDCEnable *)payload);
                break;

            case DC_SET_POSITION:
                handleDCSetPosition((PayloadDCSetPosition *)payload);
                break;

            case DC_SET_VELOCITY:
                handleDCSetVelocity((PayloadDCSetVelocity *)payload);
                break;

            // ================================================================
            // STEPPER COMMANDS
            // ================================================================
            case STEP_ENABLE:
                handleStepEnable((PayloadStepEnable *)payload);
                break;

            case STEP_SET_ACCEL:
                handleStepSetAccel((PayloadStepSetAccel *)payload);
                break;

            case STEP_SET_VEL:
                handleStepSetVel((PayloadStepSetVel *)payload);
                break;

            case STEP_MOVE:
                handleStepMove((PayloadStepMove *)payload);
                break;

            case STEP_HOME:
                handleStepHome((PayloadStepHome *)payload);
                break;

            // ================================================================
            // SERVO COMMANDS
            // ================================================================
            case SERVO_ENABLE:
                handleServoEnable((PayloadServoEnable *)payload);
                break;

            case SERVO_SET:
                handleServoSet((PayloadServoSetSingle *)payload);
                break;

            // ================================================================
            // USER I/O COMMANDS
            // ================================================================
            case IO_SET_LED:
                handleSetLED((PayloadSetLED *)payload);
                break;

            case IO_SET_NEOPIXEL:
                handleSetNeoPixel((PayloadSetNeoPixel *)payload);
                break;

            // ================================================================
            // UNKNOWN COMMAND
            // ================================================================
            default:
#ifdef DEBUG_TLV_PACKETS
                DEBUG_SERIAL.print(F("  - WARNING: Unknown TLV type "));
                DEBUG_SERIAL.println(tlvType);
#endif
                break;
        }
    }
}

// ============================================================================
// COMMAND HANDLERS (RPi → Arduino)
// ============================================================================

// ----------------------------------------------------------------------------
// System Commands
// ----------------------------------------------------------------------------

void MessageCenter::handleHeartbeat(const PayloadHeartbeat *payload) {
    lastHeartbeatTime = millis();
    heartbeatReceived = true;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[Heartbeat] timestamp="));
    DEBUG_SERIAL.print(payload->timestamp);
    DEBUG_SERIAL.print(F(", flags=0x"));
    DEBUG_SERIAL.println(payload->flags, HEX);
#endif

    // Check emergency stop flag (bit 0)
    if (payload->flags & 0x01) {
        DEBUG_SERIAL.println(F("[Heartbeat] EMERGENCY STOP REQUESTED!"));
        // TODO Phase 3: Disable all motors
    }
}

void MessageCenter::handleSetPID(const PayloadSetPID *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[SetPID] motor="));
    DEBUG_SERIAL.print(payload->motorId);
    DEBUG_SERIAL.print(F(", type="));
    DEBUG_SERIAL.print(payload->motorType);
    DEBUG_SERIAL.print(F(", loop="));
    DEBUG_SERIAL.print(payload->loopType);
    DEBUG_SERIAL.print(F(", Kp="));
    DEBUG_SERIAL.println(payload->kp);
#endif

    // TODO Phase 3: Apply PID gains to motor controller
}

void MessageCenter::handleGetPID(const PayloadGetPID *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[GetPID] motor="));
    DEBUG_SERIAL.print(payload->motorId);
    DEBUG_SERIAL.print(F(", type="));
    DEBUG_SERIAL.print(payload->motorType);
    DEBUG_SERIAL.print(F(", loop="));
    DEBUG_SERIAL.println(payload->loopType);
#endif

    // TODO Phase 3: Read PID gains from motor and send response
    // For now, send dummy response
    PayloadResPID response;
    response.motorId = payload->motorId;
    response.motorType = payload->motorType;
    response.loopType = payload->loopType;
    response.reserved = 0;
    response.kp = DEFAULT_VEL_KP;
    response.ki = DEFAULT_VEL_KI;
    response.kd = DEFAULT_VEL_KD;
    response.maxOutput = PID_OUTPUT_MAX;
    response.maxIntegral = 1000.0f;

    queueMessage(SYS_RES_PID, &response, sizeof(response));
}

// ----------------------------------------------------------------------------
// DC Motor Commands
// ----------------------------------------------------------------------------

void MessageCenter::handleDCEnable(const PayloadDCEnable *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[DCEnable] motor="));
    DEBUG_SERIAL.print(payload->motorId);
    DEBUG_SERIAL.print(F(", enable="));
    DEBUG_SERIAL.print(payload->enable);
    DEBUG_SERIAL.print(F(", mode="));
    DEBUG_SERIAL.println(payload->mode);
#endif

    // TODO Phase 3: Enable/disable DC motor
}

void MessageCenter::handleDCSetPosition(const PayloadDCSetPosition *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[DCSetPosition] motor="));
    DEBUG_SERIAL.print(payload->motorId);
    DEBUG_SERIAL.print(F(", target="));
    DEBUG_SERIAL.print(payload->targetPos);
    DEBUG_SERIAL.print(F(", maxVel="));
    DEBUG_SERIAL.println(payload->maxVelocity);
#endif

    // TODO Phase 3: Command motor to target position
}

void MessageCenter::handleDCSetVelocity(const PayloadDCSetVelocity *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[DCSetVelocity] motor="));
    DEBUG_SERIAL.print(payload->motorId);
    DEBUG_SERIAL.print(F(", target="));
    DEBUG_SERIAL.print(payload->targetVel);
    DEBUG_SERIAL.print(F(", maxAccel="));
    DEBUG_SERIAL.println(payload->maxAccel);
#endif

    // TODO Phase 3: Command motor to target velocity
}

// ----------------------------------------------------------------------------
// Stepper Commands
// ----------------------------------------------------------------------------

void MessageCenter::handleStepEnable(const PayloadStepEnable *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[StepEnable] stepper="));
    DEBUG_SERIAL.print(payload->stepperId);
    DEBUG_SERIAL.print(F(", enable="));
    DEBUG_SERIAL.println(payload->enable);
#endif

    // TODO Phase 4: Enable/disable stepper
}

void MessageCenter::handleStepSetAccel(const PayloadStepSetAccel *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[StepSetAccel] stepper="));
    DEBUG_SERIAL.print(payload->stepperId);
    DEBUG_SERIAL.print(F(", accel="));
    DEBUG_SERIAL.print(payload->accel);
    DEBUG_SERIAL.print(F(", decel="));
    DEBUG_SERIAL.println(payload->decel);
#endif

    // TODO Phase 4: Set stepper acceleration
}

void MessageCenter::handleStepSetVel(const PayloadStepSetVel *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[StepSetVel] stepper="));
    DEBUG_SERIAL.print(payload->stepperId);
    DEBUG_SERIAL.print(F(", maxVel="));
    DEBUG_SERIAL.println(payload->maxVelocity);
#endif

    // TODO Phase 4: Set stepper max velocity
}

void MessageCenter::handleStepMove(const PayloadStepMove *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[StepMove] stepper="));
    DEBUG_SERIAL.print(payload->stepperId);
    DEBUG_SERIAL.print(F(", type="));
    DEBUG_SERIAL.print(payload->moveType);
    DEBUG_SERIAL.print(F(", target="));
    DEBUG_SERIAL.println(payload->target);
#endif

    // TODO Phase 4: Move stepper to target
}

void MessageCenter::handleStepHome(const PayloadStepHome *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[StepHome] stepper="));
    DEBUG_SERIAL.print(payload->stepperId);
    DEBUG_SERIAL.print(F(", dir="));
    DEBUG_SERIAL.print(payload->direction);
    DEBUG_SERIAL.print(F(", vel="));
    DEBUG_SERIAL.println(payload->homeVelocity);
#endif

    // TODO Phase 4: Home stepper to limit switch
}

// ----------------------------------------------------------------------------
// Servo Commands
// ----------------------------------------------------------------------------

void MessageCenter::handleServoEnable(const PayloadServoEnable *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[ServoEnable] enable="));
    DEBUG_SERIAL.println(payload->enable);
#endif

    // TODO Phase 4: Enable/disable PCA9685
}

void MessageCenter::handleServoSet(const PayloadServoSetSingle *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[ServoSet] channel="));
    DEBUG_SERIAL.print(payload->channel);
    DEBUG_SERIAL.print(F(", pulse="));
    DEBUG_SERIAL.print(payload->pulseUs);
    DEBUG_SERIAL.println(F("µs"));
#endif

    // TODO Phase 4: Set servo pulse width
}

// ----------------------------------------------------------------------------
// User I/O Commands
// ----------------------------------------------------------------------------

void MessageCenter::handleSetLED(const PayloadSetLED *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[SetLED] id="));
    DEBUG_SERIAL.print(payload->ledId);
    DEBUG_SERIAL.print(F(", mode="));
    DEBUG_SERIAL.print(payload->mode);
    DEBUG_SERIAL.print(F(", brightness="));
    DEBUG_SERIAL.println(payload->brightness);
#endif

    // TODO Phase 5: Set LED mode
}

void MessageCenter::handleSetNeoPixel(const PayloadSetNeoPixel *payload) {
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[SetNeoPixel] index="));
    DEBUG_SERIAL.print(payload->index);
    DEBUG_SERIAL.print(F(", RGB=("));
    DEBUG_SERIAL.print(payload->red);
    DEBUG_SERIAL.print(F(","));
    DEBUG_SERIAL.print(payload->green);
    DEBUG_SERIAL.print(F(","));
    DEBUG_SERIAL.print(payload->blue);
    DEBUG_SERIAL.println(F(")"));
#endif

    // TODO Phase 5: Set NeoPixel color
}

// ============================================================================
// OUTGOING MESSAGE QUEUE (Arduino → RPi)
// ============================================================================

void MessageCenter::queueMessage(uint32_t tlvType, const void *payload, uint32_t payloadSize) {
    addTlvPacket(&encoder, tlvType, payloadSize, payload);
    pendingMessageCount++;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[MessageCenter] Queued TLV type "));
    DEBUG_SERIAL.print(tlvType);
    DEBUG_SERIAL.print(F(", size "));
    DEBUG_SERIAL.println(payloadSize);
#endif

    // Flush immediately if queue is full
    if (pendingMessageCount >= MSG_MAX_PENDING_MSGS) {
        flushMessages();
    }
}

void MessageCenter::flushMessages() {
    if (pendingMessageCount == 0) {
        return;  // Nothing to send
    }

    // Wrap up TLV frame
    int bufferSize = wrapupBuffer(&encoder);

    // Send via Serial2
    RPI_SERIAL.write(encoder.buffer, bufferSize);

    // Track messages sent
    messagesSent++;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[MessageCenter] Sent frame: "));
    DEBUG_SERIAL.print(pendingMessageCount);
    DEBUG_SERIAL.print(F(" TLVs, "));
    DEBUG_SERIAL.print(bufferSize);
    DEBUG_SERIAL.println(F(" bytes"));
#endif

    // Reset encoder for next frame
    resetDescriptor(&encoder);
    pendingMessageCount = 0;
}

// ============================================================================
// SEND FUNCTIONS (Arduino → RPi)
// ============================================================================

void MessageCenter::sendSystemStatus() {
    PayloadSystemStatus status;
    status.uptimeMs = millis();
    status.lastHeartbeatMs = timeSinceHeartbeat();
    status.batteryMv = 0;       // TODO Phase 5: Read from ADC
    status.rail5vMv = 0;        // TODO Phase 5: Read from ADC
    status.servoRailMv = 0;     // TODO Phase 5: Read from ADC
    status.errorFlags = ERR_NONE;  // TODO: Aggregate error flags
    status.motorEnableMask = 0;    // TODO Phase 3: Read from motor states
    status.stepperEnableMask = 0;  // TODO Phase 4: Read from stepper states
    status.reserved = 0;

    if (!isHeartbeatValid()) {
        status.errorFlags |= ERR_HEARTBEAT_LOST;
    }

    queueMessage(SYS_STATUS, &status, sizeof(status));
}

void MessageCenter::sendDCMotorStatus(uint8_t motorId) {
    // TODO Phase 3: Implement with real motor data
    PayloadDCStatus status;
    status.motorId = motorId;
    status.enabled = 0;
    status.mode = DC_MODE_DISABLED;
    status.faultFlags = 0;
    status.position = 0;
    status.velocity = 0;
    status.targetPos = 0;
    status.targetVel = 0;
    status.pwmOutput = 0;
    status.currentMa = -1;  // Not available

    queueMessage(DC_STATUS, &status, sizeof(status));
}

void MessageCenter::sendAllDCMotorStatus() {
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        sendDCMotorStatus(i);
    }
}

void MessageCenter::sendStepperStatus(uint8_t stepperId) {
    // TODO Phase 4: Implement with real stepper data
    PayloadStepStatus status;
    status.stepperId = stepperId;
    status.enabled = 0;
    status.state = STEPPER_IDLE;
    status.limitHit = 0;
    status.position = 0;
    status.targetPos = 0;
    status.currentVel = 0;
    status.stepsRemaining = 0;

    queueMessage(STEP_STATUS, &status, sizeof(status));
}

void MessageCenter::sendVoltageData() {
    // TODO Phase 5: Read from ADC
    PayloadSensorVoltage data;
    data.batteryMv = 12000;     // Placeholder: 12V
    data.rail5vMv = 5000;       // Placeholder: 5V
    data.servoRailMv = 6000;    // Placeholder: 6V
    data.reserved = 0;

    queueMessage(SENSOR_VOLTAGE, &data, sizeof(data));
}

void MessageCenter::sendEncoderData() {
    // TODO Phase 3: Read from encoders
    PayloadSensorEncoder data;
    for (uint8_t i = 0; i < 4; i++) {
        data.position[i] = 0;
        data.velocity[i] = 0;
    }
    data.timestamp = micros();

    queueMessage(SENSOR_ENCODER, &data, sizeof(data));
}

void MessageCenter::sendCurrentData() {
    // TODO Phase 3: Read from current sensors
    PayloadSensorCurrent data;
    for (uint8_t i = 0; i < 4; i++) {
        data.motorCurrentMa[i] = -1;  // Not available
    }
    data.totalCurrentMa = -1;
    data.reserved = 0;

    queueMessage(SENSOR_CURRENT, &data, sizeof(data));
}

void MessageCenter::sendIMUData() {
    // TODO Phase 5: Read from ICM-20948
    PayloadIMU data;
    data.accelX = 0;
    data.accelY = 0;
    data.accelZ = 0;
    data.gyroX = 0;
    data.gyroY = 0;
    data.gyroZ = 0;
    data.magX = 0;
    data.magY = 0;
    data.magZ = 0;
    data.temperature = 0;
    data.timestamp = micros();

    queueMessage(SENSOR_IMU, &data, sizeof(data));
}

void MessageCenter::sendRangeData(uint8_t sensorId, uint16_t distanceMm, uint8_t status) {
    PayloadSensorRange data;
    data.sensorId = sensorId;
    data.status = status;
    data.distanceMm = distanceMm;
    data.timestamp = micros();

    queueMessage(SENSOR_RANGE, &data, sizeof(data));
}

void MessageCenter::sendButtonState(uint16_t buttonMask, uint16_t changedMask) {
    PayloadButtonState data;
    data.buttonMask = buttonMask;
    data.changedMask = changedMask;
    data.timestamp = millis();

    queueMessage(IO_BUTTON_STATE, &data, sizeof(data));
}

void MessageCenter::sendLimitState(uint8_t limitMask, uint8_t changedMask) {
    PayloadLimitState data;
    data.limitMask = limitMask;
    data.changedMask = changedMask;
    data.reserved = 0;
    data.timestamp = millis();

    queueMessage(IO_LIMIT_STATE, &data, sizeof(data));
}
