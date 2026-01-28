/**
 * @file TLV_Payloads.h
 * @brief Packed struct definitions for TLV message payloads
 *
 * All structs use #pragma pack(push, 1) for wire format compatibility.
 * These payloads are used for communication between Raspberry Pi and Arduino
 * via the TLV protocol over UART.
 *
 * IMPORTANT: Struct layout must match exactly on both sides (Arduino + Python/C++).
 * Do not add padding, virtual functions, or inheritance to these structs.
 */

#ifndef TLV_PAYLOADS_H
#define TLV_PAYLOADS_H

#include <stdint.h>

// Ensure structs are tightly packed with no padding
#pragma pack(push, 1)

// ============================================================================
// SYSTEM PAYLOADS
// ============================================================================

/**
 * @brief Heartbeat payload (RPi → Arduino)
 * TLV Type: SYS_HEARTBEAT (1)
 *
 * Sent periodically to indicate the RPi is alive. If not received within
 * HEARTBEAT_TIMEOUT_MS, all motors are disabled for safety.
 */
struct PayloadHeartbeat {
    uint32_t timestamp;     // RPi timestamp (milliseconds since boot)
    uint8_t  flags;         // Reserved flags (bit 0: emergency stop request)
};

/**
 * @brief System status payload (Arduino → RPi)
 * TLV Type: SYS_STATUS (2)
 *
 * Sent periodically or on request to report overall system health.
 */
struct PayloadSystemStatus {
    uint32_t uptimeMs;          // Arduino uptime in milliseconds
    uint32_t lastHeartbeatMs;   // Time since last heartbeat (ms)
    uint16_t batteryMv;         // Battery voltage (millivolts)
    uint16_t rail5vMv;          // 5V rail voltage (millivolts)
    uint16_t servoRailMv;       // Servo rail voltage (millivolts)
    uint8_t  errorFlags;        // Error bitmask (see ErrorFlags enum)
    uint8_t  motorEnableMask;   // Bitmask of enabled motors (bit N = motor N)
    uint8_t  stepperEnableMask; // Bitmask of enabled steppers
    uint8_t  reserved;          // Padding for alignment
};

/**
 * @brief Error flags bitmask for PayloadSystemStatus
 */
enum SystemErrorFlags {
    ERR_NONE            = 0x00,
    ERR_LOW_BATTERY     = 0x01,  // Battery below threshold
    ERR_HEARTBEAT_LOST  = 0x02,  // No heartbeat received (safety timeout)
    ERR_MOTOR_FAULT     = 0x04,  // Motor driver fault detected
    ERR_I2C_ERROR       = 0x08,  // I2C communication error
    ERR_IMU_ERROR       = 0x10,  // IMU not responding
    ERR_ENCODER_ERROR   = 0x20,  // Encoder fault detected
    ERR_THERMAL         = 0x40,  // Thermal shutdown warning
    ERR_WATCHDOG        = 0x80,  // Watchdog reset occurred
};

/**
 * @brief Set PID gains payload (RPi → Arduino)
 * TLV Type: SYS_SET_PID (3)
 *
 * Configure PID controller gains for a specific motor and loop type.
 */
struct PayloadSetPID {
    uint8_t motorId;        // Motor index (0-3 for DC, 0-3 for stepper)
    uint8_t motorType;      // 0 = DC motor, 1 = stepper
    uint8_t loopType;       // 0 = position, 1 = velocity, 2 = torque
    uint8_t reserved;       // Padding
    float   kp;             // Proportional gain
    float   ki;             // Integral gain
    float   kd;             // Derivative gain
    float   maxOutput;      // Output limit (e.g., 255 for PWM)
    float   maxIntegral;    // Anti-windup integral limit
};

/**
 * @brief Get PID gains payload (RPi → Arduino)
 * TLV Type: SYS_GET_PID (4)
 *
 * Request current PID gains for a specific motor and loop type.
 */
struct PayloadGetPID {
    uint8_t motorId;        // Motor index
    uint8_t motorType;      // 0 = DC motor, 1 = stepper
    uint8_t loopType;       // 0 = position, 1 = velocity, 2 = torque
    uint8_t reserved;       // Padding
};

/**
 * @brief PID gains response payload (Arduino → RPi)
 * TLV Type: SYS_RES_PID (5)
 *
 * Response to SYS_GET_PID request.
 */
struct PayloadResPID {
    uint8_t motorId;        // Motor index
    uint8_t motorType;      // 0 = DC motor, 1 = stepper
    uint8_t loopType;       // 0 = position, 1 = velocity, 2 = torque
    uint8_t reserved;       // Padding
    float   kp;             // Proportional gain
    float   ki;             // Integral gain
    float   kd;             // Derivative gain
    float   maxOutput;      // Output limit
    float   maxIntegral;    // Anti-windup integral limit
};

// ============================================================================
// DC MOTOR PAYLOADS
// ============================================================================

/**
 * @brief DC motor enable/disable payload (RPi → Arduino)
 * TLV Type: DC_ENABLE (256)
 */
struct PayloadDCEnable {
    uint8_t motorId;        // Motor index (0-3)
    uint8_t enable;         // 0 = disable, 1 = enable
    uint8_t mode;           // Control mode: 0 = disabled, 1 = position, 2 = velocity
    uint8_t reserved;       // Padding
};

/**
 * @brief DC motor control modes
 */
enum DCMotorMode {
    DC_MODE_DISABLED = 0,   // Motor disabled (coasts or brakes)
    DC_MODE_POSITION = 1,   // Position control (cascade PID)
    DC_MODE_VELOCITY = 2,   // Velocity control
    DC_MODE_PWM      = 3,   // Direct PWM control (open loop)
};

/**
 * @brief DC motor set position payload (RPi → Arduino)
 * TLV Type: DC_SET_POSITION (257)
 *
 * Command the motor to move to an absolute position.
 */
struct PayloadDCSetPosition {
    uint8_t  motorId;       // Motor index (0-3)
    uint8_t  reserved[3];   // Padding for alignment
    int32_t  targetPos;     // Target position (encoder ticks)
    int32_t  maxVelocity;   // Maximum velocity (ticks/sec), 0 = use default
};

/**
 * @brief DC motor set velocity payload (RPi → Arduino)
 * TLV Type: DC_SET_VELOCITY (258)
 *
 * Command the motor to maintain a target velocity.
 */
struct PayloadDCSetVelocity {
    uint8_t  motorId;       // Motor index (0-3)
    uint8_t  reserved[3];   // Padding for alignment
    int32_t  targetVel;     // Target velocity (ticks/sec)
    int32_t  maxAccel;      // Maximum acceleration (ticks/sec²), 0 = instant
};

/**
 * @brief DC motor status payload (Arduino → RPi)
 * TLV Type: DC_STATUS (259)
 *
 * Reports current state of a DC motor. Sent periodically at sensor update rate.
 */
struct PayloadDCStatus {
    uint8_t  motorId;       // Motor index (0-3)
    uint8_t  enabled;       // 0 = disabled, 1 = enabled
    uint8_t  mode;          // Current control mode (see DCMotorMode)
    uint8_t  faultFlags;    // Fault bitmask (bit 0: overcurrent, bit 1: stall)
    int32_t  position;      // Current position (encoder ticks)
    int32_t  velocity;      // Current velocity (ticks/sec)
    int32_t  targetPos;     // Target position (if in position mode)
    int32_t  targetVel;     // Target velocity
    int16_t  pwmOutput;     // Current PWM output (-255 to 255)
    int16_t  currentMa;     // Motor current (milliamps), -1 if not available
};

// ============================================================================
// STEPPER MOTOR PAYLOADS
// ============================================================================

/**
 * @brief Stepper motor enable/disable payload (RPi → Arduino)
 * TLV Type: STEP_ENABLE (512)
 */
struct PayloadStepEnable {
    uint8_t stepperId;      // Stepper index (0-3)
    uint8_t enable;         // 0 = disable (release), 1 = enable (hold)
    uint8_t reserved[2];    // Padding
};

/**
 * @brief Stepper motor set acceleration payload (RPi → Arduino)
 * TLV Type: STEP_SET_ACCEL (513)
 */
struct PayloadStepSetAccel {
    uint8_t  stepperId;     // Stepper index (0-3)
    uint8_t  reserved[3];   // Padding
    uint32_t accel;         // Acceleration (steps/sec²)
    uint32_t decel;         // Deceleration (steps/sec²), 0 = same as accel
};

/**
 * @brief Stepper motor set velocity payload (RPi → Arduino)
 * TLV Type: STEP_SET_VEL (514)
 */
struct PayloadStepSetVel {
    uint8_t  stepperId;     // Stepper index (0-3)
    uint8_t  reserved[3];   // Padding
    uint32_t maxVelocity;   // Maximum velocity (steps/sec)
};

/**
 * @brief Stepper motor move payload (RPi → Arduino)
 * TLV Type: STEP_MOVE (515)
 *
 * Move stepper to absolute position or by relative steps.
 */
struct PayloadStepMove {
    uint8_t  stepperId;     // Stepper index (0-3)
    uint8_t  moveType;      // 0 = absolute position, 1 = relative steps
    uint8_t  reserved[2];   // Padding
    int32_t  target;        // Target position (absolute) or steps (relative)
};

/**
 * @brief Stepper move types
 */
enum StepMoveType {
    STEP_MOVE_ABSOLUTE = 0, // Move to absolute position
    STEP_MOVE_RELATIVE = 1, // Move by relative steps
};

/**
 * @brief Stepper motor home payload (RPi → Arduino)
 * TLV Type: STEP_HOME (516)
 *
 * Move stepper until limit switch triggers, then zero position.
 */
struct PayloadStepHome {
    uint8_t  stepperId;     // Stepper index (0-3)
    int8_t   direction;     // -1 = reverse, +1 = forward
    uint8_t  reserved[2];   // Padding
    uint32_t homeVelocity;  // Velocity during homing (steps/sec)
    int32_t  backoffSteps;  // Steps to back off after hitting limit
};

/**
 * @brief Stepper motor status payload (Arduino → RPi)
 * TLV Type: STEP_STATUS (517)
 */
struct PayloadStepStatus {
    uint8_t  stepperId;     // Stepper index (0-3)
    uint8_t  enabled;       // 0 = disabled, 1 = enabled
    uint8_t  state;         // 0 = idle, 1 = moving, 2 = homing, 3 = fault
    uint8_t  limitHit;      // Bitmask: bit 0 = min limit, bit 1 = max limit
    int32_t  position;      // Current position (steps from home)
    int32_t  targetPos;     // Target position
    uint32_t currentVel;    // Current velocity (steps/sec)
    uint32_t stepsRemaining;// Steps remaining in current move
};

/**
 * @brief Stepper states
 */
enum StepperState {
    STEPPER_IDLE    = 0,    // Not moving
    STEPPER_ACCEL   = 1,    // Accelerating phase
    STEPPER_CRUISE  = 2,    // Constant velocity phase
    STEPPER_DECEL   = 3,    // Decelerating phase
    STEPPER_HOMING  = 4,    // Homing sequence (moving until limit switch)
    STEPPER_FAULT   = 5,    // Fault condition (e.g., stall detected)
};

// ============================================================================
// SERVO PAYLOADS
// ============================================================================

/**
 * @brief Servo controller enable/disable payload (RPi → Arduino)
 * TLV Type: SERVO_ENABLE (768)
 *
 * Enable or disable the PCA9685 servo controller output.
 */
struct PayloadServoEnable {
    uint8_t enable;         // 0 = disable (OE high), 1 = enable (OE low)
    uint8_t reserved[3];    // Padding
};

/**
 * @brief Servo set position payload (RPi → Arduino)
 * TLV Type: SERVO_SET (769)
 *
 * Set position for one or more servo channels.
 */
struct PayloadServoSet {
    uint8_t  channel;       // Servo channel (0-15), or 0xFF for bulk update
    uint8_t  count;         // Number of channels to update (1 for single, >1 for bulk)
    uint16_t pulseUs[1];    // Pulse width in microseconds (500-2500 typical)
                            // For bulk: array of count values starting at channel
};

/**
 * @brief Single servo channel command (simpler alternative)
 */
struct PayloadServoSetSingle {
    uint8_t  channel;       // Servo channel (0-15)
    uint8_t  reserved;      // Padding
    uint16_t pulseUs;       // Pulse width in microseconds (500-2500)
};

/**
 * @brief Bulk servo command for synchronized motion
 */
struct PayloadServoSetBulk {
    uint8_t  startChannel;  // First channel to update
    uint8_t  count;         // Number of consecutive channels
    uint16_t pulseUs[16];   // Pulse widths (only first 'count' values used)
};

// ============================================================================
// SENSOR PAYLOADS (Arduino → RPi)
// ============================================================================

/**
 * @brief Voltage sensor readings payload (Arduino → RPi)
 * TLV Type: SENSOR_VOLTAGE (1024)
 */
struct PayloadSensorVoltage {
    uint16_t batteryMv;     // Battery voltage (millivolts)
    uint16_t rail5vMv;      // 5V rail voltage (millivolts)
    uint16_t servoRailMv;   // Servo power rail voltage (millivolts)
    uint16_t reserved;      // Future use (e.g., motor driver Vcc)
};

/**
 * @brief Encoder readings payload (Arduino → RPi)
 * TLV Type: SENSOR_ENCODER (1025)
 *
 * Raw encoder data for all motors in one packet.
 */
struct PayloadSensorEncoder {
    int32_t  position[4];   // Encoder positions (ticks) for motors 0-3
    int32_t  velocity[4];   // Velocities (ticks/sec) for motors 0-3
    uint32_t timestamp;     // Arduino timestamp (micros) when sampled
};

/**
 * @brief Current sensor readings payload (Arduino → RPi)
 * TLV Type: SENSOR_CURRENT (1026)
 */
struct PayloadSensorCurrent {
    int16_t  motorCurrentMa[4]; // Motor currents in milliamps (-1 = not available)
    int16_t  totalCurrentMa;    // Total system current (if measured)
    uint16_t reserved;          // Padding
};

/**
 * @brief IMU sensor data payload (Arduino → RPi)
 * TLV Type: SENSOR_IMU (1027)
 *
 * 6-axis IMU data from ICM-20948.
 */
struct PayloadIMU {
    int16_t  accelX;        // Accelerometer X (raw value, LSB)
    int16_t  accelY;        // Accelerometer Y
    int16_t  accelZ;        // Accelerometer Z
    int16_t  gyroX;         // Gyroscope X (raw value, LSB)
    int16_t  gyroY;         // Gyroscope Y
    int16_t  gyroZ;         // Gyroscope Z
    int16_t  magX;          // Magnetometer X (raw value, LSB)
    int16_t  magY;          // Magnetometer Y
    int16_t  magZ;          // Magnetometer Z
    int16_t  temperature;   // Temperature (raw value, or °C × 100)
    uint32_t timestamp;     // Arduino timestamp (micros) when sampled
};

/**
 * @brief Range sensor data payload (Arduino → RPi)
 * TLV Type: SENSOR_RANGE (1028)
 */
struct PayloadSensorRange {
    uint8_t  sensorId;      // Sensor index
    uint8_t  status;        // 0 = valid, 1 = out of range, 2 = error
    uint16_t distanceMm;    // Distance in millimeters
    uint32_t timestamp;     // Arduino timestamp (micros)
};

// ============================================================================
// USER I/O PAYLOADS
// ============================================================================

/**
 * @brief Set LED state payload (RPi → Arduino)
 * TLV Type: IO_SET_LED (1280)
 */
struct PayloadSetLED {
    uint8_t  ledId;         // LED index (0 = status LED, 1-2 = user LEDs)
    uint8_t  mode;          // 0=off, 1=on, 2=blink, 3=breathe, 4=PWM
    uint8_t  brightness;    // Brightness (0-255) for PWM/breathe modes
    uint8_t  reserved;      // Padding
    uint16_t periodMs;      // Blink/breathe period in milliseconds
    uint16_t dutyCycle;     // Duty cycle for blink mode (0-1000 = 0-100.0%)
};

/**
 * @brief LED modes
 */
enum LEDMode {
    LED_OFF     = 0,        // LED off
    LED_ON      = 1,        // LED on (full brightness)
    LED_BLINK   = 2,        // Blinking (square wave)
    LED_BREATHE = 3,        // Breathing (sine wave fade)
    LED_PWM     = 4,        // Constant PWM (dimming)
};

/**
 * @brief Set NeoPixel color payload (RPi → Arduino)
 * TLV Type: IO_SET_NEOPIXEL (1281)
 */
struct PayloadSetNeoPixel {
    uint8_t index;          // Pixel index (0xFF = all pixels)
    uint8_t red;            // Red component (0-255)
    uint8_t green;          // Green component (0-255)
    uint8_t blue;           // Blue component (0-255)
};

/**
 * @brief Bulk NeoPixel update (for LED strips)
 */
struct PayloadSetNeoPixelBulk {
    uint8_t startIndex;     // First pixel to update
    uint8_t count;          // Number of pixels
    uint8_t reserved[2];    // Padding
    uint8_t colors[48];     // RGB triplets (max 16 pixels: 16 × 3 = 48 bytes)
};

/**
 * @brief Button state payload (Arduino → RPi)
 * TLV Type: IO_BUTTON_STATE (1282)
 */
struct PayloadButtonState {
    uint16_t buttonMask;    // Bitmask of pressed buttons (bit N = button N)
    uint16_t changedMask;   // Bitmask of buttons that changed since last report
    uint32_t timestamp;     // Arduino timestamp (millis)
};

/**
 * @brief Limit switch state payload (Arduino → RPi)
 * TLV Type: IO_LIMIT_STATE (1283)
 */
struct PayloadLimitState {
    uint8_t  limitMask;     // Bitmask of triggered limits (bit N = limit N)
    uint8_t  changedMask;   // Bitmask of limits that changed
    uint16_t reserved;      // Padding
    uint32_t timestamp;     // Arduino timestamp (millis)
};

// ============================================================================
// END OF PACKED STRUCTS
// ============================================================================

#pragma pack(pop)

// ============================================================================
// PAYLOAD SIZE VALIDATION
// ============================================================================

// Compile-time size checks to catch packing issues
#define STATIC_ASSERT_SIZE(type, expected) \
    static_assert(sizeof(type) == expected, #type " size mismatch")

// System payloads
STATIC_ASSERT_SIZE(PayloadHeartbeat, 5);
STATIC_ASSERT_SIZE(PayloadSystemStatus, 18);
STATIC_ASSERT_SIZE(PayloadSetPID, 24);
STATIC_ASSERT_SIZE(PayloadGetPID, 4);
STATIC_ASSERT_SIZE(PayloadResPID, 24);

// DC Motor payloads
STATIC_ASSERT_SIZE(PayloadDCEnable, 4);
STATIC_ASSERT_SIZE(PayloadDCSetPosition, 12);
STATIC_ASSERT_SIZE(PayloadDCSetVelocity, 12);
STATIC_ASSERT_SIZE(PayloadDCStatus, 24);

// Stepper payloads
STATIC_ASSERT_SIZE(PayloadStepEnable, 4);
STATIC_ASSERT_SIZE(PayloadStepSetAccel, 12);
STATIC_ASSERT_SIZE(PayloadStepSetVel, 8);
STATIC_ASSERT_SIZE(PayloadStepMove, 8);
STATIC_ASSERT_SIZE(PayloadStepHome, 12);
STATIC_ASSERT_SIZE(PayloadStepStatus, 20);

// Servo payloads
STATIC_ASSERT_SIZE(PayloadServoEnable, 4);
STATIC_ASSERT_SIZE(PayloadServoSetSingle, 4);
STATIC_ASSERT_SIZE(PayloadServoSetBulk, 34);

// Sensor payloads
STATIC_ASSERT_SIZE(PayloadSensorVoltage, 8);
STATIC_ASSERT_SIZE(PayloadSensorEncoder, 36);
STATIC_ASSERT_SIZE(PayloadSensorCurrent, 12);
STATIC_ASSERT_SIZE(PayloadIMU, 24);
STATIC_ASSERT_SIZE(PayloadSensorRange, 8);

// I/O payloads
STATIC_ASSERT_SIZE(PayloadSetLED, 8);
STATIC_ASSERT_SIZE(PayloadSetNeoPixel, 4);
STATIC_ASSERT_SIZE(PayloadSetNeoPixelBulk, 52);
STATIC_ASSERT_SIZE(PayloadButtonState, 8);
STATIC_ASSERT_SIZE(PayloadLimitState, 8);

#endif // TLV_PAYLOADS_H
