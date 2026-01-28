/**
 * @file config.h
 * @brief Central configuration for Arduino Mega 2560 firmware
 *
 * This file contains all compile-time parameters for hardware configuration,
 * timing, communication, and debug settings.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// DC Motors
#define NUM_DC_MOTORS           4       // Total DC motor channels

#define DC_MOTOR_1_ENABLED      1       // Default wheel motor (left/right)
#define DC_MOTOR_2_ENABLED      1       // Default wheel motor (left/right)
#define DC_MOTOR_3_ENABLED      0       // Manipulator motor (optional)
#define DC_MOTOR_4_ENABLED      0       // Manipulator motor (optional)

// Stepper Motors
#define NUM_STEPPERS            4       // Total stepper channels

#define STEPPER_1_ENABLED       0
#define STEPPER_2_ENABLED       0
#define STEPPER_3_ENABLED       0
#define STEPPER_4_ENABLED       0

// Servos (via PCA9685)
#define NUM_SERVO_CHANNELS      16      // PCA9685 provides 16 channels

#define SERVO_CONTROLLER_ENABLED 0      // Enable PCA9685 servo driver

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================

#define ENCODER_PPR             1440    // Pulses per revolution (manufacturer spec)
#define ENCODER_MAX_RPM         100     // Maximum expected motor RPM

// Encoder resolution modes
#define ENCODER_2X              2       // 2x counting (phase A only)
#define ENCODER_4X              4       // 4x counting (both phases)

// Per-motor encoder mode (use ENCODER_2X or ENCODER_4X)
#define ENCODER_1_MODE          ENCODER_2X
#define ENCODER_2_MODE          ENCODER_2X
#define ENCODER_3_MODE          ENCODER_2X
#define ENCODER_4_MODE          ENCODER_2X

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

// Scheduler base tick (DO NOT CHANGE - Timer1 configured for 1kHz)
#define SCHEDULER_TICK_FREQ_HZ  1000    // 1ms period

// Task update frequencies
#define DC_PID_FREQ_HZ          200     // DC motor PID loop (5ms period)
#define UART_COMMS_FREQ_HZ      100     // UART communication (10ms period)
#define SENSOR_UPDATE_FREQ_HZ   50      // Sensor reading (20ms period)
#define USER_IO_FREQ_HZ         20      // LED/button update (50ms period)

// Stepper pulse generation (Timer3)
#define STEPPER_TIMER_FREQ_HZ   10000   // 10kHz interrupt rate (100µs period)
#define STEPPER_MAX_RATE_SPS    5000    // Maximum steps per second per motor

// Safety timeout
#define HEARTBEAT_TIMEOUT_MS    500      // Disable motors if no heartbeat

// ============================================================================
// COMMUNICATION SETTINGS
// ============================================================================

// UART to Raspberry Pi (Serial2)
#define RPI_BAUD_RATE           500000  // High-speed UART
#define RPI_SERIAL              Serial2 // Hardware serial port

// Debug serial (Serial0 - USB)
#define DEBUG_BAUD_RATE         115200
#define DEBUG_SERIAL            Serial

// Device identification
#define DEVICE_ID               0x01    // Arduino device ID for TLV protocol
#define ENABLE_CRC_CHECK      1       // Enable CRC checks on TLV packets

// ============================================================================
// VELOCITY ESTIMATION CONFIGURATION
// ============================================================================

// Velocity estimator settings
#define VELOCITY_FILTER_SIZE    4       // Moving average filter size (2-8 samples)
#define VELOCITY_ZERO_TIMEOUT   50      // Zero velocity timeout (milliseconds)

// ============================================================================
// PID CONTROLLER DEFAULTS
// ============================================================================

// Default PID gains for DC motors (runtime configurable via TLV)
// Position PID (outer loop)
#define DEFAULT_POS_KP          1.0f
#define DEFAULT_POS_KI          0.0f
#define DEFAULT_POS_KD          0.0f

// Velocity PID (middle loop)
#define DEFAULT_VEL_KP          0.5f
#define DEFAULT_VEL_KI          0.1f
#define DEFAULT_VEL_KD          0.0f

// Torque PID (inner loop - optional, requires current sensing)
#define DEFAULT_TRQ_KP          0.2f
#define DEFAULT_TRQ_KI          0.05f
#define DEFAULT_TRQ_KD          0.0f

// PID output limits
#define PID_OUTPUT_MIN          -255    // Minimum PWM value
#define PID_OUTPUT_MAX          255     // Maximum PWM value

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

// IMU (ICM-20948)
#define IMU_ENABLED             0
#define IMU_I2C_ADDR            0x68    // Default I2C address (AD0=LOW)

// Ultrasonic rangefinder
#define ULTRASONIC_ENABLED      0
#define ULTRASONIC_I2C_ADDR     0x30    // Example I2C address

// Voltage monitoring
#define VBAT_ENABLED            1       // Battery voltage monitoring
#define V5_ENABLED              1       // 5V rail monitoring
#define VSERVO_ENABLED          1       // Servo rail monitoring

// ============================================================================
// VOLTAGE DIVIDER RATIOS (ADC INPUT SCALING)
// ============================================================================

// Battery voltage divider (VBAT_SENSE on A0)
// Hardware: 50kΩ + 10kΩ divider = 1:6 ratio
#define VBAT_DIVIDER_R1         50000.0f  // Upper resistor (Ω)
#define VBAT_DIVIDER_R2         10000.0f  // Lower resistor (Ω)
#define VBAT_DIVIDER_RATIO      ((VBAT_DIVIDER_R1 + VBAT_DIVIDER_R2) / VBAT_DIVIDER_R2)

// 5V rail divider (V5_SENSE on A1)
// Hardware: 1:2 ratio
#define V5_DIVIDER_RATIO        2.0f

// Servo rail divider (VSERVO_SENSE on A2)
// Hardware: 1:3 ratio
#define VSERVO_DIVIDER_RATIO    3.0f

// ADC reference voltage (Arduino Mega 2560)
#define ADC_VREF                5.0f    // 5V reference
#define ADC_RESOLUTION          1024    // 10-bit ADC

// Low battery threshold (volts)
#define VBAT_LOW_THRESHOLD      10.5f   // Warn below this voltage

// ============================================================================
// NEOPIXEL CONFIGURATION
// ============================================================================

#define NEOPIXEL_COUNT          1       // Number of WS2812B LEDs
#define NEOPIXEL_BRIGHTNESS     64      // Default brightness (0-255)

// System status colors (first pixel reserved for status indication)
#define STATUS_COLOR_OK         0x00FF00  // Green - normal operation
#define STATUS_COLOR_LOW_BAT    0xFF0000  // Red - low battery
#define STATUS_COLOR_ERROR      0xFF8800  // Orange - error state
#define STATUS_COLOR_DISABLED   0x000000  // Off - motors disabled

// ============================================================================
// LIMIT SWITCH CONFIGURATION
// ============================================================================

// Assign limit switches to stepper motors (0 = unused)
// Each stepper can have one limit switch for homing
#define STEPPER_1_LIMIT_PIN     40      // LIM1 / BTN3
#define STEPPER_2_LIMIT_PIN     41      // LIM2 / BTN4
#define STEPPER_3_LIMIT_PIN     48      // LIM3 / BTN5
#define STEPPER_4_LIMIT_PIN     49      // LIM4 / BTN6

// Limit switch active state
#define LIMIT_ACTIVE_LOW        1       // 1 = active low, 0 = active high

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

// Uncomment to enable debug features (increases code size and reduces performance)
// #define DEBUG_MOTOR_PID         // Print PID debug info to Serial
// #define DEBUG_ENCODER           // Print encoder counts to Serial
// #define DEBUG_TLV_PACKETS       // Print TLV packet info to Serial
// #define DEBUG_VELOCITY          // Print velocity estimation debug info
// #define DEBUG_SCHEDULER         // Print scheduler task execution info

// Debug pins for oscilloscope timing measurement
// These pins toggle on entry/exit of critical sections for timing analysis
#define DEBUG_PINS_ENABLED      1

#if DEBUG_PINS_ENABLED
  #define DEBUG_PIN_ENCODER_ISR   A7    // Toggle on encoder ISR entry/exit
  #define DEBUG_PIN_STEPPER_ISR   A8    // Toggle on stepper timer ISR entry/exit
  #define DEBUG_PIN_SCHEDULER     A9    // Toggle on scheduler tick ISR entry/exit
  #define DEBUG_PIN_PID_LOOP      A10   // Toggle during PID computation
#endif

// ============================================================================
// COMPILE-TIME VALIDATION
// ============================================================================

// Ensure timer frequencies are valid
#if (SCHEDULER_TICK_FREQ_HZ != 1000)
  #error "SCHEDULER_TICK_FREQ_HZ must be 1000 Hz (Timer1 is hardcoded for 1ms)"
#endif

#if (DC_PID_FREQ_HZ > SCHEDULER_TICK_FREQ_HZ)
  #error "DC_PID_FREQ_HZ cannot exceed SCHEDULER_TICK_FREQ_HZ"
#endif

#if (UART_COMMS_FREQ_HZ > SCHEDULER_TICK_FREQ_HZ)
  #error "UART_COMMS_FREQ_HZ cannot exceed SCHEDULER_TICK_FREQ_HZ"
#endif

// Ensure encoder modes are valid
#if (ENCODER_1_MODE != ENCODER_2X && ENCODER_1_MODE != ENCODER_4X)
  #error "ENCODER_1_MODE must be ENCODER_2X or ENCODER_4X"
#endif

#if (ENCODER_2_MODE != ENCODER_2X && ENCODER_2_MODE != ENCODER_4X)
  #error "ENCODER_2_MODE must be ENCODER_2X or ENCODER_4X"
#endif

#if (ENCODER_3_MODE != ENCODER_2X && ENCODER_3_MODE != ENCODER_4X)
  #error "ENCODER_3_MODE must be ENCODER_2X or ENCODER_4X"
#endif

#if (ENCODER_4_MODE != ENCODER_2X && ENCODER_4_MODE != ENCODER_4X)
  #error "ENCODER_4_MODE must be ENCODER_2X or ENCODER_4X"
#endif

#endif // CONFIG_H
