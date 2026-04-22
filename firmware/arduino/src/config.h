/**
 * @file config.h
 * @brief Central configuration for Arduino Mega 2560 firmware
 *
 * This file contains the compile-time firmware configuration.
 *
 * Quick guide:
 *   1. Most users should start with the "Quick Tune Settings" section.
 *   2. The rest of the file contains hardware mapping, thresholds, and
 *      advanced debug / bring-up settings.
 *   3. Keep changes here behavioral only; avoid putting implementation logic
 *      into config macros.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// FIRMWARE VERSION
// ============================================================================

#define FIRMWARE_VERSION        0x00090800  // Version 0.9.8
#define TLV_PROTOCOL_VERSION_MAJOR 4
#define TLV_PROTOCOL_VERSION_MINOR 0
#define BOARD_REVISION          0       // 0 = unspecified / unknown

// ============================================================================
// QUICK TUNE SETTINGS
// ============================================================================
//
// These are the settings most users are expected to touch first.
// Battery chemistry / voltage thresholds are configured later in the dedicated
// battery section below.

// UART link to Raspberry Pi
#define RPI_BAUD_RATE           200000  // Stable bring-up rate for Mega2560 <-> RPi
#define HEARTBEAT_TIMEOUT_MS    500     // Disable actuators if heartbeat stops

// Shared I2C bus used by the Arduino-side sensor stack.
// Bring the bus up conservatively at 100 kHz during init, then run the IMU
// path at 400 kHz during normal polling. The servo controller forces its own
// 100 kHz access before PCA9685 transactions.
#define I2C_INIT_CLOCK_HZ       100000
#define I2C_BUS_CLOCK_HZ        400000
#define SERVO_I2C_CLOCK_HZ      100000
#define I2C_WIRE_TIMEOUT_US     5000

// Soft task cadences
#define UART_COMMS_FREQ_HZ      100     // UART service task
#define MOTOR_UPDATE_FREQ_HZ    200     // DC control compute round
#define SENSOR_UPDATE_FREQ_HZ   100     // Sensor dispatch task
#define IMU_UPDATE_FREQ_HZ      25      // IMU read + Fusion update cadence
#define USER_IO_FREQ_HZ         20      // LEDs / buttons / UI indications

// Human-readable status output on USB serial
#define STATUS_REPORTER_ENABLED 0       // 1 = emit [SYSTEM]/[TIMING]/[UART], 0 = disable
#define STATUS_REPORT_HZ        1       // Report frequency when enabled
// Lightweight fault-event logging on USB serial. Unlike StatusReporter, this
// only emits short one-line messages when new loop/UART/control faults appear.
#define FAULT_EVENT_LOG_ENABLED 1
#define FAULT_EVENT_MIN_INTERVAL_MS 100

// Debug serial
#define DEBUG_BAUD_RATE         115200
#define DEBUG_LOG_BUFFER_SIZE   768
#define DEBUG_FLUSH_MAX_BYTES_PER_PASS 16

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// DC Motors
#define NUM_DC_MOTORS           4       // Total DC motor channels

// All DC motor channels are always initialized. Use DC_ENABLE TLV command at
// runtime to activate specific motors. Set NUM_DC_MOTORS to reduce channel count.

// DC Motor direction inversion (H-bridge wiring correction)
// Set to 1 to invert motor direction (swaps forward/reverse)
#define DC_MOTOR_1_DIR_INVERTED 1       // 0=normal, 1=inverted
#define DC_MOTOR_2_DIR_INVERTED 1       // 0=normal, 1=inverted
#define DC_MOTOR_3_DIR_INVERTED 1       // 0=normal, 1=inverted
#define DC_MOTOR_4_DIR_INVERTED 1       // 0=normal, 1=inverted

// Stepper Motors
#define NUM_STEPPERS            4       // Total stepper channels

// All stepper channels are always initialized. Use STEP_ENABLE TLV command at
// runtime to activate specific steppers. Set NUM_STEPPERS to reduce channel count.

// Servos (via PCA9685)
#define NUM_SERVO_CHANNELS      16      // PCA9685 provides 16 channels

#define SERVO_CONTROLLER_ENABLED 1      // Enable PCA9685 servo driver

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================

#define ENCODER_PPR             1440    // Pulses per revolution (manufacturer spec, for encoder_4x)
#define ENCODER_MAX_RPM         100     // Maximum expected motor RPM

// Encoder resolution modes
#define ENCODER_2X              2       // 2x counting (phase A only)
#define ENCODER_4X              4       // 4x counting (both phases)

// Per-motor encoder mode (use ENCODER_2X or ENCODER_4X)
#define ENCODER_1_MODE          ENCODER_4X
#define ENCODER_2_MODE          ENCODER_4X
#define ENCODER_3_MODE          ENCODER_4X
#define ENCODER_4_MODE          ENCODER_4X

// Encoder direction inversion (polarity correction)
// Set to 1 to invert encoder count direction (flips positive/negative)
#define ENCODER_1_DIR_INVERTED  0       // 0=normal, 1=inverted
#define ENCODER_2_DIR_INVERTED  0       // 0=normal, 1=inverted
#define ENCODER_3_DIR_INVERTED  0       // 0=normal, 1=inverted
#define ENCODER_4_DIR_INVERTED  0       // 0=normal, 1=inverted

// ============================================================================
// ROBOT GEOMETRY — edit these to match your robot
// ============================================================================

// Outer diameter of each drive wheel (mm)
#define WHEEL_DIAMETER_MM   74.0f

// Centre-to-centre track width between the two drive wheels (mm)
#define WHEEL_BASE_MM       333.0f

// Initial theta (degree)
#define INITIAL_THETA       90.0f

// DC motor index that drives the left drive wheel (0-based, 0–3)
// Positive encoder ticks must mean "wheel moving forward".
// If the count direction is wrong, set ENCODER_N_DIR_INVERTED in config.h.
#define ODOM_LEFT_MOTOR     0 // MOTOR_1
#define ODOM_LEFT_MOTOR_DIR_INVERTED 0

// DC motor index that drives the right drive wheel (0-based, 0–3)
#define ODOM_RIGHT_MOTOR    1 // MOTOR_2
#define ODOM_RIGHT_MOTOR_DIR_INVERTED 1

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

// Control and sensor scheduling frequencies
#define DC_PID_FREQ_HZ          800     // DC motor Timer1 round-robin slot (1.25 ms; 200 Hz per motor)

// Stepper pulse generation (Timer3)
#define STEPPER_TIMER_FREQ_HZ   10000   // 10kHz interrupt rate (100µs period)
#define STEPPER_MAX_RATE_SPS    5000    // Maximum steps per second per motor

// ============================================================================
// FAULT DETECTION THRESHOLDS
// ============================================================================

// Encoder stall detection (per DC motor)
// If |PWM| > threshold and encoder position unchanged for timeout → disable that motor
// Set ENCODER_STALL_DETECTION to 0 to disable this safety feature entirely (e.g. if
// encoders are not fitted or the timeout is too aggressive for your PID tuning).
#define ENCODER_STALL_DETECTION     0    // 1=enabled, 0=disabled
#define ENCODER_FAIL_PWM_THRESHOLD  51   // ~20% of 255; below this = intentional slow/stop
#define ENCODER_FAIL_TIMEOUT_MS     500  // ms without encoder movement before fault declared
#define DC_CURRENT_SENSE_ENABLED    0    // 1=sample CT analog pins in DCMotor::service()
#define MOTOR_TASK_CATCHUP_BUDGET_US 1000 // max wall-clock catch-up budget per taskMotors() pass

// UART task wall-clock budget (used only for oscilloscope debug pin A9 reference)
// taskUART() runs at 100 Hz; the 10 ms budget is the full tick period.
// NOTE: micros()-based measurement includes ISR preemption time — it is NOT a
// control-loop overrun. The PID loop is in Timer1 ISR and cannot overrun here.
#define UART_TASK_BUDGET_US      10000   // 10 ms = full 100 Hz tick period

// UART-safe ISR budgets.
// At 200 kbps a UART byte arrives every 50 us. Keeping the common ISR path
// below ~80 us preserves margin against USART2 hardware overrun (DOR2).
#define PID_ISR_UART_BUDGET_US      80
#define STEPPER_ISR_UART_BUDGET_US  60

// Full round-robin PID/apply monitoring.
// Track the wall-clock span from the first Timer1 motor slice to the last
// Timer1 motor slice against the 200 Hz-per-motor (5 ms) control budget.
#define PID_ROUND_BUDGET_US    (1000000UL / MOTOR_UPDATE_FREQ_HZ)

// ============================================================================
// COMMUNICATION SETTINGS
// ============================================================================

// UART to Raspberry Pi (Serial2)
#define RPI_SERIAL              Serial2 // Hardware serial port

// USB debug console
//
// DEBUG_SERIAL_PORT is the raw USB serial device (Serial0). DEBUG_SERIAL is a
// Print-compatible handle routed through DebugLog so normal DEBUG_SERIAL.print*
// calls are buffered during runtime. setup() temporarily enables passthrough so
// early bring-up prints still appear immediately on USB.
class HardwareSerial;
class Print;
extern HardwareSerial &DEBUG_SERIAL_PORT;
extern Print &DEBUG_SERIAL;

// TLV runtime stream periods to the Raspberry Pi.
// These are live wire-message periods, not USB debug print intervals.
// Effective ceiling: periodic TLVs are emitted from taskUART(), so with the
// default UART_COMMS_FREQ_HZ = 100 the fastest practical interval is 10 ms.
#define TELEMETRY_SYS_STATE_RUN_MS        100
#define TELEMETRY_SYS_STATE_IDLE_MS       1000
#define TELEMETRY_SYS_POWER_RUN_MS        100
#define TELEMETRY_SYS_POWER_IDLE_MS       1000
#define TELEMETRY_DC_STATE_MS             20
#define TELEMETRY_STEP_STATE_MS           20
#define TELEMETRY_SERVO_STATE_MS          100
#define TELEMETRY_IMU_MS                  40
#define TELEMETRY_IMU_IDLE_MS             100
#define TELEMETRY_KINEMATICS_MS           40
#define TELEMETRY_KINEMATICS_IDLE_MS      100
#define TELEMETRY_IO_INPUT_STATE_MS       20
#define TELEMETRY_IO_OUTPUT_STATE_MS      100

// Device identification
#define DEVICE_ID               0x01    // Arduino device ID for TLV protocol
#define ENABLE_CRC_CHECK        1       // Enable CRC checks on TLV packets. Set to 0 to disable.

// ============================================================================
// VELOCITY ESTIMATION CONFIGURATION
// ============================================================================

// Fixed-rate velocity feedback settings
#define VELOCITY_LOWPASS_CONST  0.50f   // Low-pass weight on raw speed; y=(y+raw*k)/(1+k)
#define VELOCITY_ZERO_TIMEOUT   50      // Zero velocity timeout (milliseconds)

// ============================================================================
// PID CONTROLLER DEFAULTS
// ============================================================================

// Default PID gains for DC motors (runtime configurable via TLV)
// Position PID (outer loop)
#define DEFAULT_POS_KP          5.0f
#define DEFAULT_POS_KI          0.0f
#define DEFAULT_POS_KD          0.5f

// Velocity PID (middle loop)
#define DEFAULT_VEL_KP          0.2f
#define DEFAULT_VEL_KI          4.0f
#define DEFAULT_VEL_KD          0.0f

// PID output limits
#define PID_OUTPUT_MIN          -255    // Minimum PWM value
#define PID_OUTPUT_MAX          255     // Maximum PWM value

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

// IMU (ICM-20948 via SparkFun library + Fusion AHRS)
#define IMU_ENABLED             0
// AD0_VAL: 0 = I2C addr 0x68 (AD0 pin LOW), 1 = I2C addr 0x69 (AD0 pin HIGH)
#define IMU_AD0_VAL             1       // SparkFun breakout default: AD0 high = 0x69
// Explicit IMU full-scale settings. Keep these aligned with FusionWrapper and
// the raw sensor conversion factors in IMUDriver.
#define IMU_ACCEL_RANGE_G       2       // 2, 4, 8, or 16 g
#define IMU_GYRO_RANGE_DPS      250     // 250, 500, 1000, or 2000 dps

// Fusion AHRS settings (Madgwick-based sensor fusion)
// gain: higher = faster convergence, but more sensitivity to noise/disturbance.
// For a general rover, start around 0.2-0.3 and only increase if attitude
// response feels too sluggish.
#define FUSION_GAIN             0.2f
// Gyroscope angular-rate recovery threshold in DPS used by the Fusion library.
// This is NOT the IMU scale factor. Set to 0 to disable the automatic AHRS
// reset when a high rotation rate is observed. That is the safer default for
// a rover because hand-testing or sharp yaw motions can easily exceed a strict
// 250 dps threshold and pin yaw back to zero during recovery.
#define FUSION_GYRO_RECOVERY_DPS 0.0f
// Rejection thresholds are angular error limits in DEGREES inside the Fusion
// library, not sensor units. Lower = reject bad accel/mag data more aggressively.
#define FUSION_ACCEL_REJECTION  10.0f   // degrees of gravity disagreement
#define FUSION_MAG_REJECTION    20.0f   // degrees of magnetic disagreement
// Recovery trigger period in seconds before the algorithm exits recovery mode
#define FUSION_RECOVERY_PERIOD  5       // seconds

// Lidar (Garmin LIDAR-Lite v4)
// Note: this lidar is not supported on the Arduino firmware. Use the RPi-side
// Qwiic bus and the Pi test tooling in ros2_ws/tests if the lidar is needed.

// Ultrasonic
// Note: ultrasonic is not supported on the Arduino firmware at the moment. Use the RPi-side
// sensor stack for ultrasonic sensing, similar to the lidar path.
#define ULTRASONIC_COUNT        0

// ============================================================================
// MAGNETOMETER CALIBRATION
// ============================================================================

// Minimum samples required before calibration can be saved.
// The robot must be rotated through all orientations during this time.
// At 100 Hz IMU rate, 50 samples = 0.5 s minimum; collect for 10-20 s in practice.
#define MAG_CAL_MIN_SAMPLES     50

// EEPROM layout and addressing are managed by PersistentStorage.
// See firmware/arduino/src/modules/PersistentStorage.h for layout details.

// Voltage monitoring
#define VBAT_ENABLED            1       // Battery voltage monitoring
#define V5_ENABLED              1       // 5V rail monitoring
#define VSERVO_ENABLED          1       // Servo rail monitoring

// ============================================================================
// BATTERY TYPE CONFIGURATION
// ============================================================================
//
// Set BATTERY_TYPE to the battery you have installed.
// This automatically configures three voltage thresholds:
//
//   VBAT_WARN_V       — warn below this (LED blink, NeoPixel yellow, ERR flag)
//   VBAT_CUTOFF_V     — disable motors below this (hard safety, same as heartbeat loss)
//   VBAT_OVERVOLTAGE_V — warn above this (wrong charger / wrong chemistry)
//
// ── NiMH ────────────────────────────────────────────────────────────────────
#define BATTERY_NIMH_8CELL    1   //  8-cell NiMH,  9.6 V nominal
#define BATTERY_NIMH_10CELL   2   // 10-cell NiMH, 12.0 V nominal  ← DEFAULT
// ── LiPo ────────────────────────────────────────────────────────────────────
#define BATTERY_LIPO_2S       3   //  2S LiPo,  7.4 V nominal
#define BATTERY_LIPO_3S       4   //  3S LiPo, 11.1 V nominal
#define BATTERY_LIPO_4S       5   //  4S LiPo, 14.8 V nominal
#define BATTERY_LIPO_5S       6   //  5S LiPo, 18.5 V nominal
#define BATTERY_LIPO_6S       7   //  6S LiPo, 22.2 V nominal (⚠ near 24 V hw limit)
// ── Custom ───────────────────────────────────────────────────────────────────
#define BATTERY_CUSTOM        99  // Set VBAT_WARN_V/CUTOFF_V/OVERVOLTAGE_V manually

// ── Selection ────────────────────────────────────────────────────────────────
#define BATTERY_TYPE    BATTERY_NIMH_10CELL   // ← change this line to switch battery

// ── Per-type thresholds ───────────────────────────────────────────────────────
// NiMH thresholds use 1.05 V/cell warn, 1.00 V/cell cutoff, 1.55 V/cell over.
// LiPo thresholds use 3.5 V/cell warn,  3.3 V/cell cutoff, 4.3 V/cell over.
#if   BATTERY_TYPE == BATTERY_NIMH_8CELL
  #define VBAT_WARN_V           8.4f   // 8 × 1.05 V
  #define VBAT_CUTOFF_V         8.0f   // 8 × 1.00 V — disable motors (prevents over-discharge)
  #define VBAT_OVERVOLTAGE_V   12.5f   // above max charge (~8 × 1.45 V)
#elif BATTERY_TYPE == BATTERY_NIMH_10CELL
  #define VBAT_WARN_V          10.5f   // 10 × 1.05 V
  #define VBAT_CUTOFF_V        10.0f   // 10 × 1.00 V
  #define VBAT_OVERVOLTAGE_V   15.5f   // above max charge (~10 × 1.45 V)
#elif BATTERY_TYPE == BATTERY_LIPO_2S
  #define VBAT_WARN_V           7.0f   // 2 × 3.5 V
  #define VBAT_CUTOFF_V         6.6f   // 2 × 3.3 V — LiPo damage threshold
  #define VBAT_OVERVOLTAGE_V    8.6f   // above 2 × 4.3 V
#elif BATTERY_TYPE == BATTERY_LIPO_3S
  #define VBAT_WARN_V          10.5f   // 3 × 3.5 V
  #define VBAT_CUTOFF_V         9.9f   // 3 × 3.3 V
  #define VBAT_OVERVOLTAGE_V   12.9f   // above 3 × 4.3 V
#elif BATTERY_TYPE == BATTERY_LIPO_4S
  #define VBAT_WARN_V          14.0f   // 4 × 3.5 V
  #define VBAT_CUTOFF_V        13.2f   // 4 × 3.3 V
  #define VBAT_OVERVOLTAGE_V   17.2f   // above 4 × 4.3 V
#elif BATTERY_TYPE == BATTERY_LIPO_5S
  #define VBAT_WARN_V          17.5f   // 5 × 3.5 V
  #define VBAT_CUTOFF_V        16.5f   // 5 × 3.3 V
  #define VBAT_OVERVOLTAGE_V   21.5f   // above 5 × 4.3 V
#elif BATTERY_TYPE == BATTERY_LIPO_6S
  #define VBAT_WARN_V          21.0f   // 6 × 3.5 V
  #define VBAT_CUTOFF_V        19.8f   // 6 × 3.3 V
  #define VBAT_OVERVOLTAGE_V   24.0f   // hw limit; 6S full charge = 25.2 V (not measurable)
#elif BATTERY_TYPE == BATTERY_CUSTOM
  // Define your own thresholds:
  #define VBAT_WARN_V          10.5f
  #define VBAT_CUTOFF_V        10.0f
  #define VBAT_OVERVOLTAGE_V   16.0f
#else
  #error "Unknown BATTERY_TYPE — see BATTERY_TYPE options in config.h"
#endif

// Backward-compatible alias used by SensorManager::isBatteryLow()
#define VBAT_LOW_THRESHOLD  VBAT_WARN_V

// Minimum voltage to consider a battery present and safe for motor enables.
// Below this threshold (including 0 V when no battery is connected), all
// actuator enable commands are silently rejected without entering ERROR state.
// Must be well below any real battery's minimum cell voltage — 2 V covers
// all supported chemistries (NiMH, LiPo 2S–6S) while remaining above ADC noise.
#define VBAT_MIN_PRESENT_V      2.0f

// Minimum servo rail voltage to consider external servo power present.
// Servos are powered from a separate rail, so servo enable should not depend
// solely on the main battery measurement.
#define VSERVO_MIN_PRESENT_V    4.0f

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

// ============================================================================
// NEOPIXEL CONFIGURATION
// ============================================================================

#define NEOPIXEL_COUNT          1       // Number of WS2812B LEDs
#define NEOPIXEL_BRIGHTNESS     64      // Default brightness (0-255)

// ============================================================================
// LIMIT SWITCH CONFIGURATION
// ============================================================================

// Stepper limit switch assignments (maps to limit switch pins)
// Uncomment and update these definitions if limit switches are connected for homing.
// If no limit switches are used, leave these undefined
// Homing is disabled if no limit switch pins are defined.

#define PIN_ST1_LIMIT           PIN_LIM1  // Stepper 1 limit (40)
// #define PIN_ST2_LIMIT           PIN_LIM2  // Stepper 2 limit (41)
// #define PIN_ST3_LIMIT           PIN_LIM3  // Stepper 3 limit (48)
// #define PIN_ST4_LIMIT           PIN_LIM4  // Stepper 4 limit (49)

// DC motor home limit switch assignments (optional)
// Uncomment and update these definitions if DC motors use limit switches for
// encoder zeroing / homing. Leave undefined to disable homing for that motor.

#define PIN_M1_LIMIT        PIN_LIM5
// #define PIN_M2_LIMIT        PIN_LIM6
// #define PIN_M3_LIMIT        PIN_LIM7
// #define PIN_M4_LIMIT        PIN_LIM8

// Limit switch active state
#define LIMIT_ACTIVE_LOW        1       // 1 = active low, 0 = active high

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

// Uncomment to enable debug features (increases code size and reduces performance)
// #define DEBUG_MOTOR_PID         // Print PID debug info to Serial
// #define DEBUG_ENCODER           // Print encoder counts to Serial
// #define DEBUG_TLV_PACKETS       // Print TLV packet info to Serial
// #define DEBUG_UART_RX_BYTES     // Print every raw byte received on Serial2
// #define DEBUG_VELOCITY          // Print velocity estimation debug info
// #define DEBUG_SCHEDULER         // Print scheduler task execution info

// Debug pins for oscilloscope timing measurement
// These pins toggle on entry/exit of critical sections for timing analysis.
//
// SCOPE CHANNEL GUIDE — connect probes, set to 3.3V/5V trigger:
//
//   ISR timing (always available when ENABLED):
//     A10  PID_LOOP      HIGH = inside Timer1 ISR (200 Hz, budget ~500 µs)
//     A8   STEPPER_ISR   HIGH = inside Timer3 ISR (10 kHz, budget ~10 µs)
//     A7   ENCODER_ISR   HIGH = inside any encoder interrupt
//
//   UART task timing:
//     A11  UART_TASK     HIGH = entire taskUART() running (100 Hz, ~1-6 ms typical)
//     A12  UART_RX       HIGH = inside processIncoming() only
//     A13  UART_TX       HIGH = inside sendTelemetry() only
//     A9   UART_LATE     Pulse when wall-clock > 10 ms (usually ISR preemption, not a real problem)
//
// KEY INSIGHT (verified by scope):
//   micros()-based measurement in loop() INCLUDES ISR preemption time.
//   A PID_LOOP pulse (A10) inside a UART_TASK window inflates the elapsed time
//   without any actual UART slowness. This is normal and expected — it is NOT a
//   control-loop overrun. The PID runs in Timer1 ISR and cannot be affected by
//   anything in loop().
//
// SUGGESTED SCOPE SETUPS:
//
//   Q: "Is RX or TX the bottleneck?"
//     Ch1=A12 (UART_RX), Ch2=A13 (UART_TX) — time-base 2 ms/div.
//
//   Q: "Are ISRs preempting the UART task?"
//     Ch1=A11 (UART_TASK), Ch2=A10 (PID_LOOP) — time-base 1 ms/div.
//     PID pulses inside UART_TASK window = ISR preemption inflation (normal).
//
//   Q: "Is the UART task running reliably at 100 Hz?"
//     Ch1=A11 (UART_TASK) — time-base 5 ms/div, trigger rising.
//     Period should be ~10 ms; width should be 1-6 ms.
//
#define DEBUG_PINS_ENABLED      0

#if DEBUG_PINS_ENABLED
  // ISR channels (original)
  #define DEBUG_PIN_ENCODER_ISR   A7    // HIGH inside any encoder ISR
  #define DEBUG_PIN_STEPPER_ISR   A8    // HIGH inside Timer3 (stepper) ISR
  #define DEBUG_PIN_UART_LATE     A9    // Pulse when taskUART() wall-clock > 10 ms (ISR inflation, not a real error)
  #define DEBUG_PIN_PID_LOOP      A10   // HIGH inside Timer1 (PID) ISR

  // UART task sub-channels (new — overrun diagnosis)
  #define DEBUG_PIN_UART_TASK     A11   // HIGH for entire taskUART() execution
  #define DEBUG_PIN_UART_RX       A12   // HIGH during processIncoming() only
  #define DEBUG_PIN_UART_TX       A13   // HIGH during sendTelemetry() only
#endif

// ============================================================================
// COMPILE-TIME VALIDATION
// ============================================================================

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

#if STATUS_REPORTER_ENABLED && (STATUS_REPORT_HZ < 1)
  #error "STATUS_REPORT_HZ must be >= 1 when STATUS_REPORTER_ENABLED=1"
#endif

#endif // CONFIG_H
