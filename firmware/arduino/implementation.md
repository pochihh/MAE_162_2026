# Arduino Firmware Implementation Plan

This document details the implementation plan for the Arduino Mega 2560 firmware controlling a differential drive robot platform.

---

## Overview

The firmware provides real-time motor control, sensor integration, and communication with a Raspberry Pi master via TLV protocol over UART.
Refer to [pin_table.md](pin_table.md) for the complete GPIO mapping.
Refer to [technical_notes.md](technical_notes.md) for low-level technical details on timers, interrupts, and design rationale.

## Compile the Code

The main sketch is in `firmware/arduino/`; tests are in `firmware/tests/`. Run the following commands from the `firmware/` directory:

```bash
# Compile main firmware
arduino-cli compile --fqbn arduino:avr:mega arduino

# Compile specific test
arduino-cli compile --fqbn arduino:avr:mega tests/test_scheduler

# Upload to Arduino Mega (find your port with: arduino-cli board list)
arduino-cli upload -p /dev/cu.usbserial* --fqbn arduino:avr:mega arduino
```

The test sketches are in `firmware/tests/`. Each test must be in its own folder (Arduino IDE requirement).

### Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Multitasking | Cooperative scheduler + hardware ISRs | Deterministic timing, no RTOS overhead |
| Encoder counting | Direct hardware ISR (INT0-INT5) | Lowest latency, no missed edges |
| DC Motor PID | 200 Hz via scheduler | Smooth control, adequate for most DC motors |
| Stepper control | Timer3 interrupt (10kHz) | Precise step timing, low jitter |
| Sensor polling | 50 Hz via scheduler | Sufficient for navigation sensors |
| UART comms | 100 Hz via scheduler @ 921600 baud | High throughput, matches sensor rate |
| Safety timeout | 50ms global | All motors disabled if no heartbeat |

---

## Interrupt and Scheduler Architecture

> **📖 For detailed technical information** about ATmega2560 interrupt priorities, vector tables, nesting behavior, and design rationale, see [technical_notes.md](technical_notes.md).

The firmware uses a hybrid interrupt architecture combining hardware ISRs with a cooperative scheduler:

### Interrupt Priority Summary

| Vector | Source | Frequency | Purpose |
|--------|--------|-----------|---------|
| 1-6 | External INT0-INT5 (Encoders) | Event-driven (~1200 Hz max) | Encoder pulse counting |
| 19 | Timer1 COMPA | 1 kHz (1ms period) | Scheduler base tick |
| 35 | Timer3 COMPA | 10 kHz (100µs period) | Stepper pulse generation |
| N/A | Main loop | Continuous | Cooperative task execution |

**Key Points:**
- Interrupt priorities are **FIXED by hardware** (cannot be changed in software)
- All ISRs run to completion without nesting (Arduino default)
- Encoder ISRs (vectors 1-6) have highest priority
- Timer1 (vector 19) has higher priority than Timer3 (vector 35)

### Scheduler Task Priorities

Within the main loop, the cooperative scheduler runs tasks in priority order:

| Priority | Task | Period | Purpose |
|----------|------|--------|---------|
| 0 (highest) | DC Motor PID | 5ms (200Hz) | Motor control loops |
| 1 | UART Communication | 10ms (100Hz) | Process TLV messages, safety timeout |
| 2 | Sensor Reading | 20ms (50Hz) | Read all sensors, queue outgoing data |
| 3 (lowest) | User I/O | 50ms (20Hz) | Update LED animations |

**Note:** Hardware ISRs always preempt scheduler tasks, regardless of task priority.

---

### 1. Encoder ISRs (Hardware Interrupts)

> **📖 For ISR design principles and timing analysis**, see [technical_notes.md](technical_notes.md#isr-design-principles).

Encoders use direct hardware interrupts (INT0-INT5) for minimum latency:

```
Pin 2  (INT0) ──┬── encoderISR_M1() ──> volatile encoderCount[0]
Pin 3  (INT1) ──┼── encoderISR_M2() ──> volatile encoderCount[1]
Pin 18 (INT5) ──┼── encoderISR_M3() ──> volatile encoderCount[2]
Pin 19 (INT4) ──┴── encoderISR_M4() ──> volatile encoderCount[3]
```

**ISR Implementation Requirements:**
- Keep ISR body minimal (<10µs execution time)
- Read direction pin, increment/decrement counter, capture timestamp
- No function calls, no floating point, no Serial operations
- Velocity computation happens in PID loop (200Hz), NOT in ISR

**Supported Resolution Modes:**
- **2x Mode** (default): Phase A interrupt only, read phase B for direction
- **4x Mode** (future): Both phases interrupt, state machine decoding

### 2. Timer3 - Stepper Pulse Generation

> **📖 For timer configuration details**, see [technical_notes.md](technical_notes.md#timer-configuration-details).

Timer3 generates precise step pulses for all stepper motors:

```
Timer3 @ 10kHz ──> ISR(TIMER3_COMPA_vect) ──> For each enabled stepper:
                                               ├── Decrement step interval counter
                                               ├── Toggle STEP pin when counter reaches zero
                                               └── Update position tracking
```

**Configuration:**
- 10kHz interrupt rate (100µs step resolution)
- Maximum step rate: ~5000 steps/sec per motor
- ISR time budget: <50µs total for all 4 steppers
- Acceleration: Linear (trapezoidal) profile

### 3. Timer1 - Scheduler Base Tick

Timer1 provides the 1ms base tick for the cooperative scheduler:

```
Timer1 @ 1kHz ──> ISR(TIMER1_COMPA_vect) ──> For each registered task:
                                            ├── Decrement countdown
                                            └── If zero: set ready flag, reload period

Main Loop ──> Scheduler::tick() ──> Find highest-priority ready task
                                 ├── Clear ready flag
                                 ├── Execute task callback
                                 └── Return to loop()
```

**Important:** Tasks are cooperative (not preemptive). Each task should complete within its time budget to avoid delaying lower-priority tasks.

### Timer Allocation Summary

| Timer | Resolution | Usage | Notes |
|-------|------------|-------|-------|
| Timer0 | 8-bit | Arduino millis()/micros() | DO NOT MODIFY |
| Timer1 | 16-bit | Scheduler base tick (1kHz) | OCR1A compare match |
| Timer2 | 8-bit | Available | Could use for tone/PWM |
| Timer3 | 16-bit | Stepper pulse generation (10kHz) | OCR3A compare match |
| Timer4 | 16-bit | Available | Reserved for future use |
| Timer5 | 16-bit | Available | Reserved for future use |

---

## Proposed Changes

### Global Configuration

#### [NEW] `src/config.h`

Central configuration header for all compile-time parameters. Contains:

- Hardware counts (NUM_DC_MOTORS, NUM_STEPPERS, NUM_SERVO_CHANNELS)
- Per-channel enable flags (DC_MOTOR_1_ENABLED, etc.)
- Timing parameters (DC_PID_FREQ_HZ, SENSOR_UPDATE_FREQ_HZ, etc.)
- Communication settings (RPI_BAUD_RATE, DEVICE_ID)
- Encoder configuration:
  ```cpp
  #define ENCODER_PPR             1440    // Pulses per revolution (4x counted)
  #define ENCODER_MAX_RPM         100     // Maximum motor RPM

  // Per-motor resolution mode (ENCODER_2X or ENCODER_4X)
  #define ENCODER_1_MODE          ENCODER_2X
  #define ENCODER_2_MODE          ENCODER_2X
  #define ENCODER_3_MODE          ENCODER_2X
  #define ENCODER_4_MODE          ENCODER_2X
  ```
- Default PID gains (runtime configurable via TLV)
- Sensor enable flags and I2C addresses
- Voltage divider ratios for ADC readings
- Debug options:
  ```cpp
  // Uncomment to enable debug features
  // #define DEBUG_MOTOR_PID       // Print PID debug info
  // #define DEBUG_ENCODER         // Print encoder counts
  // #define DEBUG_TLV_PACKETS     // Print TLV packet info

  // Debug pin toggles for oscilloscope timing measurement
  #define DEBUG_PINS_ENABLED      1
  #define DEBUG_PIN_ENCODER_ISR   A7    // Toggle on encoder ISR entry/exit
  #define DEBUG_PIN_STEPPER_ISR   A8    // Toggle on stepper timer ISR
  #define DEBUG_PIN_SCHEDULER     A9    // Toggle on scheduler tick
  #define DEBUG_PIN_PID_LOOP      A10   // Toggle during PID computation
  ```

#### [NEW] `src/pins.h`

Hardware pin assignments separated from configuration. Contains:

```cpp
// All pin definitions in one place - matches README.md GPIO table
// DC Motors: EN, IN1, IN2, ENC_A, ENC_B, CT pins per motor
// Steppers: STEP, DIR, EN pins per stepper
// Limit switches: LIM1-LIM8 pins
// User I/O: LED pins, button pins, NeoPixel pin
// Analog: VBAT, V5, VSERVO sense pins
```

This file should only contain `#define` statements for pin numbers. It mirrors the GPIO table in README.md and makes hardware changes easy to track.

---

### Timer/Scheduler Module

#### [NEW] `src/Scheduler.h` / `src/Scheduler.cpp`

Cooperative multitasking scheduler using Timer1 for precise timing.

**Public Interface:**
- `Scheduler::init()` - Configure Timer1 for 1kHz interrupts
- `Scheduler::registerTask(callback, periodMs, priority)` - Add task to scheduler
- `Scheduler::tick()` - Called from main loop, executes highest-priority ready task
- `Scheduler::timerISR()` - Called from Timer1 ISR, updates task ready flags

**Implementation Notes:**
- Maximum 8 registered tasks (compile-time limit)
- Priority 0 is highest, 7 is lowest
- Tasks are NOT preemptive - each runs to completion
- Timer1 configured in CTC mode with OCR1A for 1ms period
- ISR only sets flags, never executes task code directly

---

### Encoder Module

#### [NEW] `src/modules/EncoderCounter.h` / `src/modules/EncoderCounter.cpp`

Modular encoder counting with support for 2x and 4x resolution modes.

**Interface:**
```cpp
class IEncoderCounter {
public:
    virtual void init() = 0;
    virtual int32_t getCount() const = 0;
    virtual void resetCount() = 0;
    virtual void onInterruptA() = 0;  // Called from phase A ISR
    virtual void onInterruptB() = 0;  // Called from phase B ISR (4x mode only)
    virtual uint32_t getLastEdgeUs() const = 0;  // For velocity estimation
    virtual uint8_t getResolutionMultiplier() const = 0;  // Returns 2 or 4
};
```

**Implementations:**

1. **`EncoderCounter2x`** (Current Hardware)
   - Interrupt on phase A only (CHANGE = rising + falling)
   - Reads phase B to determine direction
   - Resolution: 2 edges per encoder pulse (720 edges/rev with 1440 PPR encoder)
   - ISR time: ~10 CPU cycles

2. **`EncoderCounter4x`** (Future Hardware)
   - Interrupts on both phase A and phase B (CHANGE on both)
   - State machine decoding for direction:
     ```
     State Table (prevA, prevB, currA, currB) → direction
     00 → 01: +1    01 → 11: +1    11 → 10: +1    10 → 00: +1
     00 → 10: -1    10 → 11: -1    11 → 01: -1    01 → 00: -1
     ```
   - Resolution: 4 edges per encoder pulse (1440 edges/rev)
   - ISR time: ~15-20 CPU cycles

**Configuration in `config.h`:**
```cpp
// Per-motor encoder resolution mode
#define ENCODER_1_MODE  ENCODER_2X  // or ENCODER_4X
#define ENCODER_2_MODE  ENCODER_2X
#define ENCODER_3_MODE  ENCODER_2X
#define ENCODER_4_MODE  ENCODER_2X
```

**Hardware Requirements for 4x Mode:**
- Phase B pins must be on interrupt-capable pins (INT0-INT5 or PCINT)
- Current hardware only routes phase A to hardware INT pins
- Future PCB revision could route phase B to available INT2/INT3 pins
- Alternative: Use Pin Change Interrupts (PCINT) for phase B (higher latency)

**Why We Use INT0-INT5 (External Interrupts) for Encoders:**
- External interrupts have the **HIGHEST hardware priority** on ATmega2560 (vectors 1-6)
- Ensures encoder edges are NEVER missed, even during heavy timer/UART activity
- Motor 1 (INT0, vector 1) gets slightly higher priority than Motor 3 (INT5, vector 6)
- At 100 RPM with 1440 PPR encoders, the ~1µs priority difference between motors is negligible
- All four encoder ISRs have priority over Timer1 (scheduler) and Timer3 (steppers)
- This design guarantees zero missed encoder edges under all operating conditions

---

### Velocity Estimation Module

#### [NEW] `src/modules/VelocityEstimator.h` / `src/modules/VelocityEstimator.cpp`

Modular velocity estimation with swappable algorithms.

**Interface:**
```cpp
class IVelocityEstimator {
public:
    virtual void onEncoderEdge(int8_t direction, uint32_t timestampUs) = 0;
    virtual float getVelocity() const = 0;  // ticks/sec
    virtual void update(uint32_t currentUs) = 0;  // Called from PID loop
    virtual void reset() = 0;
};
```

**Data Flow:**
```
Encoder ISR                           PID Loop (200Hz)
    │                                      │
    ├─> Store timestamp ──────────────────>├─> VelocityEstimator::update()
    ├─> Increment counter                  ├─> Compute velocity from timestamps
    └─> (exit ISR)                         └─> Use velocity in PID calculation
```

**Implementations:**

1. **`EdgeTimeVelocityEstimator`** (Default)
   - ISR captures `micros()` timestamp for each edge
   - `update()` computes velocity from time between recent edges
   - Moving average filter (configurable size: 2-8 samples)
   - Zero-velocity detection via timeout (no edges for >50ms = velocity 0)
   - Best for: Low-medium speeds, smooth velocity estimate

2. **`PulseCountVelocityEstimator`** (Alternative)
   - Counts edges over fixed time window
   - Better for: High speeds where edge timing is noisy
   - Lower resolution at low speeds

**Why compute in PID loop, not ISR:**
- Keeps ISR minimal (~10 cycles vs ~50+ for velocity math)
- 200Hz update rate is adequate for PID control
- Floating point math in ISR would block other interrupts

---

### DC Motor Control

#### [NEW] `src/drivers/DCMotor.h` / `src/drivers/DCMotor.cpp`

DC motor driver with cascade PID control.

**Public Interface:**
- `init()`, `enable()`, `disable()` - Lifecycle management
- `setTargetPosition(ticks)` - Position control mode
- `setTargetVelocity(ticksPerSec)` - Velocity control mode
- `setPositionPID()`, `setVelocityPID()`, `setTorquePID()` - Runtime PID tuning
- `update()` - Called from scheduler at 200Hz
- `getPosition()`, `getVelocity()`, `getCurrent()` - State queries

**Control Modes:**
- `DISABLED` - Motor driver disabled, no PWM output
- `POSITION` - Cascade: Position PID → Velocity PID → (optional) Torque PID → PWM
- `VELOCITY` - Cascade: Velocity PID → (optional) Torque PID → PWM

**Cascade PID Structure:**
```
Target Position ──> [Position PID] ──> Velocity Setpoint
                                              │
Target Velocity ──────────────────────────────┴──> [Velocity PID] ──> Torque Setpoint
                                                                             │
                                                          [Torque PID] ──────┴──> PWM Output
                                                          (optional)
```

**Dependencies:**
- Uses `IEncoderCounter` for position feedback
- Uses `IVelocityEstimator` for velocity feedback
- Current sensing via ADC for torque loop (optional)

---

### Stepper Motor Control

#### [NEW] `src/drivers/StepperMotor.h` / `src/drivers/StepperMotor.cpp`

Individual stepper motor driver with acceleration support.

**Public Interface:**
- `init()`, `enable()`, `disable()` - Lifecycle management
- `moveSteps(steps)` - Relative move
- `moveToPosition(position)` - Absolute move
- `home()` - Move until limit switch triggers
- `stop()` - Emergency stop (immediate, no deceleration)
- `setMaxVelocity(stepsPerSec)`, `setAcceleration(stepsPerSecSq)` - Motion profile
- `timerCallback()` - Called from Timer3 ISR, generates step pulses

**States:** `IDLE`, `MOVING`, `HOMING`

**Motion Profile:**
- Trapezoidal velocity profile (accelerate → cruise → decelerate)
- Acceleration computed using Bresenham-style integer math
- Step interval updated each step for smooth acceleration

#### [NEW] `src/modules/StepperManager.h` / `src/modules/StepperManager.cpp`

Manages Timer3 and coordinates all stepper motors.

**Responsibilities:**
- Configure Timer3 for 10kHz interrupt rate
- Timer3 ISR calls `timerCallback()` on each enabled stepper
- Provides access to individual steppers via `getStepper(id)`

**Timer3 ISR Timing:**
- 10kHz = 100µs period
- ISR must complete in <50µs to avoid jitter
- With 4 steppers: ~10µs per stepper available

---

### Servo Control

#### [NEW] `src/drivers/ServoController.h` / `src/drivers/ServoController.cpp`

Wrapper around PCA9685 I2C PWM driver for servo control.

**Public Interface:**
- `init()`, `enable()`, `disable()` - Lifecycle (controls PCA9685 OE pin)
- `setPositionUs(channel, pulseWidthUs)` - Set pulse width (500-2500µs typical)
- `setPositionDeg(channel, degrees)` - Set angle (0-180°)
- `setMultiplePositions(...)` - Bulk update for synchronized motion

**Notes:**
- Uses I2C communication (not time-critical)
- PCA9685 handles PWM generation independently
- 50Hz update rate to PCA9685 is sufficient for servos

---

### Communication Layer

#### [MODIFY] `src/messages/MessageCenter.h` / `src/messages/MessageCenter.cpp`

Central message handling for TLV protocol communication.

**Responsibilities:**
- Receive and decode TLV packets from RPi via Serial2
- Dispatch commands to appropriate handlers
- Queue and send outgoing sensor/status packets
- Track heartbeat for safety timeout

**Public Interface:**
- `init()` - Initialize UART and TLV codec
- `processingTick()` - Called at 100Hz from scheduler
- `sendXXX()` methods - Queue outgoing data packets
- `isHeartbeatValid()` - Check safety timeout status

**Command Dispatch:**
Each TLV type has a dedicated handler function (e.g., `handleDCEnable()`, `handleStepMove()`). Handlers parse the payload struct and call appropriate motor/sensor APIs.

#### [NEW] `src/messages/TLV_Payloads.h`

Packed struct definitions for all TLV message payloads.

**Command Payloads (RPi → Arduino):**
- `PayloadDCEnable`, `PayloadDCSetPosition`, `PayloadDCSetVelocity`
- `PayloadSetPID`, `PayloadStepEnable`, `PayloadStepMove`
- `PayloadStepSetAccel`, `PayloadStepSetVel`
- `PayloadServoSet`, `PayloadSetLED`, `PayloadSetNeoPixel`

**Response Payloads (Arduino → RPi):**
- `PayloadDCStatus`, `PayloadStepStatus`
- `PayloadSensorVoltage`, `PayloadIMU`
- `PayloadSystemStatus`

All structs use `#pragma pack(push, 1)` for wire format compatibility.

#### [EXISTING] `src/messages/TLV_TypeDefs.h`

TLV type constants matching README.md protocol specification.

---

### Sensor Modules

#### [NEW] `src/modules/SensorManager.h` / `src/modules/SensorManager.cpp`

Aggregates all sensor readings and provides a unified interface.

**Public Interface:**
- `init()` - Initialize all enabled sensors
- `update()` - Called at 50Hz from scheduler, reads all sensors
- `getBatteryVoltage()`, `get5VRailVoltage()`, `getServoVoltage()` - ADC readings
- `getIMUData(accel, gyro)` - Returns latest IMU data
- `getRange(sensorId)` - Ultrasonic distance reading

**Sensor Drivers Used:**
- `IMUDriver` for ICM-20948
- `UltrasonicDriver` for I2C rangefinder
- Direct ADC for voltage monitoring

#### [NEW] `src/drivers/IMUDriver.h` / `src/drivers/IMUDriver.cpp`

Wrapper around ICM-20948 library (in `lib/`).

**Public Interface:**
- `init()` - Configure IMU over I2C
- `readAccelGyro(accel, gyro)` - Read 6-axis data

**Notes:**
- DMP support planned for future (quaternion output)
- I2C address configurable (default 0x68)

#### [NEW] `src/drivers/UltrasonicDriver.h` / `src/drivers/UltrasonicDriver.cpp`

Wrapper for I2C ultrasonic sensor (e.g., HC-SR04 with I2C adapter).

**Public Interface:**
- `init()` - Configure sensor
- `triggerMeasurement()` - Start ranging
- `readDistanceMm()` - Get last measurement

---

### User I/O Module

#### [NEW] `src/modules/UserIO.h` / `src/modules/UserIO.cpp`

Manages buttons, LEDs, and NeoPixels.

**Public Interface:**
- `init()` - Configure all I/O pins
- `readButtons()` - Returns 10-bit bitmask of button states
- `readLimitSwitches()` - Returns 8-bit bitmask of limit switch states
- `setLED(ledId, mode, brightness, periodMs)` - Control user LEDs
- `setNeoPixel(index, r, g, b)` - Set individual pixel color
- `setSystemStatus(status)` - Set first NeoPixel to status color
- `update()` - Called at 20Hz, handles LED animations

**LED Modes:** `OFF`, `ON`, `PWM`, `BLINK`, `BREATHE`

#### [NEW] `src/drivers/NeoPixelDriver.h` / `src/drivers/NeoPixelDriver.cpp`

Our abstraction layer for addressable RGB LEDs.

**Purpose:**
- Wraps underlying NeoPixel library (currently testing Adafruit_NeoPixel)
- Allows swapping to alternative libraries (FastLED, tinyNeoPixel, etc.)
- Isolates timing-critical code from application logic

**Public Interface:**
- `init()` - Configure pin and LED count
- `setPixel(index, r, g, b)` - Set pixel color (buffered)
- `show()` - Push buffer to LEDs (disables interrupts briefly ~30µs per LED)
- `clear()` - Set all pixels to off

---

### Main Application

#### [MODIFY] `arduino.ino`

Main entry point and system initialization.

**Responsibilities:**
- Include all module headers
- Create global DC motor and encoder instances
- Define encoder ISR trampolines (forward calls to encoder objects)
- Define scheduler task callbacks
- `setup()`: Initialize all subsystems, attach interrupts, register tasks
- `loop()`: Call `Scheduler::tick()` continuously

**Initialization Order:**
1. Debug serial (Serial0)
2. Scheduler (Timer1)
3. MessageCenter (Serial2 + TLV codec)
4. SensorManager (I2C, ADC)
5. UserIO (GPIO, NeoPixel)
6. ServoController (PCA9685 via I2C)
7. StepperManager (Timer3)
8. DC Motors (PWM pins, encoder counters)
9. Attach encoder ISRs (attachInterrupt)
10. Register scheduler tasks

**Encoder ISR Attachment:**
```cpp
// ISR trampolines - minimal code, just forward to encoder object
void encoderISR_M1() { encoders[0].onInterrupt(); }
void encoderISR_M2() { encoders[1].onInterrupt(); }
void encoderISR_M3() { encoders[2].onInterrupt(); }
void encoderISR_M4() { encoders[3].onInterrupt(); }

// In setup():
attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2, CHANGE);
attachInterrupt(digitalPinToInterrupt(PIN_M3_ENC_A), encoderISR_M3, CHANGE);
attachInterrupt(digitalPinToInterrupt(PIN_M4_ENC_A), encoderISR_M4, CHANGE);
```

**Registered Tasks:**
| Priority | Callback | Period | Purpose |
|----------|----------|--------|---------|
| 0 | `taskDCMotorPID()` | 5ms | Update all DC motor PID loops |
| 1 | `taskUARTComms()` | 10ms | Process TLV messages, check safety timeout |
| 2 | `taskSensorRead()` | 20ms | Read all sensors, queue outgoing data |
| 3 | `taskUserIO()` | 50ms | Update LED animations |

---

## File Structure

```
firmware/
├── arduino/                    # Main firmware sketch
│   ├── arduino.ino                 # Main entry point
│   ├── tlvcodec_compile.cpp        # Wrapper to compile tlvcodec.c
│   ├── implementation.md           # This document
│   ├── lib/
│   │   ├── tlvcodec.c/h            # TLV protocol codec (C library)
│   │   ├── ICM_20948.cpp/h         # IMU driver
│   │   └── PCA9685.cpp/h           # PWM servo driver
│   └── src/
│       ├── config.h                # Compile-time configuration
│       ├── pins.h                  # Hardware pin definitions
│       ├── Scheduler.h/cpp         # Timer1-based cooperative scheduler
│       │
│       ├── messages/               # Communication Layer
    │   ├── MessageCenter.h/cpp    # TLV packet handling
    │   ├── TLV_TypeDefs.h         # TLV type constants
    │   └── TLV_Payloads.h         # Packed payload structs
    │
    ├── drivers/               # Hardware Abstraction (low-level)
    │   ├── DCMotor.h/cpp          # DC motor with cascade PID
    │   ├── StepperMotor.h/cpp     # Stepper with acceleration
    │   ├── ServoController.h/cpp  # PCA9685 wrapper
    │   ├── IMUDriver.h/cpp        # ICM-20948 wrapper
    │   ├── UltrasonicDriver.h/cpp # I2C ultrasonic wrapper
    │   └── NeoPixelDriver.h/cpp   # Addressable LED wrapper
    │
│       └── modules/               # High-level Logic
│           ├── EncoderCounter.h/cpp   # Encoder counting (2x/4x modes)
│           ├── VelocityEstimator.h/cpp # Velocity calculation algorithms
│           ├── StepperManager.h/cpp   # Timer3 + stepper coordination
│           ├── SensorManager.h/cpp    # Sensor data aggregation
│           └── UserIO.h/cpp           # Buttons, LEDs, NeoPixels
│
└── tests/                      # Test sketches (separate from main sketch)
    ├── test_scheduler/
    │   ├── test_scheduler.ino      # Phase 1 test
    │   ├── src -> ../../arduino/src    # Symbolic link
    │   └── lib -> ../../arduino/lib    # Symbolic link
    ├── test_uart_tlv/
    │   ├── test_uart_tlv.ino       # Phase 2 test
    │   ├── src -> ../../arduino/src    # Symbolic link
    │   └── lib -> ../../arduino/lib    # Symbolic link
    └── (future test folders...)
```

**Layer Separation:**
- `lib/`: External libraries, not modified (or minimally patched)
- `src/drivers/`: Direct hardware interaction, thin wrappers
- `src/modules/`: Business logic, coordinates multiple drivers
- `src/messages/`: Communication protocol handling
- `tests/`: Each test in its own folder with symbolic links to share src/ and lib/

---

## Verification Plan

### Automated Tests

1. **Compile check**: `arduino-cli compile --fqbn arduino:avr:mega`
2. **Unit tests** (on host with mocks):
   - PID controller math verification
   - TLV packet encode/decode round-trip
   - Velocity estimator edge cases
   - Encoder state machine (4x mode)

### Hardware Tests

1. **Scheduler timing**: Verify task periods with oscilloscope on debug pin toggle
2. **Encoder counting**: Manually rotate motors, verify count accuracy and direction
3. **Motor control**: Step response tests for position/velocity modes
4. **Stepper motion**: Verify acceleration profiles and limit switch homing
5. **TLV comms**: Loopback test with Python on RPi

### Integration Tests

1. **Safety timeout**: Disconnect RPi, verify motors stop within 50ms
2. **Full sensor read cycle**: Log all sensor data at 50Hz
3. **Multi-motor coordination**: Run 4 DC motors simultaneously
4. **ISR timing**: Verify no missed encoder edges under full system load
5. **End-to-end**: RPi sends position commands, Arduino responds with status

---

## Implementation Order

> [!IMPORTANT]
> Dependencies must be built in order. Each phase should be tested with the corresponding test sketch before proceeding.

### Phase 1: Foundation

| Component | Description |
|-----------|-------------|
| `config.h` | All timing, enable flags, and default parameters |
| `pins.h` | Hardware pin definitions (from README.md GPIO table) |
| `Scheduler.h/cpp` | Timer1 configuration and task management |
| `arduino.ino` | Basic structure with scheduler initialization |

**Test:** `tests/test_scheduler.ino` - Verify 1ms tick accuracy with LED blink

### Phase 2: Communication

| Component | Description |
|-----------|-------------|
| `TLV_Payloads.h` | Packed struct definitions for all message types |
| `MessageCenter.h/cpp` | Refactored for new TLV protocol with command dispatch |

**Test:** `tests/test_uart_tlv.ino` - Verify 921600 baud, encode/decode round-trip

### Phase 3: DC Motors

| Component | Description |
|-----------|-------------|
| `EncoderCounter.h/cpp` | 2x/4x resolution encoder counting module |
| `VelocityEstimator.h/cpp` | Edge-time velocity calculation |
| `DCMotor.h/cpp` | Full cascade PID controller |

**Tests:**
- `tests/test_encoder.ino` - Verify counting accuracy (both directions)
- `tests/test_dc_motor_pwm.ino` - Direct PWM control
- `tests/test_dc_motor_pid.ino` - Position/velocity control loops

### Phase 4: Steppers & Servos

| Component | Description |
|-----------|-------------|
| `StepperMotor.h/cpp` | Acceleration profiles, limit switch support |
| `StepperManager.h/cpp` | Timer3 configuration, multi-stepper coordination |
| `ServoController.h/cpp` | PCA9685 I2C wrapper |

**Tests:**
- `tests/test_stepper.ino` - Step/direction control, acceleration
- `tests/test_servo.ino` - PCA9685 sweep test

### Phase 5: Sensors & I/O

| Component | Description |
|-----------|-------------|
| `IMUDriver.h/cpp` | ICM-20948 wrapper |
| `UltrasonicDriver.h/cpp` | I2C ultrasonic wrapper |
| `NeoPixelDriver.h/cpp` | Addressable LED wrapper |
| `SensorManager.h/cpp` | Sensor aggregation and scheduling |
| `UserIO.h/cpp` | Buttons, LEDs, status indication |

**Tests:**
- `tests/test_imu.ino` - 6-axis IMU reading
- `tests/test_voltage.ino` - ADC voltage monitoring
- `tests/test_buttons.ino` - Button/limit switch states
- `tests/test_leds.ino` - LED patterns and NeoPixel colors

### Phase 6: Integration

| Component | Description |
|-----------|-------------|
| Full `arduino.ino` | Complete system integration |
| Safety timeout | Verify heartbeat monitoring |
| End-to-end | RPi communication testing |

**Test:** `tests/test_full_system.ino` - All subsystems running together

---

## Test Sketches

All test sketches are in `firmware/tests/`. Each test is in its own folder (Arduino IDE requirement).

**Folder Structure:**
```
tests/
├── test_scheduler/
│   └── test_scheduler.ino
├── test_uart_tlv/
│   └── test_uart_tlv.ino
├── test_encoder/
│   └── test_encoder.ino
...
```

**Setup Test Environment (One-time):**

Each test folder needs symbolic links to share the src/ and lib/ directories with the main sketch:

```bash
# From firmware/ directory:
cd tests/test_scheduler
ln -s ../../arduino/src src
ln -s ../../arduino/lib lib
cd ../..

cd tests/test_uart_tlv
ln -s ../../arduino/src src
ln -s ../../arduino/lib lib
cd ../..
```

**Compile Tests:**

```bash
# From firmware/ directory:
arduino-cli compile --fqbn arduino:avr:mega tests/test_scheduler
arduino-cli compile --fqbn arduino:avr:mega tests/test_uart_tlv

# Upload test to Arduino Mega:
arduino-cli upload -p /dev/cu.usbserial* --fqbn arduino:avr:mega tests/test_scheduler
```

**Verified Compilation Results:**
- `test_scheduler`: 5590 bytes (2% flash), 445 bytes RAM (5%)
- `test_uart_tlv`: 10706 bytes (4% flash), 2754 bytes RAM (33%)
- `test_encoder`: 6292 bytes (2% flash), 515 bytes RAM (6%)
- `test_dc_motor_pwm`: 7058 bytes (2% flash), 501 bytes RAM (6%)
- `test_dc_motor_pid`: 13362 bytes (5% flash), 808 bytes RAM (9%)
- `arduino` (main): 15436 bytes (6% flash), 3410 bytes RAM (41%)

| Test Folder | Phase | Purpose |
|-------------|-------|---------|
| `test_scheduler/` | 1 | Timer interrupt timing accuracy |
| `test_uart_tlv/` | 2 | TLV encode/decode at 921600 baud |
| `test_encoder/` | 3 | Quadrature encoder counting |
| `test_dc_motor_pwm/` | 3 | Direct PWM motor control |
| `test_dc_motor_pid/` | 3 | PID position/velocity control |
| `test_stepper/` | 4 | Stepper step/dir/enable |
| `test_servo/` | 4 | PCA9685 servo sweep |
| `test_imu/` | 5 | ICM-20948 accel/gyro reading |
| `test_ultrasonic/` | 5 | I2C ultrasonic distance |
| `test_voltage/` | 5 | Battery/rail voltage ADC |
| `test_buttons/` | 5 | Button/limit switch states |
| `test_leds/` | 5 | User LEDs and NeoPixels |
| `test_full_system/` | 6 | Complete integration test |

Each test sketch includes:
- Clear header comment explaining what it tests
- Expected behavior description
- How to verify (visual, Serial Monitor, oscilloscope)
- Interactive commands where applicable

See individual test files for implementation details.

---

## Progress Tracking

> [!NOTE]
> Update this section as implementation progresses. Mark items with completion dates.

| Phase | Component | Status | Date | Notes |
|-------|-----------|--------|------|-------|
| **1** | `config.h` | ✅ | 2026-01-27 | Timing, enables, defaults |
| **1** | `pins.h` | ✅ | 2026-01-27 | Pin definitions from README |
| **1** | `Scheduler.h/cpp` | ✅ | 2026-01-27 | Timer1 @ 1kHz + task management |
| **1** | `arduino.ino` | ✅ | 2026-01-27 | Main structure with placeholders |
| **1** | `test_scheduler/` | ✅ | 2026-01-27 | Multi-LED blink test verified |
| **2** | `TLV_Payloads.h` | ✅ | 2026-01-27 | Packed structs for all message types |
| **2** | `TLV_TypeDefs.h` | ✅ | 2026-01-27 | Moved to messages/, C-compatible |
| **2** | `MessageCenter.h/cpp` | ✅ | 2026-01-27 | Complete refactor with TLV protocol |
| **2** | `tlvcodec integration` | ✅ | 2026-01-27 | Wrapper for Arduino IDE compilation |
| **2** | `test_uart_tlv/` | ✅ | 2026-01-27 | Communication quality stats added, CRC enabled |
| **3** | `EncoderCounter.h/cpp` | ✅ | 2026-01-27 | 2x/4x modes, interface + implementations |
| **3** | `VelocityEstimator.h/cpp` | ✅ | 2026-01-27 | Edge-time + pulse-count methods |
| **3** | `DCMotor.h/cpp` | ✅ | 2026-01-27 | Cascade PID, position/velocity modes |
| **3** | `test_encoder.ino` | ✅ | 2026-01-27 | 6292 bytes, encoder counting verified |
| **3** | `test_dc_motor_pwm.ino` | ✅ | 2026-01-27 | 7058 bytes, direct PWM control test |
| **3** | `test_dc_motor_pid.ino` | ✅ | 2026-01-27 | 13362 bytes, closed-loop PID test |
| **3** | `arduino.ino` integration | ✅ | 2026-01-27 | Phase 3 motors integrated, 15436 bytes |
| **4** | `StepperMotor.h/cpp` | ⬜ | | Acceleration support |
| **4** | `StepperManager.h/cpp` | ⬜ | | Timer3 management |
| **4** | `ServoController.h/cpp` | ⬜ | | PCA9685 wrapper |
| **4** | `test_stepper.ino` | ⬜ | | |
| **4** | `test_servo.ino` | ⬜ | | |
| **5** | `IMUDriver.h/cpp` | ⬜ | | ICM-20948 wrapper |
| **5** | `UltrasonicDriver.h/cpp` | ⬜ | | I2C ultrasonic |
| **5** | `NeoPixelDriver.h/cpp` | ⬜ | | LED library wrapper |
| **5** | `SensorManager.h/cpp` | ⬜ | | Aggregation |
| **5** | `UserIO.h/cpp` | ⬜ | | Buttons/LEDs |
| **5** | `test_imu.ino` | ⬜ | | |
| **5** | `test_voltage.ino` | ⬜ | | |
| **5** | `test_buttons.ino` | ⬜ | | |
| **5** | `test_leds.ino` | ⬜ | | |
| **6** | `arduino.ino` integration | ⬜ | | Full system |
| **6** | `test_full_system.ino` | ⬜ | | |
| **6** | End-to-end with RPi | ⬜ | | |

### Status Legend
- ⬜ Not Started
- 🔄 In Progress
- ✅ Complete
- ⚠️ Blocked
- ❌ Cancelled

---

## Design Decisions (Resolved)

| Question | Decision | Rationale |
|----------|----------|-----------|
| Encoder 4x Mode | Per-motor configuration | Flexibility for mixed hardware |
| Velocity Estimation | Timestamp in ISR, compute in PID loop | Keeps ISR minimal (~10 cycles) |
| Stepper Acceleration | Linear (trapezoidal) | Sufficient for this application |
| Debug Pin Toggles | Yes, on unused analog pins | Enables oscilloscope timing verification |

### Encoder Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Pulses per revolution | 1440 PPR | Already 4x counted (360 base CPR) |
| Maximum motor RPM | 100 RPM | Geared motor output shaft |
| Max edge rate (4x mode) | ~2400 edges/sec | 1440 × (100/60) = 2400 Hz |
| Max edge rate (2x mode) | ~1200 edges/sec | Half of 4x rate |
| ISR period at max speed | ~417 µs (4x) / ~833 µs (2x) | Plenty of margin |

**Conclusion:** At 100 RPM max, the encoder ISR rate is very manageable. Even with 4 motors in 4x mode running at max speed, total ISR rate would be ~9600/sec (~104 µs between interrupts on average), leaving ample time for other processing.

