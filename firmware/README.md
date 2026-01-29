# Arduino Firmware

This directory contains the Arduino firmware for low-level real-time control and sensor integration on a differential drive robot platform.
Arduino Mega 2560 is used as the microcontroller, acting as a slave device to a Raspberry Pi master via UART using a custom TLV (Type-Length-Value) protocol.

## Key features
1. The firmware uses the tlvcodec protocol over UART to communicate with the Raspberry Pi
2. Controls up to 4 DC motors with quadrature encoders (A/B phase) and current sensing (via ADC inputs)
3. Controls up to 4 stepper motors with STEP/DIR/ENABLE signals
4. I2C bus for Qwiic sensors and PCA9685 PWM driver for servos
5. User buttons, limit switches, user LEDs, and one status RGB LED (WS2812B NeoPixel, can be extended)
6. Battery voltage and 5V/servo rail voltage monitoring via ADC inputs

## Firmware Operation
- At startup, all sensors are initialized and all motors are disabled.
- After receiving the `MOTOR_ENABLE` command, the specified motors are enabled.
- The firmware continuously reads sensor data and sends it to the Raspberry Pi at a globally configurable update rate.
- The firmware listens for control commands (motor, stepper, servos, etc.) from the Raspberry Pi and executes them in real-time.
- **Safety timeout**: If no command is received from the Raspberry Pi for 50ms (default), all motors are automatically disabled. This timeout is configurable in the firmware header file.

## Sensors
- **Voltage monitors**:
    - Battery voltage (via resistor divider)
    - Servo rail voltage (via resistor divider)
    - Motor current sense (CT) for each DC motor (used for torque control)
- **Quadrature encoders**: A/B phase encoder for each DC motor
- **User buttons**: 10 on-board buttons with INPUT_PULLUP
    - *Note: 8 of these (BTN3-BTN10) share GPIO pins with limit switch inputs and can be configured as limit switches in firmware*
- **Qwiic-compatible sensors** (via I2C bus):
    - Supports IMU, ultrasonic rangefinder, environmental sensors, etc.
    - Codebase includes a template interface for adding new Qwiic sensors

## Motor Control

### DC Motors (4 channels)
- Controlled via PWM (EN pin) for speed and digital signals (IN1/IN2) for direction
- **Cascade PID control** with encoder and current feedback:
    - **Position → Velocity → Torque** control (full cascade, if current sensing is reliable)
    - **Position → Velocity** control (fallback option)
    - PID gains are **runtime configurable** via TLV commands

### Stepper Motors (4 channels)
- Supports A4988 and DRV8825 stepper drivers
- Controlled via STEP/DIR/ENABLE signals through external stepper driver modules
- **Open-loop control** (position tracked by step counting, no encoder feedback)
- Supports microstepping (configured on the driver module)
- Can be associated with limit switches for homing (configured in firmware header file)

### Servos (via PCA9685 I2C PWM driver)
- Simple command interface to set servo positions

## Configuration

Most firmware parameters are configured via a **header file** at compile time:
- Number of active motors, steppers, and servos
- Which sensors are enabled
- Safety timeout duration (default 50ms)
- Sensor update rate
- Stepper-to-limit-switch associations

**Runtime configurable** (via TLV commands):
- PID gains for DC motor control

## Threads
- **Main loop**: Housekeeping tasks and debug output to serial monitor (UART0)
- **UART thread**: Real-time communication with Raspberry Pi (UART2)
- **DC Motor control**: PID loop execution, PWM generation, encoder reading, current sensing
- **Stepper Motor control**: Step pulse generation with STEP/DIR/ENABLE signals
- **Servo control**: I2C communication with PCA9685
- **Sensor thread**: Periodic reading of all enabled sensors

## TLV Protocol

Commands are sent from Raspberry Pi (master) to Arduino (slave). Responses and sensor data are sent from Arduino to Raspberry Pi.

### SYSTEM `0x001-0x0FF`
| Command | Direction | Description |
|---------|-----------|-------------|
| `SYS_HEARTBEAT` | RPi → Arduino | Watchdog heartbeat. Must be sent within timeout period (default 50ms) to keep motors enabled. |
| `SYS_STATUS` | Arduino → RPi | Reports system status: enabled motors, error flags, voltage levels, uptime. |
| `SYS_SET_PID` | RPi → Arduino | Set PID gains for a specific DC motor (runtime configurable). |
| `SYS_GET_PID` | RPi → Arduino | Request current PID gains for a specific DC motor. |
| `SYS_RES_PID` | Arduino → RPi | Returns the requested PID gains. |

### MOTORS `0x100-0x3FF`
| Command | Direction | Description |
|---------|-----------|-------------|
| `DC_ENABLE` | RPi → Arduino | Enable/disable specific DC motors. Motors start disabled on boot. |
| `DC_SET_POSITION` | RPi → Arduino | Set target position for a DC motor. Implicitly switches to position control mode. Motor holds position until disabled. |
| `DC_SET_VELOCITY` | RPi → Arduino | Set target velocity for a DC motor. Implicitly switches to velocity control mode. |
| `DC_STATUS` | Arduino → RPi | Reports DC motor state: position (encoder ticks), velocity, current draw, control mode. |
| `STEP_ENABLE` | RPi → Arduino | Enable/disable specific stepper motors. Motors start disabled on boot. |
| `STEP_SET_ACCEL` | RPi → Arduino | Set acceleration/deceleration rate for a specific stepper motor (runtime configurable). |
| `STEP_SET_VEL` | RPi → Arduino | Set maximum velocity for a specific stepper motor (runtime configurable). |
| `STEP_MOVE` | RPi → Arduino | Command stepper to move a specified number of steps. Uses configured velocity/acceleration. |
| `STEP_HOME` | RPi → Arduino | Command stepper to home using associated limit switch. |
| `STEP_STATUS` | Arduino → RPi | Reports stepper state: current position (step count), moving/idle, limit switch triggered. |
| `SERVO_ENABLE` | RPi → Arduino | Enable/disable the PCA9685 servo driver. Starts disabled on boot. |
| `SERVO_SET` | RPi → Arduino | Set servo position (pulse width or angle) for a specific channel. |

### SENSORS `0x400-0x4FF`
Sensor data is sent as separate packets grouped by sensor type. If multiple sensors of the same type are connected (e.g., two IMUs), their data is combined into a single packet.

| Command | Direction | Description |
|---------|-----------|-------------|
| `SENSOR_VOLTAGE` | Arduino → RPi | Battery voltage, servo rail voltage, 5V rail status. |
| `SENSOR_ENCODER` | Arduino → RPi | Encoder data for all DC motors: position (ticks), velocity (ticks/sec). |
| `SENSOR_CURRENT` | Arduino → RPi | Motor current readings (CT) for all DC motors. |
| `SENSOR_IMU` | Arduino → RPi | IMU data (if connected): acceleration, gyroscope, orientation. Multiple IMUs in one packet. |
| `SENSOR_RANGE` | Arduino → RPi | Ultrasonic rangefinder distance readings (if connected). Multiple sensors in one packet. |

### USER IO `0x500-0x5FF`
| Command | Direction | Description |
|---------|-----------|-------------|
| `IO_SET_LED` | RPi → Arduino | Set state of user LEDs (blue, orange, purple). Supports on/off, PWM brightness, and blink/breathing patterns. |
| `IO_SET_NEOPIXEL` | RPi → Arduino | Set NeoPixel RGB LED color and patterns. The fisrt pixel is the top-most pixel (ID 0). The user can still control the first pixel, but its behavior will be overridden by the system status (e.g. red when battery is low). |
| `IO_BUTTON_STATE` | Arduino → RPi | Reports current state of all 10 user buttons (active low). |
| `IO_LIMIT_STATE` | Arduino → RPi | Reports state of limit switches (only pins configured as limit switch mode). |

## GPIO Pin Assignment Table
**Exposed pins are available on screw terminals/headers for reuse; core comms and default wheel drive pins remain internal.**

| Pin(s) | Pin Name | Function | Exposed? | Notes |
|--------|----------|----------|----------|-------|
| 0 (RX0) | RX0 | USB Serial | No | Programming/debug only |
| 1 (TX0) | TX0 | USB Serial | No | Programming/debug only |
| 2 (INT0) | M1_ENC_A | Motor 1 Encoder A | No | Default left/right wheel encoder A |
| 3 (INT1) | M2_ENC_A | Motor 2 Encoder A | No | Default left/right wheel encoder A |
| 4 | M1_ENC_B | Motor 1 Encoder B | No | Default wheel encoder B |
| 5 (PWM) | M1_EN | Motor 1 EN | No | Default wheel PWM |
| 6 (PWM) | M2_EN | Motor 2 EN | No | Default wheel PWM |
| 7 | M2_ENC_B | Motor 2 Encoder B | No | Default wheel encoder B |
| 8 | M1_IN1 | Motor 1 IN1 | No | Default wheel direction |
| 9 (PWM) | M3_EN | Motor 3 EN | Yes | PWM-capable |
| 10 (PWM) | M4_EN | Motor 4 EN | Yes | PWM-capable |
| 11 (PWM) | LED_RED | Status LED Red | No | Error/low battery indicator |
| 12 | M2_IN1 | Motor 2 IN1 | No | Default wheel direction |
| 13 | M2_IN2 | Motor 2 IN2 | No | Default wheel direction |
| 14 | ST1_STEP | Stepper 1 STEP | Yes | Stepper control |
| 15 | ST2_STEP | Stepper 2 STEP | Yes | Stepper control |
| 16 (TX2) | TX_RPI | **UART to RPi5** | No | Via level shifter (5V → 3.3V) |
| 17 (RX2) | RX_RPI | **UART from RPi5** | No | Via level shifter (3.3V → 5V) |
| 18 (INT5) | M3_ENC_A | Motor 3 Encoder A | Yes | Interrupt-capable |
| 19 (INT4) | M4_ENC_A | Motor 4 Encoder A | Yes | Interrupt-capable |
| 20 (SDA) | SDA | I2C Data | Yes | Qwiic + PCA9685 module |
| 21 (SCL) | SCL | I2C Clock | Yes | Qwiic + PCA9685 module |
| 22 | ST1_DIR | Stepper 1 DIR | Yes | Stepper direction |
| 23 | ST2_DIR | Stepper 2 DIR | Yes | Stepper direction |
| 24 | ST3_DIR | Stepper 3 DIR | Yes | Stepper direction |
| 25 | ST4_DIR | Stepper 4 DIR | Yes | Stepper direction |
| 26 | ST1_EN | Stepper 1 ENABLE | Yes | Individual enable |
| 27 | ST2_EN | Stepper 2 ENABLE | Yes | Individual enable |
| 28 | ST3_EN | Stepper 3 ENABLE | Yes | Individual enable |
| 29 | ST4_EN | Stepper 4 ENABLE | Yes | Individual enable |
| 30 | M3_ENC_B | Motor 3 Encoder B | Yes | Quadrature input |
| 31 | M4_ENC_B | Motor 4 Encoder B | Yes | Quadrature input |
| 32 | ST3_STEP | Stepper 3 STEP | Yes | Stepper control |
| 33 | ST4_STEP | Stepper 4 STEP | Yes | Stepper control |
| 34 | M3_IN1 | Motor 3 IN1 | Yes | Direction |
| 35 | M3_IN2 | Motor 3 IN2 | Yes | Direction |
| 36 | M4_IN1 | Motor 4 IN1 | Yes | Direction |
| 37 | M4_IN2 | Motor 4 IN2 | Yes | Direction |
| 38 | BTN1 | User Button 1 | No | On-board only, INPUT_PULLUP |
| 39 | BTN2 | User Button 2 | No | On-board only, INPUT_PULLUP |
| 40 | LIM1 / BTN3 | Limit Switch 1 / Button 3 | Yes | JST XH 3-pin (V, S, G) |
| 41 | LIM2 / BTN4 | Limit Switch 2 / Button 4 | Yes | JST XH 3-pin (V, S, G) |
| 42 | NEOPIXEL_DIN | WS2812B RGB LED Data | No | NeoPixel control |
| 43 | M1_IN2 | Motor 1 IN2 | No | Default wheel direction |
| 44 (PWM) | LED_GREEN | Status LED Green | No | System OK (default) |
| 45 (PWM) | LED_BLUE | User LED Blue | Yes | Exposed for user |
| 46 (PWM) | LED_ORANGE | User LED Orange | Yes | Exposed for user |
| 47 | LED_PURPLE | User LED Purple | Yes | Exposed for user (non-PWM) |
| 48 | LIM3 / BTN5 | Limit Switch 3 / Button 5 | Yes | JST XH 3-pin (V, S, G) |
| 49 | LIM4 / BTN6 | Limit Switch 4 / Button 6 | Yes | JST XH 3-pin (V, S, G) |
| 50 | LIM5 / BTN7 | Limit Switch 5 / Button 7 | Yes | JST XH 3-pin (V, S, G) |
| 51 | LIM6 / BTN8 | Limit Switch 6 / Button 8 | Yes | JST XH 3-pin (V, S, G) |
| 52 | LIM7 / BTN9 | Limit Switch 7 / Button 9 | Yes | JST XH 3-pin (V, S, G) |
| 53 | LIM8 / BTN10 | Limit Switch 8 / Button 10 | Yes | JST XH 3-pin (V, S, G) |
| A0 | VBAT_SENSE | Battery Voltage Monitor | No | Divider 1:6 on BAT_IN |
| A1 | V5_SENSE | 5V Rail Monitor | No | Divider 1:2 after 5V buck |
| A2 | VSERVO_SENSE | Servo Rail Monitor | No | Divider 1:3 servo rail |
| A3 | M1_CT | Motor 1 Current Sense (CT) | No | H-bridge feedback |
| A4 | M2_CT | Motor 2 Current Sense (CT) | No | H-bridge feedback |
| A5 | M3_CT | Motor 3 Current Sense (CT) | Yes | H-bridge feedback |
| A6 | M4_CT | Motor 4 Current Sense (CT) | Yes | H-bridge feedback |
| A7-A15 | ANALOG_EXP | Analog Expansion | Yes | Available for sensors |

**DC Motor Control Summary (per motor):**
| Motor | EN (PWM) | IN1 | IN2 | Encoder A | Encoder B | Current (CT) |
|-------|----------|-----|-----|-----------|-----------|--------------|
| 1 | Pin 5 | Pin 8 | Pin 43 | Pin 2 (INT0) | Pin 4 | A3 |
| 2 | Pin 6 | Pin 12 | Pin 13 | Pin 3 (INT1) | Pin 7 | A4 |
| 3 | Pin 9 | Pin 34 | Pin 35 | Pin 18 (INT5) | Pin 30 | A5 |
| 4 | Pin 10 | Pin 36 | Pin 37 | Pin 19 (INT4) | Pin 31 | A6 |

**Hardware Interrupts Used:**
- INT0 (pin 2): Motor 1 Encoder A
- INT1 (pin 3): Motor 2 Encoder A
- INT4 (pin 19): Motor 4 Encoder A
- INT5 (pin 18): Motor 3 Encoder A

**Serial Ports:**
- Serial0 (pins 0/1): USB programming/debug
- Serial2 (pins 16/17): Raspberry Pi communication via level shifter
- Serial1 (pins 18/19): NOT AVAILABLE (used for encoder interrupts)
- Serial3 (pins 14/15): NOT AVAILABLE (used for stepper STEP signals)

## Know issues
### Stepper motor
- When a stepper is disabled in config.h, the StepperMotor won't be able show the user that it's disabled in the config.h. 
- 


