# The NUEVO Board - User Guide
**Navigation Unit for Education and Versatile Operations**

**Board Revision:** Rev. B
**Design Status:** Designed and fabricated (awaiting delivery and testing)
**Last Updated:** 2026-02-13

---

## Overview

The NUEVO Board is the main controller board for the MAE 162 educational robotics platform. It integrates power management, motor control interfaces, and communication between the Arduino Mega 2560 microcontroller and Raspberry Pi 5.

**Design Philosophy:** This board uses a modular architecture with off-the-shelf motor driver and PWM modules. The custom PCB focuses on robust power management, clean signal interfaces, and educational accessibility.

**Key Features:**
- Dual DC-DC buck converters (5V for logic, adjustable 5-10V for servos)
- Support for 4 DC motors with quadrature encoders (full 4x resolution)
- Support for 4 stepper motors (A4988/DRV8825 driver sockets)
- Support for up to 16 servos (via external PCA9685 module)
- Arduino ↔ Raspberry Pi UART communication with level shifting
- 6 Qwiic connectors for I2C sensor expansion (4 Arduino, 2 RPi)
- 10 user buttons / limit switches
- RGB LED strip support (WS2812B)

---

## Quick Specifications

### Power System
- **Input:** 12V NiMH battery (default), supports 7.4V-24V (via XT60 connector)
- **5V Rail:** Up to 6A (Raspberry Pi 5, Arduino, sensors, encoders)
- **Servo Rail:** Adjustable 5-10V @ 6A (potentiometer adjustable, default ~6V)
- **Protection:** 15A replaceable fuse, reverse polarity protection, power switch

### Motor Control (External Modules)
- **DC Motors:** 4 motors via 2 dual-channel H-bridge modules (external)
  - Encoder support: 4x quadrature resolution on all motors
  - Current sensing on all channels
- **Stepper Motors:** 4 sockets for A4988/DRV8825 driver modules (on-board)
- **Servo Motors:** Up to 16 servos via PCA9685 I2C PWM module (external)

### Communication
- **Arduino ↔ Raspberry Pi:** UART Serial2 (pins 16/17) with 5V ↔ 3.3V level shifting
- **I2C Expansion:** 6 Qwiic connectors (JST-SH 4-pin)
  - 4 on Arduino bus (voltage-selectable: 5V or 3.3V per bank)
  - 2 on Raspberry Pi bus (3.3V only)

### User Interface
- **LEDs:** 5 status/user LEDs (red, green, blue, orange, purple)
- **RGB LED:** 1 on-board WS2812B + extension socket for external strips
- **Buttons:** 2 dedicated on-board buttons + 8 dual-purpose button/limit switch inputs
- **Power Indicators:** 5V rail LED, servo rail LED, power-good LED

---

## Getting Started

### 1. Initial Inspection

**Before powering up:**
- [ ] Visually inspect PCB for damage, solder bridges, or loose components
- [ ] Verify all module connections are secure (H-bridge, stepper drivers, PCA9685)
- [ ] Check fuse is properly seated (15A blade fuse)
- [ ] Ensure power switch is in OFF position

### 2. Power Requirements

**Battery:**
- **Recommended:** 12V NiMH battery pack (10-14.4V range)
- **Supported:** 7.4V-24V (2S to 6S LiPo, or equivalent)
- **Connector:** XT60 male on battery cable (female on PCB)
- **Capacity:** Minimum 2000mAh recommended for typical operation

**⚠️ IMPORTANT - H-Bridge Voltage Limit:**
- Default H-bridge modules (DC5-12V 30A Dual Channel) support **5-12V maximum**
- If using batteries >12V (e.g., 4S-6S LiPo), replace with higher-voltage modules (e.g., BTS7960 for 24V)
- Always match H-bridge voltage rating to your battery voltage

### 3. First Power-Up Checklist

**Step-by-step:**
1. **Install fuse:** Insert 15A blade fuse into holder
2. **Connect battery:** Plug in XT60 connector (observe polarity - reverse polarity protection is active)
3. **Set servo voltage:** Adjust potentiometer if needed (default ~6V is safe for most servos)
4. **Power ON:** Flip main power switch
5. **Check LEDs:**
   - 5V power indicator (green) should illuminate
   - Servo rail power indicator (green) should illuminate
   - Power-good LED should illuminate (indicates 5V buck is active)
6. **Verify voltages:**
   - Measure 5V rail with multimeter: Should read 4.9-5.1V
   - Measure servo rail: Should match potentiometer setting (5-10V)
7. **Upload firmware:** Connect Arduino via USB and upload test firmware
8. **Check ROS2 communication:** Power Raspberry Pi 5 and test UART bridge

---

## Board Layout & Connectors

### Power Connectors

| Connector | Type | Location | Function |
|-----------|------|----------|----------|
| **J_BAT** | XT60 female | Left edge | Battery input (7.4V-24V) |
| **SW1** | Toggle/rocker switch | Left edge | Main power ON/OFF |
| **F1** | Blade fuse holder | Near battery input | 15A replaceable fuse |
| **POT_SERVO** | Trimmer potentiometer | Center area | Servo rail voltage adjustment (5-10V) |

### Motor Connectors

**DC Motors (H-Bridge Module Interface):**
- **J_M1_CTRL**, **J_M2_CTRL**: Motor 1/2 control headers (EN, IN1, IN2, CT pins)
- **J_M3_CTRL**, **J_M4_CTRL**: Motor 3/4 control headers (EN, IN1, IN2, CT pins)
- **J_M1_ENC** to **J_M4_ENC**: 4-pin encoder headers (VCC, GND, A, B)

**Stepper Motors (On-Board Driver Sockets):**
- **U_ST1** to **U_ST4**: A4988/DRV8825 driver module sockets
- **JP_MS1_1** to **JP_MS3_4**: Microstepping jumpers (3 per driver × 4 drivers = 12 jumpers)
- **J_ST1_MOTOR** to **J_ST4_MOTOR**: 4-pin motor connectors (A+, A-, B+, B-)

**Servo Motors (PCA9685 Interface):**
- **J_PCA9685**: 4-pin header (VCC, GND, SDA, SCL) connects to external PCA9685 module
- Servos plug into PCA9685 module, not directly into main PCB

### Communication Connectors

**Qwiic I2C (Arduino Bus - 3.3V, Level-Shifted):**
- **J_QWIIC_1** to **J_QWIIC_4**: 4 Qwiic connectors (JST-SH 4-pin, 1mm pitch)
- All connectors on Arduino I2C bus (pins 20/21 SDA/SCL)
- **Voltage:** 3.3V (compatible with all Qwiic/STEMMA QT devices)
- **Level shifting:** TXS0102 translates Arduino 5V ↔ Qwiic 3.3V automatically
- **Pin definition** (for custom cables):
  - Pin 1: GND (black wire)
  - Pin 2: 3.3V (red wire)
  - Pin 3: SDA (blue wire)
  - Pin 4: SCL (yellow wire)

**Qwiic I2C (Raspberry Pi Bus - 3.3V Native):**
- **J_QWIIC_RPI1**, **J_QWIIC_RPI2**: 2 connectors on RPi I2C bus (GPIO2/GPIO3)
- **Voltage:** 3.3V (no level shifting needed)
- Same pin definition as Arduino Qwiic connectors

**Arduino ↔ Raspberry Pi UART:**
- Internal connection via level shifter (not externally exposed)
- Arduino Serial2 (pins 16/17) ↔ RPi GPIO14/GPIO15

### User Interface Connectors

**Buttons (On-Board):**
- **BTN1**, **BTN2**: Dedicated user buttons (pins 38, 39)

**Dual-Purpose Button/Limit Switch Inputs:**
- **BTN3/LIM1** to **BTN10/LIM8**: 8 dual-purpose inputs (pins 40, 41, 48-53)
- Each has on-board tactile button + 3-pin JST XH connector (V, S, G)
- **JP_LIM_V**: 3-pin jumper selects VCC for all limit switch connectors (5V or 3.3V)

**LEDs:**
- **LED1** (Green): System OK indicator (pin 44, PWM)
- **LED2** (Red): Error/low battery indicator (pin 5, digital only - no PWM)
- **LED3** (Blue): User LED (pin 45, PWM, exposed)
- **LED4** (Orange): User LED (pin 46, PWM, exposed)
- **LED5** (Purple): User LED (pin 47, digital only, exposed)

**RGB LED:**
- **D_RGB1**: On-board WS2812B LED (pin 42)
- **J_NEOPIXEL**: 3-pin extension socket (DIN, 5V, GND) for external LED strips

---

## Configuration & Jumpers

### 1. Servo Power Jumper (PCA9685)

**JP_SERVO_PWR (PCA9685 Power Enable):**
- **Jumper CLOSED (installed):** Servo rail power connected directly to PCA9685 VCC
  - **Benefit:** No external power cable needed for PCA9685 module
  - **Use case:** Standard setup when using on-board servo power
- **Jumper OPEN (removed):** PCA9685 powered externally
  - **Use case:** When using separate servo power supply or troubleshooting

**Default:** Jumper CLOSED (most common configuration)

**⚠️ Note:** The servo rail voltage is adjustable (5-10V via POT_SERVO). Ensure PCA9685 and connected servos can handle the selected voltage before enabling this jumper.

### 2. Limit Switch Voltage Selection

**JP_LIM_V (All 8 limit switch connectors share this setting):**
- **Position 1-2 (5V):** Use for 5V sensors (mechanical switches, 5V optical endstops)
- **Position 2-3 (3.3V):** Use for 3.3V sensors (some Hall effect, low-voltage proximity sensors)

**Default:** 5V (suitable for mechanical switches and most common optical endstops)

### 3. Stepper Microstepping Configuration

**Each stepper driver has 3 jumpers (MS1, MS2, MS3):**

**Microstepping Table (A4988 and DRV8825):**

| JP_MS1 | JP_MS2 | JP_MS3 | A4988 Mode | DRV8825 Mode |
|--------|--------|--------|------------|--------------|
| Open   | Open   | Open   | Full step  | Full step    |
| Closed | Open   | Open   | 1/2 step   | 1/2 step     |
| Open   | Closed | Open   | 1/4 step   | 1/4 step     |
| Closed | Closed | Open   | 1/8 step   | 1/8 step     |
| Closed | Closed | Closed | 1/16 step  | 1/32 step    |

**Recommended Settings:**
- **1/16 microstepping (A4988):** Smooth motion, good for most applications
- **1/8 microstepping:** Balance between smoothness and speed
- **Full step:** Maximum speed, used for rapid positioning

**Configuration:**
- Jumper CLOSED = Insert jumper cap (shorted)
- Jumper OPEN = Remove jumper cap

### 4. Servo Rail Voltage Adjustment

**POT_SERVO (Trimmer potentiometer):**
- **Range:** 5V to 10V (adjustable)
- **Default:** ~6V (set at factory, safe for standard servos)
- **Adjustment:** Turn clockwise to increase voltage, counterclockwise to decrease

**How to Set:**
1. Power on the board
2. Measure voltage at servo rail test point (or PCA9685 VCC) with multimeter
3. Adjust potentiometer slowly until desired voltage is reached
4. Common settings:
   - **5-6V:** Standard analog servos
   - **6-7.4V:** High-torque servos
   - **7.4-8.4V:** Digital servos (check servo specs)

**⚠️ WARNING:** Do not exceed servo voltage rating! Most standard servos are rated for 4.8V-6V.

---

## Usage Guidelines

### Power Budget and Current Limits

**15A Fuse (Default):**
- Adequate for typical operation (2-3 motors running, servos idle)
- **Peak currents** (all motors stalled) can exceed 15A and blow fuse
- Replace fuse with 20A if running heavy-duty applications

**Staggered Motor Startup:**
- Avoid starting all motors simultaneously (reduces inrush current)
- Firmware should implement sequential startup where possible

### H-Bridge Voltage Compatibility

**Default H-Bridge Module (DC5-12V 30A Dual Channel):**
- **Maximum:** 12V (do NOT use with batteries >12V)
- **Minimum:** 5V (battery voltage directly powers motors)

**If using >12V batteries:**
- Replace with BTS7960 H-bridge modules (5-27V rating)
- Or use voltage regulator to step down battery voltage to motor supply

### Arduino Pin Usage Notes

**Serial Ports:**
- **Serial0 (USB):** Programming and debug only
- **Serial2 (pins 16/17):** Reserved for Raspberry Pi communication (do NOT use for other purposes)
- **Serial1 (pins 18/19):** NOT AVAILABLE (used for M2 encoder interrupts)
- **Serial3 (pins 14/15):** NOT AVAILABLE (used for stepper STEP signals)

**PWM Limitations:**
- **Pin 5 (LED_RED):** Digital ON/OFF only (no PWM or breathing - Timer 3 conflict with steppers)
- **Pin 47 (LED_PURPLE):** Digital ON/OFF only (not PWM-capable)

**Interrupt Pins (Rev. B):**
- **INT0 (pin 2):** M1_ENC_A (highest priority)
- **INT1 (pin 3):** M1_ENC_B
- **INT4 (pin 19):** M2_ENC_B
- **INT5 (pin 18):** M2_ENC_A
- **PCINT (pins 11, 12):** M4_ENC_A, M4_ENC_B
- **PCINT (A14, A15):** M3_ENC_A, M3_ENC_B

### Encoder Configuration (Firmware)

**4x Quadrature Mode (All Motors):**
- Motors 1 and 2 (wheels): Full hardware interrupts (INT0-INT5)
- Motors 3 and 4 (manipulators): Pin Change Interrupts (PCINT)
- All motors support full 4x resolution (1440 PPR → 5760 counts/rev)

**Encoder Power:**
- All encoders powered from 5V rail (~20mA each)
- Encoder VCC pins have 100nF decoupling capacitors

---

## Troubleshooting

### Power Issues

**No LEDs illuminate when power switch is ON:**
- Check fuse is properly seated and not blown (visual inspection)
- Verify battery voltage at XT60 connector with multimeter (should be 7.4V-24V)
- Check power switch is in correct position
- Measure voltage after fuse (should match battery voltage)

**5V power indicator OFF, but battery voltage present:**
- 5V buck converter may have shut down due to overcurrent or thermal protection
- Disconnect all loads (unplug Raspberry Pi, modules)
- Power cycle and check if 5V LED illuminates
- If still off, buck converter may be damaged

**Servo rail indicator OFF:**
- Check servo rail buck converter enable pin (should be pulled high)
- Measure input voltage to buck converter (should match battery)
- Adjust potentiometer and verify output changes (5-10V range)

**Power-good LED behavior:**
- **ON:** 5V power from buck converter (normal)
- **OFF:** 5V backfed from Raspberry Pi USB-C (ideal diode blocking buck)
- **Flickering:** Potential voltage conflict or unstable 5V rail

### Motor Control Issues

**DC motors not responding:**
- Check H-bridge module power connections (V+, GND from battery rail)
- Verify Arduino PWM and direction pins are connected to correct H-bridge inputs
- Measure voltage at H-bridge EN pin (should be PWM signal, 0-5V)
- Test motor directly with bench power supply to rule out motor failure

**Encoders not counting:**
- Check encoder power (5V, GND) at encoder connector
- Verify encoder signal wires (A, B) are connected to correct Arduino pins
- Use oscilloscope to verify encoder generates pulses when motor shaft rotates
- Check firmware ISR handlers are enabled for correct pins

**Steppers not moving:**
- Verify stepper driver module is properly seated in socket
- Check stepper motor coil connections (A+, A-, B+, B-) are correct
- Measure voltage at driver STEP pin (should see pulses when moving)
- Check driver ENABLE pin is LOW (active low enable)
- Verify motor power (VMOT) is present on driver module

### Communication Issues

**Arduino-Raspberry Pi UART not working:**
- Check level shifter power (5V on Arduino side, 3.3V on RPi side)
- Verify UART pins: Arduino TX2 (16) → RPi RXD (GPIO15), Arduino RX2 (17) ← RPi TXD (GPIO14)
- Test with loopback: Short TX to RX on one side and send test data
- Check baud rate matches on both sides (115200 or 230400)

**I2C devices not detected:**
- Verify correct Qwiic voltage jumper setting (5V or 3.3V) for your device
- Check I2C pull-up resistors are present (should measure ~2.5V on idle bus)
- Use I2C scanner sketch to detect device addresses
- Verify SDA/SCL are not swapped

**Qwiic connectors not working:**
- Check which bank (A or B) and which bus (Arduino or RPi)
- Arduino banks A and B are on same I2C bus (pins 20/21) - voltage jumpers only affect level shifting
- RPi Qwiic connectors are separate bus (GPIO2/3) - always 3.3V

### LED/Button Issues

**Status LEDs not working:**
- Check Arduino pin assignments in firmware match PCB (see pin table)
- For LED_RED (pin 5): Digital ON/OFF only, PWM disabled due to Timer 3 conflict
- Measure voltage at LED pin (should be 0V when OFF, 5V when ON)

**User buttons not responding:**
- Verify INPUT_PULLUP mode is enabled in firmware
- Check button pins read HIGH when not pressed, LOW when pressed
- Test with multimeter: Measure pin voltage (should be ~5V idle, ~0V pressed)

**Limit switches triggering incorrectly:**
- Check limit switch voltage jumper (JP_LIM_V) matches sensor requirements
- Verify limit switch type (normally-open vs normally-closed)
- Adjust firmware pin mode (INPUT_PULLUP for most sensors)

### Thermal Issues

**Buck converters overheating:**
- Normal: Converters may reach 60-80°C under load (warm to touch)
- Excessive: >100°C indicates problem (too hot to touch)
- Reduce load current or add heatsinks to converter ICs
- Verify thermal vias are connected to bottom ground plane

---

## Safety Warnings

⚠️ **IMPORTANT SAFETY INFORMATION:**

1. **Never hot-plug motors or high-current connections** while board is powered
2. **Always verify voltage settings** before connecting new modules
3. **Do not exceed 15A continuous draw** without upgrading fuse and verifying trace capacity
4. **Ensure proper ventilation** for DC-DC converters under heavy load
5. **Use appropriate wire gauge** for motor power (≥18 AWG for high-current motors)
6. **Keep exposed conductors away from metal chassis** to prevent shorts

---

## About the Name

**NUEVO** = **N**avigation **U**nit for **E**ducation and **V**ersatile **O**perations

The NUEVO Board is designed as a flexible, modular platform for teaching robotics concepts while supporting diverse project configurations - from wheeled robots to manipulator arms.

---

## Additional Resources

- **Technical Specifications:** See `SPECIFICATIONS.md` for complete electrical and mechanical details
- **Pin Assignments:** Detailed pin tables in `SPECIFICATIONS.md` (Rev. A and Rev. B)
- **Firmware Guide:** See `../firmware/README.md` for Arduino code examples and configuration
- **ROS2 Integration:** See `../ros2_ws/README.md` for Raspberry Pi setup and UART protocol
- **Schematic & Layout:** See `schematic/` and `layout/` folders for PDF exports

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| **Rev. B** | 2026-02 | Full 4x encoder support on all motors, relocated pins for interrupt optimization, Timer 3 conflict resolved |
| **Rev. A** | 2025-12 | Initial production design |

---

**For technical support or questions, contact the MAE 162 instructional team.**
