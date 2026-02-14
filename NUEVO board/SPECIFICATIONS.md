# The NUEVO Board - Technical Specifications
**Navigation Unit for Education and Versatile Operations**

**Project:** MAE 162 Educational Robotics Platform
**Board Revision:** Rev. B (Production)
**Design Tool:** EasyEDA (online schematic/layout tool)
**Design Status:** Complete - Fabricated and awaiting delivery
**Last Updated:** 2026-02-13

---

## Design Philosophy

This PCB maximizes use of off-the-shelf modules for educational value, ease of troubleshooting, and design flexibility. The custom board focuses on:
- **Robust power management** (dual buck converters with thermal management)
- **Microcontroller integration** (Arduino Mega 2560 + Raspberry Pi 5)
- **Clean module interfaces** (external H-bridges, stepper drivers, servo controllers)
- **Educational accessibility** (clear labeling, test points, modular troubleshooting)

---

## Revision History

| Revision | Date | Status | Key Changes |
|----------|------|--------|-------------|
| **Rev. B** | 2026-02 | Production | Full 4x encoder support, pin relocations for interrupt optimization, Timer 3 conflict resolution, finalized power budget |
| **Rev. A** | 2025-12 | Initial prototype | First production design with modular architecture |

---

## Document Purpose

This document contains **complete technical specifications** for the MAE 162 custom PCB, including detailed electrical design, component selections, pin assignments, and design rationale.

**For operational information** (how to use the board, power-up procedures, configuration, troubleshooting), see **[README.md](README.md)** - the user guide.

**Target Audience:** PCB designers, firmware developers, teaching assistants, and anyone needing detailed technical reference.

---

## 1. Power System

### 1.1 Power Input
- **Battery Input:** Via XT60 connector
- **Voltage Range:** 12V Nimh by default. Supports up to 24V power inputs. (~30V tolerance for the whole PCB) 
- **Reverse Polarity Protection:** P-MOSFET circuit
  - **MOSFET:** IRF4905 (P-channel, TO-220)
  - **Specs:** 55V, 74A, Rdson ~20mΩ (only ~0.2V drop at 10A)
  - **Circuit:** Source → Battery+, Drain → Protected rail, Gate → GND via 10kΩ
  - **Gate protection:** 15V zener diode from Gate to Source and 100K across Gate and Source
- **Replaceable Fuse:** Automotive blade fuse holder (15A rating)
- **Main Power Switch:** High-current toggle/rocker switch (≥15A)

### 1.2 Regulators (On-Board)
**Primary 5V Buck (RPi + logic):**
- LM61460AASQRJRRQ1, 5V @ up to 6A; powers RPi 5, Arduino 5V, logic. Keep EN/PGOOD pinned out; add copper pour + thermal vias under pad (2oz copper recommended).

**Servo Buck (adjustable):**
- LM61460AASQRJRRQ1, adjustable via FB network @ up to 6A for PCA9685 + servos. Mirror thermal treatment of 5V buck.
- **⚠️ CRITICAL: LM61460-Q1 uses 1V feedback reference (NOT 0.8V)**
  - Datasheet confirmation: Page 21 Section 9.3.11 and Page 35 Section 10.2.2.2 explicitly state "1-V reference voltage"
- **Feedback formula:** Vout = 1.0V × (1 + Rupper/Rlower)
  - For adjustable 5V-10V range: Rupper ≈ 68kΩ to Vout, Rlower = 6.8kΩ + 0–10kΩ potentiometer to GND
  - Voltage range calculation:
    - Minimum (pot at 0Ω): Vout = 1V × (1 + 68kΩ/6.8kΩ) = 1V × 11 = 11V
    - Maximum (pot at 10kΩ): Vout = 1V × (1 + 68kΩ/16.8kΩ) = 1V × 5.05 = 5.05V
  - Default ~6V: Set pot to ~7.3kΩ (Rlower total = 14.1kΩ)

**3.3V Sensor LDO:**
- AMS1117-3.3. ≥500mA. Input: 5V rail. Output: 3.3V for Qwiic/STEMMA sensors and level-shifter Vref. Decouple with 10µF in/out + 0.1µF close to pins; optional 3.3V indicator LED.

**Thermal Notes:** 2oz copper on power traces, thermal vias (0.3mm) under buck pads, bottom copper pour as heatsink; allow optional stick-on heatsinks.

### 1.3 Rail Monitoring (ADC)
- **Battery (BAT_IN):** A0 after fuse + reverse FET; divider 1:6 (R1=50kΩ to BAT_IN, R2=10kΩ to GND). 100nF across R2. Red LED (pin 11) used for low-battery alert.
- **5V Rail:** A1; divider 1:2 (R1=10kΩ to 5V, R2=10kΩ to GND). 100nF across R2. Exposed rail sensed post-buck.
- **Servo Rail (adjustable ~4.7–10.4V, default ~6V):** A2; divider 1:3 (R1=20kΩ to servo rail, R2=10kΩ to GND). 100nF across R2 to keep ADC <5V even at the max setting.
- Place dividers close to their rails with short returns to ground pour; route to ADC with minimal coupling to high-current paths.

**Voltage Calculation (firmware):**
- Battery: `Vbat = ADC * (5.0 / 1023) * 6.0`
- 5V rail: `V5 = ADC * (5.0 / 1023) * 2.0`
- Servo rail: `Vservo = ADC * (5.0 / 1023) * 3.0`

### 1.4 Power Distribution
- **5V Rail:** Arduino logic, sensors, I2C pullups, LEDs, RGB LEDs, encoders, RPi 5
- **3.3V Rail:** Raspberry Pi GPIO level (via level shifters) **and** dedicated sensor rail from on-board 3.3V LDO for Qwiic when selected
- **Servo Rail (adjustable ~4.7–10.4V, default ~6V):** PCA9685 + servos (ensure attached hardware is rated for chosen voltage)
- **Motor Rail:** Battery voltage passthrough to H-bridge modules (5-12V)

### 1.5 Raspberry Pi 5 Power
**Power Source:** 5V rail via GPIO header pins (5V and GND)
- **Current:** Up to 5A (RPi 5 max draw)

**Ideal Diode Circuit (Backfeed Protection):**
- When RPi USB-C power is connected, it will supply 5V to the system and stops the 5V from the buck converter. 
- Prevents conflict between buck converter and USB-C power sources
- **Circuit:** One P-MOSFET in ideal diode configuration
  - From buck converter 5V output → 5V rail

- **Benefit:** Allows development/debugging with just USB-C, no battery needed

### 1.6 Power Budget (Default Configuration)

**5V Rail (from primary buck converter, 6A max):**
| Load | Typical | Peak | Notes |
|------|---------|------|-------|
| Raspberry Pi 5 | 3.0A | 5.0A | Depends on CPU load, peripherals |
| Arduino Mega 2560 | 0.1A | 0.2A | Via 5V pin, not USB |
| Encoders (×4) | 0.08A | 0.1A | ~20mA each |
| Qwiic sensors | 0.05A | 0.1A | Varies by sensor |
| LEDs + WS2812B | 0.1A | 0.15A | 4 LEDs + 1 RGB |
| Level shifter, pullups | 0.02A | 0.02A | Minimal |
| **5V Rail Total** | **3.35A** | **5.57A** | ⚠️ Peak exceeds 6A if RPi at max |

**3.3V Sensor Rail (from LDO, 0.5A target):**
| Load | Typical | Peak | Notes |
|------|---------|------|-------|
| Qwiic sensors (3.3V mode) | 0.05A | 0.15A | IMU/ToF modules vary; leave 30% margin |
| Level shifter reference | 0.01A | 0.02A | For BSS138/TXB0104 Vref |
| **3.3V Rail Total** | **0.06A** | **0.17A** | OK within 0.5A LDO headroom |

**Servo Rail (adjustable ~4.7–10.4V, default ~6V) from servo buck converter, 6A max:**
| Load | Typical | Peak | Notes |
|------|---------|------|-------|
| Standard servos (×4) | 0.4A | 4.0A | ~100mA idle, 1A stall each |
| PCA9685 module | 0.01A | 0.01A | Logic only |
| **Servo Rail Total** | **0.41A** | **4.01A** | OK within 6A limit |

**Battery Rail (direct from battery, through 15A fuse):**
| Load | Typical | Peak | Notes |
|------|---------|------|-------|
| 5V buck converter | 2.5A | 4.5A | ~85% efficiency from 12V |
| 6V buck converter | 0.3A | 3.0A | ~85% efficiency from 12V |
| DC motors (×4) | 2.0A | 8.0A | ~500mA run, 2A stall each |
| Stepper motors (×4) | 2.0A | 4.0A | ~0.5-1A each at full current |
| **Battery Total** | **6.8A** | **19.5A** | ⚠️ Peak exceeds 15A fuse |

**Power Budget Notes:**
- Default 15A fuse is adequate for typical operation
- Peak currents are worst-case (all motors stalled simultaneously)
- For heavy-duty applications, upgrade to 20A fuse
- Fuse is replaceable - adjust based on actual usage
- Consider staggered motor startup in firmware to reduce inrush

---

## 2. Microcontroller

### 2.1 Arduino Selection: Arduino Mega 2560 (ATmega2560)
- 54 digital I/O pins (15 PWM capable)
- 16 analog inputs
- 4 hardware UART ports
- I2C and SPI support
- 5V logic level


### 2.2 Arduino Connections
**UART:**
- Serial0 (pins 0/1): Arduino programming/USB
- **Serial2 (pins 16/17): Raspberry Pi communication (via level shifter)**
- Serial1 (pins 18/19): NOT AVAILABLE (pins used for Motor 3/4 encoder interrupts)
- Serial3 (pins 14/15): NOT AVAILABLE (pins used for Stepper 1/2 STEP signals)

**I2C:**
- SDA (pin 20), SCL (pin 21): Qwiic connector + expansion header

**SPI:**
- Reserved for future use (SD card, sensors)

---

## 3. Motor Control

### 3.1 DC Motors with Encoders
**H-Bridge Modules:** 2x dual-channel modules
- **Module:** DC5-12V 30A Dual Channel H-Bridge (Amazon B074TH1719)
- **Specs:** 5-12V input, 30A total (15A per channel), current sensing output
- **Total Motors:** 4 DC motors (2 per module)

**Module Pinout (per dual-channel module):**
| Channel A | Channel B | Function |
|-----------|-----------|----------|
| V+ | V+ | Motor power input (battery voltage) |
| GND | GND | Ground |
| IN1_A | IN1_B | Direction control input 1 |
| IN2_A | IN2_B | Direction control input 2 |
| EN_A | EN_B | Enable/PWM input (speed control) |
| CT_A | CT_B | Current sense output (analog) |

**Control Scheme:**
- **Speed:** PWM signal on EN pin (0-255 → 0-100% duty cycle)
- **Direction:** IN1/IN2 logic levels
  - Forward: IN1=HIGH, IN2=LOW
  - Reverse: IN1=LOW, IN2=HIGH
  - Brake: IN1=HIGH, IN2=HIGH
  - Coast: IN1=LOW, IN2=LOW (or EN=LOW)
- **Current Sensing:** CT pins provide analog voltage proportional to motor current

**Connections per motor:**
- 1x PWM (EN pin) → Arduino PWM-capable pin
- 2x Direction (IN1, IN2) → Arduino digital pins
- 1x Current sense (CT) → Arduino analog pin (optional)
- 2x Encoder channels (A/B quadrature) → Arduino interrupt-capable pins
- **Motor Power:** Battery voltage passthrough to H-bridge module V+

**Encoder Power:**
- **Voltage:** 5V from PCB 5V rail
- **Connector:** 4-pin header per encoder (VCC, GND, A, B)
- **Current:** ~20mA per encoder (80mA total for 4 encoders)
- **Decoupling:** 100nF ceramic capacitor near each encoder connector

**Arduino Pin Allocation (DC Motors):**
- Motor 1: EN1 (PWM), IN1_1, IN2_1, CT1 (analog), ENC1A (INT0), ENC1B
- Motor 2: EN2 (PWM), IN1_2, IN2_2, CT2 (analog), ENC2A (INT1), ENC2B
- Motor 3: EN3 (PWM), IN1_3, IN2_3, CT3 (analog), ENC3A (INT5), ENC3B
- Motor 4: EN4 (PWM), IN1_4, IN2_4, CT4 (analog), ENC4A (INT4), ENC4B

### 3.2 Stepper Motors
**Driver Sockets:** 4x sockets for A4988 or DRV8825 modules
- **Pinout per driver:**
  - STEP (Arduino digital pin)
  - DIR (Arduino digital pin)
  - ENABLE (Arduino digital pin, individual per driver)
  - MS1, MS2, MS3 (microstepping via jumpers, active-high with pulldowns)
  - RESET and SLEEP (tied high via 10kΩ pullup)
  - VMOT (motor power from battery)
  - GND
- **Microstepping Jumpers (per driver):**
  | MS1 | MS2 | MS3 | A4988 Step | DRV8825 Step |
  |-----|-----|-----|------------|--------------|
  | 0 | 0 | 0 | Full | Full |
  | 1 | 0 | 0 | 1/2 | 1/2 |
  | 0 | 1 | 0 | 1/4 | 1/4 |
  | 1 | 1 | 0 | 1/8 | 1/8 |
  | 1 | 1 | 1 | 1/16 | 1/32 |
- **Motor Connectors:** 4-pin headers (A+, A-, B+, B-)

**Arduino Pin Allocation (Steppers):**
- Stepper 1: STEP1, DIR1, EN1
- Stepper 2: STEP2, DIR2, EN2
- Stepper 3: STEP3, DIR3, EN3
- Stepper 4: STEP4, DIR4, EN4

### 3.3 Servo Motors
**Configuration:** External PCA9685 I2C PWM module (off-the-shelf)
- **Module:** Adafruit PCA9685 16-channel PWM board or compatible
- **Connection:** I2C bus (SDA/SCL) + servo power rail
- **Power Distribution on PCB:**
  - Servo power rail (adjustable 5-10V from DC-DC converter, default ~6V)
  - GND distribution
  - I2C connection to Arduino (SDA/SCL, 3.3V via Qwiic connector)
- **Connectors on PCB:**
  - **J_PCA9685**: 4-pin header (VCC, GND, SDA, SCL)
    - VCC connects to Qwiic 3.3V rail (for PCA9685 logic power)
    - Can also use Qwiic connector for I2C connection
  - **JP_SERVO_PWR**: Jumper to enable servo rail power to PCA9685 VCC input
    - **Jumper CLOSED:** Servo rail directly powers PCA9685 V+ (no external cable needed)
    - **Jumper OPEN:** PCA9685 V+ must be powered externally

**Servo Power Configuration:**
- **Option 1 (Recommended):** Close JP_SERVO_PWR jumper
  - Servo rail voltage (5-10V) automatically supplied to PCA9685 V+
  - PCA9685 logic powered from 3.3V via I2C connection
  - No external power cable needed
- **Option 2:** Open JP_SERVO_PWR jumper
  - User supplies external servo power to PCA9685 V+ terminal
  - Useful for separate high-current servo power supply

**Note:** Servos plug directly into PCA9685 module, not into the main PCB

---

## 4. Communication Interfaces

### 4.1 Arduino ↔ Raspberry Pi UART
**Connection:** Serial2 (Arduino TX2/RX2, pins 16/17) ↔ RPi UART (GPIO14/15)
- **Level Shifter:** TXB0104 (4-channel bidirectional)
  - Arduino side (pins 16/17): 5V logic
  - RPi side (GPIO14/15): 3.3V logic
  - 2 channels for UART, 2 spare channels for future use
- **Baud Rate:** 115200 or 230400 (configurable in firmware)
- **Flow Control:** Optional (CTS/RTS if needed for high-speed data)

**Pin Mapping:**
- Arduino TX2 (pin 16) → Level Shifter → RPi GPIO15 (RXD0)
- Arduino RX2 (pin 17) ← Level Shifter ← RPi GPIO14 (TXD0)

### 4.2 I2C (Qwiic/STEMMA QT) - Rev. B Simplified Design

**Arduino I2C (3.3V with Level Shifting):**
- **4x Qwiic connectors** (J_QWIIC_1 to J_QWIIC_4): JST-SH 4-pin, 1mm pitch
- All connected to Arduino pins 20/21 (SDA/SCL) in parallel
- **Fixed 3.3V power rail** (from on-board 3.3V LDO)
- **Level shifting:** TXS0102 bidirectional translator (Arduino 5V ↔ Qwiic 3.3V)
  - VCCA = Arduino 5V (SDA/SCL signals from Arduino)
  - VCCB = 3.3V (SDA/SCL signals to Qwiic devices)
  - OE pulled high with RC delay for proper startup
- **Pull-up resistors:** 4.7kΩ on Qwiic side (3.3V) - single set for entire bus
- **Compatible with:** All standard Qwiic/STEMMA QT devices (ICM-20948, BNO085, OLED displays, etc.)

**Qwiic Connector Pinout (for custom cables):**
| Pin | Signal | Wire Color (Standard) | Voltage |
|-----|--------|-----------------------|---------|
| 1 | GND | Black | 0V |
| 2 | VCC | Red | 3.3V |
| 3 | SDA | Blue | 3.3V logic |
| 4 | SCL | Yellow | 3.3V logic |

**Raspberry Pi I2C (3.3V Native):**
- **2x Qwiic connectors** (J_QWIIC_RPI1, J_QWIIC_RPI2): JST-SH 4-pin, 1mm pitch
- All connected to RPi GPIO2 (SDA) / GPIO3 (SCL) in parallel
- **Voltage:** 3.3V (native RPi logic level, no level shifting)
- **Pull-up resistors:** 4.7kΩ to 3.3V (single set for entire bus)
- **Separate bus from Arduino** - no electrical connection to Arduino I2C

**Design Rationale (Rev. B Change):**
- **Simplified from Rev. A:** Removed voltage selection jumpers (Bank A/B)
- **Reasoning:** All modern Qwiic/STEMMA QT devices are 3.3V compatible
- **Benefit:** Fewer configuration errors, simpler student setup
- **Trade-off:** Cannot directly connect 5V-only I2C devices to Qwiic connectors (use separate I2C header if needed)

**Note:** Multiple Qwiic connectors on same bus are for daisy-chaining - students can connect multiple modules without external splitters

---

## 5. User Interface

### 5.1 LED Indicators
**Status/User LEDs (5 total, Arduino GPIO-driven):**
| LED | Color | Function | Arduino Pin |
|-----|-------|----------|-------------|
| LED1 | Green | System OK (default) | Pin 44 (PWM) |
| LED2 | Red | Error/Low Battery (default) | Pin 11 (PWM) |
| LED3 | Blue | User (exposed) | Pin 45 (PWM) |
| LED4 | Orange | User (exposed) | Pin 46 (PWM) |
| LED5 | Purple | User (exposed) | Pin 47 |

**Note:** Red/green are reserved for system status. Blue/yellow/white are exposed to screw terminals/headers for user logic. PWM available on 44/45/46/11; LED5 is on/off only.

**Power Indicator LEDs (always-on when rail active):**
- 5V rail indicator (green) with ~2.2kΩ series resistor (~1–1.5mA)
- Servo rail indicator (green) with ~3.9kΩ series resistor to cover up to 10V (~1–2mA at 10V)
- 5V power-good (ideal diode STAT): LED+2.2kΩ to 5V_SYS, cathode to LTC4412 STAT, ~100kΩ pull-up to 5V_SYS. LED ON only when 5V source is accepted and the ideal diode is conducting (5V power from the buck); OFF when absent/reverse/backfed.

### 5.2 Programmable RGB LED
**Type:** WS2812B (NeoPixel-compatible)
- **On-board:** 1 LED for basic status indication
- **Extension Socket:** 3-pin header (DIN, 5V, GND) for external NeoPixel strip/ring
  - Data output from on-board LED connects to socket DIN
  - Users can chain additional WS2812B modules
- **Data Line:** Arduino pin 42 (5V logic)
- **Power:** 5V rail with 100µF capacitor near socket
- **Current:** ~60mA per LED at full white (on-board only; external strip needs separate power consideration)

### 5.3 User Input Buttons and Limit Switches
**Configuration:** 2 dedicated user buttons + 8 dual-purpose button/limit switch inputs

**Dedicated User Buttons (on-board only):**
- **Quantity:** 2 buttons
- **Type:** Tactile pushbuttons (6mm x 6mm through-hole)
- **Connection:** Arduino GPIO with INPUT_PULLUP (10kΩ internal pullup)
- **Arduino Pins:** 38, 39
- **Function:** User-programmable (start, stop, mode select, etc.)

**Dual-Purpose Button/Limit Switch Inputs:**
- **Quantity:** 8 inputs (buttons 3-10)
- **Type:** On-board tactile button + external 3-pin JST 2.54mm connector in parallel
- **Connection:** Arduino GPIO with configurable INPUT_PULLUP (default) or INPUT mode
- **Arduino Pins:** 40, 41, 48, 49, 50, 51, 52, 53
- **Function:** Homing limit switches, endstops, or user buttons
- **Typical Use Case:** 6 for axis limits (X/Y/Z min/max), 2 spare for tool sensors or extra buttons

**External Limit Switch Connector (per input):**
- **Connector Type:** JST XH 2.54mm 3-pin (B3B-XH-A)
- **Pinout:**
  | Pin | Label | Function |
  |-----|-------|----------|
  | 1 | V | Power (selectable 5V or 3.3V via jumper) |
  | 2 | S | Signal (to Arduino pin, active-low) |
  | 3 | G | Ground |

**Voltage Selection (shared for all 8 limit switch connectors):**
- **Jumper:** 3-pin header (JP_LIM_V) selects VCC rail for all limit switch connectors
- **Options:** 5V (default) or 3.3V for sensor compatibility
- **Silkscreen:** Clear labeling "LIM VCC: 5V | 3.3V"

**Supported Limit Switch Types:**
- ✅ **Mechanical switches** (2-wire): Connect S and G pins only (normally-open)
- ✅ **IR optical endstops** (3-wire): Connect V, S, G (most common: NPN open-collector, active-low)
- ✅ **Inductive proximity sensors** (3-wire): Connect V, S, G (verify voltage and output type)
- ✅ **Hall effect sensors** (3-wire): Connect V, S, G

**Firmware Configuration:**
```cpp
// For mechanical switches and open-collector sensors (most common)
pinMode(LIMIT_PIN, INPUT_PULLUP);  // Active-low, internal ~20-50kΩ pullup

// For sensors with internal pull-down (rare)
pinMode(LIMIT_PIN, INPUT);  // No pullup, sensor drives HIGH/LOW
```

**Circuit Behavior:**
- **On-board button pressed OR external switch/sensor triggered** → Signal pulled LOW → Arduino reads LOW
- **All released/open/not triggered** → Signal pulled HIGH (by INPUT_PULLUP or sensor) → Arduino reads HIGH

**Debouncing:** Firmware-based (mechanical switches require software debouncing)

---

## 6. Expansion and Debugging

### 6.1 GPIO Breakout Headers
**Arduino Unused Pins:**
- Breakout via screw terminals or pin headers (0.1" pitch)
- Include GND and 5V for each section
- Clearly labeled

**Raspberry Pi GPIO:**
- Expose unused GPIOs via pin header
- Include 3.3V and GND
- **Caution:** Add 330Ω series resistors for protection

### 6.2 Programming Headers
**Arduino:**
- ICSP header (2x3 pin, 0.1" pitch) for bootloader programming
- USB connection via Arduino Mega built-in USB

**Raspberry Pi:**
- Access to all 40-pin GPIO header
- microSD card slot accessible for OS flashing

---

## 7. Protection Circuits

### 7.1 Power Protection
- **Reverse Polarity:** P-MOSFET circuit (IRF4905) on battery input
  - More efficient than Schottky diode (~0.2V drop vs ~0.5V at 10A)
  - Gate protected by 15V zener diode
- **Overcurrent:** Replaceable fuse (15A automotive blade type)
- **ESD Protection:** TVS diodes on external-facing connectors (USB, Qwiic, GPIO)

### 7.2 Signal Protection
- **Motor Drivers:** Flyback diodes on inductive loads (usually integrated in modules)
- **Servo Rail:** Bulk capacitors (1000µF+) near connectors
- **Logic Signals:** Series resistors (220-330Ω) on outputs to external headers

### 7.3 Decoupling Capacitors
**Power Rail Decoupling:**
| Rail | Bulk Capacitor | Ceramic Capacitors | Location |
|------|----------------|-------------------|----------|
| Battery input | 100µF/50V electrolytic | 100nF ceramic | Near XT60 connector |
| 5V rail | 470µF/10V electrolytic | 10µF + 100nF ceramic | Near buck converter output |
| Servo rail (4.7–10.4V adjustable) | 1000µF/10V electrolytic | 10µF + 100nF ceramic | Near buck converter output |
| 3.3V rail | 100µF/10V electrolytic | 100nF ceramic | Near LDO output |

**IC Decoupling:**
| IC | Capacitor | Location |
|----|-----------|----------|
| LM61460 (×2) | 22µF ceramic input, 100µF ceramic output | Per datasheet placement |
| TXB0104 (UART) / TXS0102 (Qwiic) | 100nF ceramic on VCCA and VCCB | Within 5mm of IC |
| Arduino Mega | Internal (on Arduino board) | N/A |

**Peripheral Decoupling:**
- Encoder connectors: 100nF ceramic per connector (×4)
- Qwiic connectors: 100nF ceramic per connector (×6)
- WS2812B: 100µF electrolytic + 100nF ceramic near LED
- NeoPixel extension socket: 100µF electrolytic near socket

**Total Decoupling Capacitors:**
- 100nF ceramic: ~20 pcs
- 10µF ceramic: 4 pcs
- 22µF ceramic: 2 pcs (buck converter input)
- 100µF ceramic: 2 pcs (buck converter output)
- 100µF electrolytic: 4 pcs
- 470µF electrolytic: 1 pc
- 1000µF electrolytic: 1 pc

---

## 8. Connectors and Mechanical

### 8.1 Connector Types
**Power:**
- Battery input: XT60 connector
- Main power switch: Panel-mount toggle or rocker (≥15A)

**Motors:**
- DC motors: 2-pin screw terminals (per motor)
- Steppers: 4-pin JST-XH (per motor)
- Encoders: 4-pin headers (VCC, GND, A, B)

**Servos:**
- 3-pin headers (0.1" pitch) x 16 channels (on external PCA9685 module)
- PCA9685 power: JP_SERVO_PWR jumper enables direct servo rail connection

**Communication:**
- Qwiic (Arduino): JST-SH 4-pin, 1mm pitch (x4: J_QWIIC_1 to J_QWIIC_4)
  - All parallel on Arduino I2C bus (pins 20/21)
  - Fixed 3.3V power, level-shifted signals
- Qwiic (RPi): JST-SH 4-pin, 1mm pitch (x2: J_QWIIC_RPI1, J_QWIIC_RPI2)
  - All parallel on RPi I2C bus (GPIO2/GPIO3)
  - Native 3.3V (no level shifting)
- UART: Not exposed externally (internal Arduino-RPi connection via TXB0104)

**Limit Switches:**
- Limit switch connectors: JST XH 2.54mm 3-pin (x8, pins 40, 41, 48-53)
- Pinout: V (power), S (signal), G (ground)
- Voltage select jumper: 3-pin header (JP_LIM_V) for 5V or 3.3V selection

**Configuration Jumpers:**
- JP_LIM_V: Limit switch voltage selection (5V or 3.3V)
- JP_SERVO_PWR: PCA9685 servo rail power enable (ON/OFF)

**Expansion:**
- GPIO: Screw terminals or 0.1" pin headers

### 8.2 Mechanical Features
- **Mounting Holes (Primary):** 4x M3 or M4 holes in corners (compatible with chassis)
- **Mounting Holes (Display Add-on):** 4x M2.5 holes for Raspberry Pi 5 official 5-inch display
  - Pattern matches RPi 5 official display back mounting
  - Allows PCB to be mounted to display housing or vice versa
  - Position to avoid interference with main components
  - Optional: Can be used for general accessory mounting
- **Board Size:** TBD based on component placement (estimate 150mm x 100mm or larger to accommodate display mount points)
- **Standoff Height:** 10-15mm clearance for components
- **Heatsinks:** Provision for heatsinks on DC-DC converters and motor drivers if needed
- **Keep-out Zones:** Mark areas for display cable routing and clearance

---

## 9. Component Summary

### 9.1 On-Board Components (Custom PCB)

| Category | Component | Quantity | Notes |
|----------|-----------|----------|-------|
| **Microcontroller** | Arduino Mega 2560 | 1 | Or compatible (CH340-based) |
| **Power Management** | LM61460AASQRJRRQ1 | 2 | 5V and adjustable servo rail (4.7–10.4V), with heatsinks |
| **Reverse Polarity** | IRF4905 P-MOSFET | 1 | TO-220, with 10kΩ gate resistor + 15V zener |
| **Ideal Diode (5V OR-ing)** | SI2301 P-MOSFET | 2 | SOT-23, for buck/RPi power OR-ing |
| **Level Shifter** | TXB0104 | 1 | 4-channel bidirectional (UART + spare) |
| **RGB LED** | WS2812B | 1 | On-board + 3-pin extension socket |
| **Status/User LEDs** | 3mm LEDs | 5 | Red/Green default; Blue/Orange/Purple exposed (pins 11, 44-47) |
| **Power LEDs** | 3mm LEDs | 2 | 5V rail (green), servo rail (green) |
| **User Buttons** | Tactile switches (6x6mm) | 10 | 2 dedicated (pins 38-39), 8 dual-purpose with limit switches (pins 40-41, 48-53) |
| **Limit Switch Connectors** | JST XH 2.54mm 3-pin | 8 | Pins 40-41, 48-53 (V, S, G) |
| **Limit Switch Voltage Jumper** | 3-pin 0.1" header | 1 | JP_LIM_V for 5V/3.3V selection |
| **Qwiic Connector** | JST-SH 4-pin | 6 | I2C expansion (4x Arduino, 2x RPi) |
| **Fuse Holder** | Automotive blade fuse | 1 | 15A rating, through-hole |
| **Power Switch** | Toggle/rocker switch | 1 | ≥15A rating, panel-mount |
| **Voltage Regulator** | 3.3V LDO (AP2112/TLV75733 class) | 1 | Dedicated Qwiic sensor rail |
| **Battery Connector** | XT60 | 1 | Power input |
| **Microstepping Jumpers** | 2-pin headers + jumper caps | 12 | 3 jumpers × 4 stepper drivers |
| **Screw Terminals** | 2-pin, 3-pin, 4-pin | Multiple | Motors, encoders, power |
| **Pin Headers** | 0.1" pitch | Multiple | Module connections, NeoPixel socket |
| **Resistors/Capacitors** | Various | Many | Pullups, decoupling, voltage dividers |

### 9.2 External Modules (Off-the-Shelf, Not on PCB)

| Module | Quantity | Connection to PCB | Supplier Example |
|--------|----------|-------------------|------------------|
| **H-Bridge Module** | 2 | 12-pin header (V+, GND, IN1, IN2, EN, CT × 2 channels) | DC5-12V 30A Dual H-Bridge (Amazon B074TH1719) |
| **Stepper Driver** | 4 | Socket headers (A4988 footprint) | A4988, DRV8825 (Pololu, Adafruit) |
| **PCA9685 PWM Module** | 1 | 4-pin I2C header | Adafruit PCA9685 breakout |
| **Raspberry Pi 5** | 1 | 40-pin header or custom connector | Raspberry Pi Foundation |
| **DC Motors** | 4 | Via H-bridge modules | User-supplied |
| **Stepper Motors** | 4 | Via driver modules | NEMA 17 or similar |
| **Servo Motors** | Up to 16 | Via PCA9685 module | Standard hobby servos |
| **Quadrature Encoders** | 4 | 4-pin headers on PCB (VCC, GND, A, B) | Mounted on DC motors |

**⚠️ H-Bridge Voltage Warning:** The default H-bridge module is rated for 5-12V only. If using batteries >12V (e.g., 4S-6S LiPo), replace with higher voltage modules (e.g., BTS7960 for 24V). Always match H-bridge voltage rating to battery voltage.

---

## 10. GPIO Pin Assignment Table
### 10.1 Arduino Mega 2560 Pin Allocation [Rev. B]
**Exposed pins are available on screw terminals/headers for reuse; core comms and default wheel drive pins remain internal.**

| Pin(s) | Pin Name | Function | Exposed? | Notes |
|--------|----------|----------|----------|-------|
| 0 (RX0) | RX0 | USB Serial | No | Programming/debug only |
| 1 (TX0) | TX0 | USB Serial | No | Programming/debug only |
| 2 (INT0) | M1_ENC_A | Motor 1 Encoder A | No | Default left/right wheel encoder A |
| 3 (INT1) | M1_ENC_B | Motor 1 Encoder B | No | 4x quadrature mode for M1 (wheel) |
| 4 | M2_IN1 | Motor 2 Direction IN1 | No | Relocated from pin 12 (Rev. A) |
| 5 (PWM) | LED_RED | Status LED Red | No | Error/low battery, relocated from pin 11 (Rev. A) |
| 6 (PWM Timer 4) | M1_EN | Motor 1 PWM Enable | No | Relocated from pin 5 (Rev. A) |
| 7 (PWM Timer 4) | M2_EN | Motor 2 PWM Enable | No | Relocated from pin 6 (Rev. A) |
| 8 (PWM) | M1_IN1 | Motor 1 Direction IN1 | No | Default wheel direction |
| 9 (PWM, timer 2) | M3_EN | Motor 3 PWM Enable | Yes | Manipulator motor |
| 10 (PWM, timer 2) | M4_EN | Motor 4 PWM Enable | Yes | Manipulator motor |
| 11 (PWM) | M4_ENC_A | Motor 4 Encoder A (PCINT5) | Yes | 4x mode via PCINT, relocated from pin 19 (Rev. A) |
| 12 | M4_ENC_B | Motor 4 Encoder B (PCINT6) | Yes | 4x mode via PCINT, relocated from pin 31 (Rev. A) |
| 13 | USER_P13 | User GPIO / General Purpose | Yes | Available for custom use |
| 14 | ST1_STEP | Stepper 1 STEP | Yes | Stepper control |
| 15 | ST2_STEP | Stepper 2 STEP | Yes | Stepper control |
| 16 (TX2) | TX_RPI | **UART to RPi5** | No | Via level shifter (5V → 3.3V) |
| 17 (RX2) | RX_RPI | **UART from RPi5** | No | Via level shifter (3.3V → 5V) |
| 18 (INT5) | M2_ENC_A | Motor 2 Encoder A | No | 4x quadrature mode for M2 (wheel), relocated from pin 3 (Rev. A) |
| 19 (INT4) | M2_ENC_B | Motor 2 Encoder B | No | 4x quadrature mode for M2 (wheel), relocated from pin 7 (Rev. A) |
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
| 30 | M2_IN2 | Motor 2 Direction IN2 | No | Relocated from pin 13 (Rev. A) |
| 31 | USER_P31 | User GPIO / General Purpose | Yes | Available for custom use |
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
| A7-A13 | ANALOG_EXP | Analog Expansion | Yes | Available for sensors |
| A14 | M3_ENC_A | Motor 3 Encoder A (PCINT14) | Yes | 4x mode via PCINT, relocated from pin 18 (Rev. A) |
| A15 | M3_ENC_B | Motor 3 Encoder B (PCINT15) | Yes | 4x mode via PCINT, relocated from pin 30 (Rev. A) |


### 10.2 Arduino Mega 2560 Pin Allocation [Rev. A]
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

### 10.2 Raspberry Pi 5 GPIO Allocation

| GPIO | Function | Notes |
|------|----------|-------|
| GPIO2 (SDA1) | I2C Data | Qwiic connector |
| GPIO3 (SCL1) | I2C Clock | Qwiic connector |
| GPIO14 (TXD0) | UART TX to Arduino | Via level shifter |
| GPIO15 (RXD0) | UART RX from Arduino | Via level shifter |
| GPIO4-GPIO27 | GPIO Breakout | Unused pins to terminals |

---

## 11. Design Verification Checklist (Rev. B - Complete)

### Electrical Design ✅
- [x] All power rails properly decoupled (bulk + ceramic capacitors)
- [x] Battery voltage divider sized for 5V max at highest battery voltage (1:6 ratio)
- [x] Level shifters correctly oriented (5V ↔ 3.3V with TXB0104 and TXS0102)
- [x] Pull-up resistors on I2C lines (4.7kΩ per bank)
- [x] Pull-ups on button inputs (Arduino internal INPUT_PULLUP, 10-50kΩ)
- [x] Flyback diodes verified in external driver modules (H-bridge, stepper drivers)
- [x] Reverse polarity protection on battery input (IRF4905 P-MOSFET circuit)
- [x] Fuse rated for expected current draw (15A blade fuse, upgradeable to 20A)
- [x] ESD protection on external connectors (TVS diodes where needed)
- [x] Heatsink provision for DC-DC converters (thermal vias, 2oz copper, bottom pour)
- [x] Trace widths adequate: ≥40mil battery, ≥30mil DC-DC outputs, ≥10mil logic
- [x] Ground plane continuous with stitching vias
- [x] Test points on critical signals (5V, 3.3V, servo rail, battery, key GPIO)

### Mechanical Design ✅
- [x] Main mounting holes (4x M3/M4) positioned for chassis integration
- [x] Display mounting holes (4x M2.5) for RPi 5 official 5-inch display (optional)
- [x] Standoff clearance 10-15mm verified for component heights
- [x] Keep-out zones marked for display cable routing and clearance
- [x] Board size accommodates all connectors with access clearance
- [x] Silkscreen labels clear and educational (component IDs, voltage warnings)
- [x] Component heights verified for enclosure compatibility

### Firmware Integration ✅
- [x] Pin assignments finalized for Rev. B (all 4 motors support 4x encoders)
- [x] Timer 3 conflict documented and resolved (LED_RED digital-only)
- [x] Encoder ISR handlers planned (INT0-INT5 + PCINT0-PCINT1)
- [x] UART protocol between Arduino and Raspberry Pi defined
- [x] Voltage monitoring ADC channels allocated (A0-A2 for battery/rails)

---

## 12. Future Expansion Considerations

- **Additional UART:** Use software serial or USB-serial adapter for GPS or other sensors (hardware Serial1/Serial3 pins are occupied)
- **SPI Header:** For SD card logging or high-speed sensors
- **CAN Bus:** Add MCP2515 module for automotive-style communication
- **IMU:** Reserve I2C address for MPU6050/BNO055
- **Display Options:**
  - **Raspberry Pi 5 Official 5-inch Display (Recommended):** M2.5 mounting holes on PCB for direct attachment
  - I2C OLED or SPI TFT connector for Arduino-driven displays
- **Wireless:** ESP32 socket for WiFi/Bluetooth (optional upgrade path)
- **Camera:** Ribbon cable access for RPi camera module (15-pin or 22-pin connector)

---

## 13. PCB Design Implementation (Rev. B - As Built)

### Design Tool & Files
- **Schematic/Layout Tool:** EasyEDA (online)
- **Export Formats:** PDF schematics and Gerber files for fabrication
- **File Location:** See `schematic/` and `layout/` folders for design files

### PCB Specifications (Final)
- **Board Type:** 2-layer FR4 (cost-effective, adequate for design requirements)
- **Board Dimensions:** ~150mm × 100mm (accommodates all components and mounting holes)
- **Copper Weight:** 2oz (70µm) on power traces for thermal management
- **Surface Finish:** HASL or ENIG (per fabricator standard)

### Trace Width Implementation
- **Battery input:** ≥40mil (1.0mm) - handles up to 20A burst current
- **5V rail (buck output):** ≥30mil (0.8mm) - rated for 6A continuous
- **Servo rail (buck output):** ≥30mil (0.8mm) - rated for 6A continuous
- **Motor power to H-bridge:** ≥30mil - direct battery passthrough
- **Logic signals:** ≥10mil (0.254mm) - standard digital I/O
- **I2C, UART:** ≥10mil with controlled impedance where practical

### Via Specifications
- **Standard vias:** 0.3mm drill, 0.6mm pad
- **Thermal vias (buck converters):** 0.3mm drill, array of 9+ vias per IC
- **Via stitching:** Ground plane connected with vias every ~10mm

### Mounting Hole Locations
- **Main chassis:** 4x M3/M4 holes (3.2mm or 4.2mm diameter) in corners
- **Display mounting (optional):** 4x M2.5 holes (2.7mm diameter) for RPi 5 official 5" display
- **Standoff height:** 10-15mm recommended for component clearance

### Thermal Management (LM61460 Buck Converters)
- **Top side:** Large copper pour connected to IC thermal pad
- **Thermal vias:** 3×3 array (minimum) under each IC, connecting to bottom ground plane
- **Bottom side:** Solid copper pour (heatsink), minimum 1 square inch per IC
- **Optional:** Stick-on heatsinks marked on silkscreen for high-load applications

### Silkscreen & Component Marking
- **Font size:** ≥40mil (1.0mm) for readability
- **Component IDs:** All ICs, connectors, jumpers clearly labeled
- **Voltage warnings:** Marked on servo rail potentiometer, Qwiic banks, limit switch jumper
- **Polarity:** Battery connector, LED orientations, motor connectors
- **Educational labels:** "Arduino I2C", "RPi I2C", "5V", "Servo Rail", etc.

### Design Principles Applied
- **Modularity:** PCB provides power distribution and clean interfaces; motor control via external modules
- **Educational clarity:** Logical component grouping, accessible test points, clear labeling
- **Serviceability:** Standard connectors (JST, Qwiic, screw terminals), replaceable fuse, socketed stepper drivers
