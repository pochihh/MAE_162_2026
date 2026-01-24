# Custom PCB Design

**PRIORITY**: PCB manufacturing has the longest lead time (2-4 weeks).

## Overview

Main controller board integrating Arduino Mega 2560, dual DC-DC converters, and Raspberry Pi 5 interface for the MAE 162 educational robotics platform.

**Design Philosophy:** Modular design using off-the-shelf motor driver and PWM modules. The custom PCB focuses on robust power management, microcontroller integration, and clean interfaces to external modules.

**Design Tool:** EasyEDA (online)

## Quick Specifications

### Power System
- **Input:** 7.4V-12V LiPo battery (2S/3S)
- **DC-DC Converters:** 2x LM61460AASQRJRRQ1
  - 5V @ 6A (RPi5 + logic systems)
  - 5-10V adjustable @ 6A (servo power)
- **Battery Monitoring:** Arduino ADC with voltage divider
- **Protection:** Replaceable fuse, power switch, reverse polarity protection

### Motor Control (External Modules)
- **DC Motors:** 4x motors via 2x external dual H-bridge modules (L298N, BTS7960, IBT-2)
  - Encoder support: 4-pin headers on PCB for quadrature encoders
- **Stepper Motors:** 4x sockets for A4988/DRV8825 external driver modules
- **Servo Motors:** Via external PCA9685 I2C PWM module (Adafruit or compatible)
  - PCB provides servo power rail (adjustable 5-10V) and I2C connection

### Communication
- **Arduino ↔ RPi5:** UART with bi-directional level shifter (5V ↔ 3.3V)
- **I2C Expansion:** 6x Qwiic connectors total
  - 4x on Arduino I2C bus (parallel)
    - Bank1: 2x Qwiic with optional operating voltage 5V/3V3
    - Bank2: 2x Qwiic with optional operating voltage 5V/3V3 
  - 2x on RPi I2C bus (parallel)
    - Bank3: 2x Qwiic with operating voltage 3V3
- **GPIO Breakout:** Unused pins on screw terminals/headers

### User Interface
- **LEDs:** Power indicators, status LEDs, low battery warning
- **RGB Lights:** 4-8x WS2812B addressable LEDs
- **Buttons:** 4-6x tactile buttons on Arduino GPIOs

## File Structure

```
pcb/
├── SPECIFICATIONS.md       Detailed design requirements and pin assignments
├── schematic/              Schematic design files (EasyEDA)
├── layout/                 PCB layout files
└── datasheets/             Component datasheets
```

## Key Design Documents

- **[SPECIFICATIONS.md](SPECIFICATIONS.md)** - Complete design requirements, component list, GPIO pin assignments, and verification checklist

## Design Checklist

- [ ] Review detailed specifications in SPECIFICATIONS.md
- [ ] Finalize component selection (verify availability and cost)
- [ ] Complete schematic in EasyEDA
- [ ] Assign footprints and create custom parts if needed
- [ ] PCB layout with proper trace widths and ground planes
- [ ] Design rule check (DRC) and electrical rule check (ERC)
- [ ] Export manufacturing files (Gerbers, drill files, BOM)
- [ ] Order prototype PCBs and components
- [ ] Assembly and testing

## Design Guidelines

- **Board Layers:** 2-layer (4-layer if signal integrity critical)
- **Trace Width:** ≥20mil for motor power, ≥10mil for logic
- **Clearance:** 10mil minimum
- **Via Size:** 0.3mm drill, 0.6mm pad
- **Mounting:** 4x M3/M4 holes for chassis integration
- **Silkscreen:** Educational labels for student assembly

## Component Availability

### On-Board Components (verify before ordering PCB):
- LM61460AASQRJRRQ1 (automotive-grade buck converter) - **Critical path item**
- Arduino Mega 2560 compatible microcontroller
- TXS0108E or TXB0104 bi-directional level shifter
- WS2812B/SK6812 RGB LEDs
- Standard passives (resistors, capacitors)

### External Modules (off-the-shelf):
- 2x Dual H-bridge modules (L298N, BTS7960, or IBT-2)
- 4x Stepper drivers (A4988 or DRV8825 from Pololu, Adafruit)
- 1x PCA9685 16-channel PWM board (Adafruit or compatible)
- 1x Raspberry Pi 5

## Design Tool

**EasyEDA:** Online EDA tool (https://easyeda.com/)
- Export schematic/PCB as PDF for documentation
- Export Gerbers via "Generate Fabrication File"
- Keep component library synchronized
