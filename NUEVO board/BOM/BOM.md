# Bill of Materials (BOM) - External Components

**Project:** MAE 162 Educational Robotics Platform - NUEVO Board
**Last Updated:** 2026-01-01

**Note:** This BOM covers components NOT included in the PCB assembly (see `BOM_Carrier_1_PCB1_1_2025-12-22.xlsx` for on-board components). Quantity listed is desinated for one set.

---

## 1. Motor Control, Motors & Power

| Item | Description | Quantity | Supplier | Part Number | Unit Price | Total Price | Notes |
|------|-------------|----------|----------|-------------|------------|-------------|-------|
| H-Bridge Module | DC5-12V 30A Dual Channel H-Bridge | 2 | Amazon | B074TH1719 | $XX.XX | $XX.XX | Each controls 2 DC motors (4 total), 15A/ch, current sensing |
| Stepper Driver | A4988 Stepper Motor Driver | 4 | Pololu/Amazon | | $XX.XX | $XX.XX | 1/16 step, DRV8825 compatible (1/32 step) |
| PWM Controller | PCA9685 16-Channel 12-bit PWM Board | 1 | Adafruit | 815 | $XX.XX | $XX.XX | I2C servo controller, 5-10V power from PCB |
| DC Motor | DC Gearmotor with Encoder | 4 | TBD | | $XX.XX | $XX.XX | 6-12V, quadrature encoder (VCC/GND/A/B) |
| Stepper Motor | NEMA 17 Bipolar Stepper | 4 | TBD | | $XX.XX | $XX.XX | Optional, bipolar 4-wire, ≤2A/phase |
| Servo Motor | Standard Hobby Servo | Up to 16 | TBD | | $XX.XX | $XX.XX | Optional, 4.8-7.4V (match PCB servo rail) |
| Battery | 12V NiMH Battery Pack | 1 | TBD | | $XX.XX | $XX.XX | Default, or 2S/3S LiPo (7.4-11.1V) |
| Fuse | Automotive Blade Micro Fuse 15A | 5 | TBD | | $XX.XX | $XX.XX | Replaceable, 5-pack |

---

## 2. Main Controller Boards

| Item | Description | Quantity | Supplier | Part Number | Unit Price | Total Price | Notes |
|------|-------------|----------|----------|-------------|------------|-------------|-------|
| Microcontroller | Arduino Mega 2560 (or compatible) | 1 | Arduino/Amazon | | $XX.XX | $XX.XX | ATmega2560, CH340 USB acceptable |
| SBC | Raspberry Pi 5 (4GB or 8GB) | 1 | Raspberry Pi | | $XX.XX | $XX.XX | 4GB minimum recommended |

---

## 3. Mechanical Hardware

| Item | Description | Quantity | Supplier | Part Number | Unit Price | Total Price | Notes |
|------|-------------|----------|----------|-------------|------------|-------------|-------|
| Standoff | M2.5 × 10mm Brass Standoff (F-F) | 20 | TBD | | $XX.XX | $XX.XX | Main PCB to chassis |
| Screw | M2.5 × 6mm Pan Head Screw (Phillip)| 40 | TBD | | $XX.XX | $XX.XX | PCB mounting |
| Nut | M3 Hex Nut | 4 | TBD | | $XX.XX | $XX.XX | Optional locking |


---

## Notes

- **H-Bridge Voltage:** Default module rated 5-12V. For >12V batteries (4S-6S LiPo), use BTS7960 modules.
- **Qwiic Sensors:** Educational add-ons (IMU, OLED, distance sensors) available from SparkFun/Adafruit.
- **Cables:** Qwiic cables, motor cables, and JST connectors as needed.
- **Tools:** Soldering iron, wire stripper, crimping tool, hex keys required for assembly.

---