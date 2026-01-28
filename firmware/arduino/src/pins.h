/**
 * @file pins.h
 * @brief Hardware pin definitions for Arduino Mega 2560
 *
 * This file contains all GPIO pin assignments matching the hardware design.
 * Pin definitions are separated from config.h for clarity and maintainability.
 *
 * NOTE: This file mirrors the GPIO table in README.md and pin_table.md.
 *       Any hardware changes should be reflected here.
 */

#ifndef PINS_H
#define PINS_H

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

// Serial0 (USB) - pins 0/1 used by Arduino core
#define PIN_USB_RX              0       // USB Serial RX (programming/debug)
#define PIN_USB_TX              1       // USB Serial TX (programming/debug)

// Serial2 (Raspberry Pi UART) - pins 16/17
#define PIN_RPI_TX              16      // TX to RPi5 (via level shifter 5V→3.3V)
#define PIN_RPI_RX              17      // RX from RPi5 (via level shifter 3.3V→5V)

// ============================================================================
// I2C BUS
// ============================================================================

#define PIN_I2C_SDA             20      // I2C Data (Qwiic + PCA9685)
#define PIN_I2C_SCL             21      // I2C Clock (Qwiic + PCA9685)

// ============================================================================
// DC MOTOR 1 (Default Left/Right Wheel)
// ============================================================================

#define PIN_M1_EN               5       // PWM Enable (speed control)
#define PIN_M1_IN1              8       // Direction control 1
#define PIN_M1_IN2              43      // Direction control 2
#define PIN_M1_ENC_A            2       // Encoder Phase A (INT0)
#define PIN_M1_ENC_B            4       // Encoder Phase B
#define PIN_M1_CT               A3      // Current sense (analog)

// ============================================================================
// DC MOTOR 2 (Default Left/Right Wheel)
// ============================================================================

#define PIN_M2_EN               6       // PWM Enable (speed control)
#define PIN_M2_IN1              12      // Direction control 1
#define PIN_M2_IN2              13      // Direction control 2
#define PIN_M2_ENC_A            3       // Encoder Phase A (INT1)
#define PIN_M2_ENC_B            7       // Encoder Phase B
#define PIN_M2_CT               A4      // Current sense (analog)

// ============================================================================
// DC MOTOR 3 (Manipulator - Optional)
// ============================================================================

#define PIN_M3_EN               9       // PWM Enable (speed control)
#define PIN_M3_IN1              34      // Direction control 1
#define PIN_M3_IN2              35      // Direction control 2
#define PIN_M3_ENC_A            18      // Encoder Phase A (INT5)
#define PIN_M3_ENC_B            30      // Encoder Phase B
#define PIN_M3_CT               A5      // Current sense (analog)

// ============================================================================
// DC MOTOR 4 (Manipulator - Optional)
// ============================================================================

#define PIN_M4_EN               10      // PWM Enable (speed control)
#define PIN_M4_IN1              36      // Direction control 1
#define PIN_M4_IN2              37      // Direction control 2
#define PIN_M4_ENC_A            19      // Encoder Phase A (INT4)
#define PIN_M4_ENC_B            31      // Encoder Phase B
#define PIN_M4_CT               A6      // Current sense (analog)

// ============================================================================
// STEPPER MOTOR 1
// ============================================================================

#define PIN_ST1_STEP            14      // Step pulse
#define PIN_ST1_DIR             22      // Direction
#define PIN_ST1_EN              26      // Enable (active low)

// ============================================================================
// STEPPER MOTOR 2
// ============================================================================

#define PIN_ST2_STEP            15      // Step pulse
#define PIN_ST2_DIR             23      // Direction
#define PIN_ST2_EN              27      // Enable (active low)

// ============================================================================
// STEPPER MOTOR 3
// ============================================================================

#define PIN_ST3_STEP            32      // Step pulse
#define PIN_ST3_DIR             24      // Direction
#define PIN_ST3_EN              28      // Enable (active low)

// ============================================================================
// STEPPER MOTOR 4
// ============================================================================

#define PIN_ST4_STEP            33      // Step pulse
#define PIN_ST4_DIR             25      // Direction
#define PIN_ST4_EN              29      // Enable (active low)

// ============================================================================
// USER BUTTONS (Active Low - INPUT_PULLUP)
// ============================================================================

#define PIN_BTN1                38      // On-board user button 1
#define PIN_BTN2                39      // On-board user button 2
#define PIN_BTN3                40      // Button 3 (shared with LIM1)
#define PIN_BTN4                41      // Button 4 (shared with LIM2)
#define PIN_BTN5                48      // Button 5 (shared with LIM3)
#define PIN_BTN6                49      // Button 6 (shared with LIM4)
#define PIN_BTN7                50      // Button 7 (shared with LIM5)
#define PIN_BTN8                51      // Button 8 (shared with LIM6)
#define PIN_BTN9                52      // Button 9 (shared with LIM7)
#define PIN_BTN10               53      // Button 10 (shared with LIM8)

// ============================================================================
// LIMIT SWITCHES (Shared with Buttons 3-10)
// ============================================================================

#define PIN_LIM1                40      // Limit switch 1 (shared with BTN3)
#define PIN_LIM2                41      // Limit switch 2 (shared with BTN4)
#define PIN_LIM3                48      // Limit switch 3 (shared with BTN5)
#define PIN_LIM4                49      // Limit switch 4 (shared with BTN6)
#define PIN_LIM5                50      // Limit switch 5 (shared with BTN7)
#define PIN_LIM6                51      // Limit switch 6 (shared with BTN8)
#define PIN_LIM7                52      // Limit switch 7 (shared with BTN9)
#define PIN_LIM8                53      // Limit switch 8 (shared with BTN10)

// ============================================================================
// STATUS AND USER LEDs
// ============================================================================

#define PIN_LED_RED             11      // Status LED Red (error/low battery)
#define PIN_LED_GREEN           44      // Status LED Green (system OK)
#define PIN_LED_BLUE            45      // User LED Blue (exposed)
#define PIN_LED_ORANGE          46      // User LED Orange (exposed)
#define PIN_LED_PURPLE          47      // User LED Purple (exposed, non-PWM)

// ============================================================================
// NEOPIXEL (WS2812B RGB LED)
// ============================================================================

#define PIN_NEOPIXEL            42      // NeoPixel data line

// ============================================================================
// SERVO CONTROLLER (PCA9685)
// ============================================================================

// I2C pins are shared (PIN_I2C_SDA, PIN_I2C_SCL)
// PCA9685 Output Enable pin (active LOW) - optional for power control
// #define PIN_SERVO_OE            xx    // Define if OE pin is connected

// ============================================================================
// STEPPER LIMIT SWITCHES (for homing)
// ============================================================================

// Stepper limit switch assignments (maps to limit switch pins)
#define PIN_ST1_LIMIT           PIN_LIM1  // Stepper 1 limit (40)
#define PIN_ST2_LIMIT           PIN_LIM2  // Stepper 2 limit (41)
#define PIN_ST3_LIMIT           PIN_LIM3  // Stepper 3 limit (48)
#define PIN_ST4_LIMIT           PIN_LIM4  // Stepper 4 limit (49)

// ============================================================================
// ANALOG VOLTAGE SENSING
// ============================================================================

#define PIN_VBAT_SENSE          A0      // Battery voltage (1:6 divider)
#define PIN_V5_SENSE            A1      // 5V rail voltage (1:2 divider)
#define PIN_VSERVO_SENSE        A2      // Servo rail voltage (1:3 divider)

// ============================================================================
// ANALOG EXPANSION (Available for Sensors)
// ============================================================================

#define PIN_ANALOG_EXP_1        A7      // Analog expansion 1
#define PIN_ANALOG_EXP_2        A8      // Analog expansion 2
#define PIN_ANALOG_EXP_3        A9      // Analog expansion 3
#define PIN_ANALOG_EXP_4        A10     // Analog expansion 4
#define PIN_ANALOG_EXP_5        A11     // Analog expansion 5
#define PIN_ANALOG_EXP_6        A12     // Analog expansion 6
#define PIN_ANALOG_EXP_7        A13     // Analog expansion 7
#define PIN_ANALOG_EXP_8        A14     // Analog expansion 8
#define PIN_ANALOG_EXP_9        A15     // Analog expansion 9

// ============================================================================
// CONVENIENCE ARRAYS FOR ITERATION
// ============================================================================

// DC Motor EN pins (PWM)
const uint8_t DC_MOTOR_EN_PINS[4] = {
  PIN_M1_EN, PIN_M2_EN, PIN_M3_EN, PIN_M4_EN
};

// DC Motor IN1 pins
const uint8_t DC_MOTOR_IN1_PINS[4] = {
  PIN_M1_IN1, PIN_M2_IN1, PIN_M3_IN1, PIN_M4_IN1
};

// DC Motor IN2 pins
const uint8_t DC_MOTOR_IN2_PINS[4] = {
  PIN_M1_IN2, PIN_M2_IN2, PIN_M3_IN2, PIN_M4_IN2
};

// DC Motor Encoder A pins (interrupt-capable)
const uint8_t DC_MOTOR_ENC_A_PINS[4] = {
  PIN_M1_ENC_A, PIN_M2_ENC_A, PIN_M3_ENC_A, PIN_M4_ENC_A
};

// DC Motor Encoder B pins
const uint8_t DC_MOTOR_ENC_B_PINS[4] = {
  PIN_M1_ENC_B, PIN_M2_ENC_B, PIN_M3_ENC_B, PIN_M4_ENC_B
};

// DC Motor Current Sense pins
const uint8_t DC_MOTOR_CT_PINS[4] = {
  PIN_M1_CT, PIN_M2_CT, PIN_M3_CT, PIN_M4_CT
};

// Stepper STEP pins
const uint8_t STEPPER_STEP_PINS[4] = {
  PIN_ST1_STEP, PIN_ST2_STEP, PIN_ST3_STEP, PIN_ST4_STEP
};

// Stepper DIR pins
const uint8_t STEPPER_DIR_PINS[4] = {
  PIN_ST1_DIR, PIN_ST2_DIR, PIN_ST3_DIR, PIN_ST4_DIR
};

// Stepper ENABLE pins
const uint8_t STEPPER_EN_PINS[4] = {
  PIN_ST1_EN, PIN_ST2_EN, PIN_ST3_EN, PIN_ST4_EN
};

// User button pins
const uint8_t BUTTON_PINS[10] = {
  PIN_BTN1, PIN_BTN2, PIN_BTN3, PIN_BTN4, PIN_BTN5,
  PIN_BTN6, PIN_BTN7, PIN_BTN8, PIN_BTN9, PIN_BTN10
};

// Limit switch pins (same as buttons 3-10)
const uint8_t LIMIT_PINS[8] = {
  PIN_LIM1, PIN_LIM2, PIN_LIM3, PIN_LIM4,
  PIN_LIM5, PIN_LIM6, PIN_LIM7, PIN_LIM8
};

// User LED pins
const uint8_t USER_LED_PINS[3] = {
  PIN_LED_BLUE, PIN_LED_ORANGE, PIN_LED_PURPLE
};

#endif // PINS_H
