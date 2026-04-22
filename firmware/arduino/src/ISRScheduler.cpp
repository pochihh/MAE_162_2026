/**
 * @file ISRScheduler.cpp
 * @brief Implementation of hard real-time ISR timer configuration
 *
 * See ISRScheduler.h for full design notes and architecture overview.
 */

#include "ISRScheduler.h"
#include "pins.h"
#include "config.h"

void ISRScheduler::attachDcEncoderInterrupts(void (*m1a)(void),
                                             void (*m1b)(void),
                                             void (*m2a)(void),
                                             void (*m2b)(void)) {
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), m1a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_B), m1b, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), m2a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_B), m2b, CHANGE);
}

void ISRScheduler::attachDcEncoderPcints() {
    // M3: PCINT2 group (Port K) — A14=PCINT22, A15=PCINT23
    PCIFR |= _BV(PCIF2);
    PCMSK2 |= _BV(PCINT22);
#if ENCODER_3_MODE == ENCODER_4X
    PCMSK2 |= _BV(PCINT23);
#endif
    PCICR |= _BV(PCIE2);

    // M4: PCINT0 group (Port B) — pin11=PCINT5, pin12=PCINT6
    PCIFR |= _BV(PCIF0);
    PCMSK0 |= _BV(PCINT5);
#if ENCODER_4_MODE == ENCODER_4X
    PCMSK0 |= _BV(PCINT6);
#endif
    PCICR |= _BV(PCIE0);
}

void ISRScheduler::configureTimer1DcSlotISR() {

    // ========================================================================
    // Timer1: Fast PWM mode 14 — 800 Hz OVF → round-robin DC slot
    // ========================================================================
    //
    //   f_cpu     = 16 MHz
    //   prescaler = 8   →   f_timer = 2 MHz
    //   ICR1      = (2 000 000 / DC_PID_FREQ_HZ) - 1 = 2499  →  800 Hz OVF
    //
    //   Fast PWM mode 14: WGM13:10 = 1110
    //     TCCR1A: WGM11=1, WGM10=0
    //     TCCR1B: WGM13=1, WGM12=1, CS11=1 (prescaler 8)
    //
    //   OC1A output connected (Rev A, pin 11) for LED_RED hardware PWM.
    //   Writing OCR1A controls LED brightness; the OVF ISR fires independently.
    // ========================================================================

    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1   = (uint16_t)((F_CPU / (8UL * DC_PID_FREQ_HZ)) - 1);   // 2499 @ 800 Hz
    TCNT1  = 0;

#if defined(PIN_LED_RED_IS_OC1A)
    // Rev A: OC1A (pin 11) → LED_RED, non-inverting fast PWM
    TCCR1A |= (1 << COM1A1);
    OCR1A   = 0;   // LED off at startup
#endif

    TIMSK1 = (1 << TOIE1);   // Timer1 round-robin ISR + PWM hardware
}

void ISRScheduler::configureTimer2PwmOnly() {
    // ========================================================================
    // Timer2: Fast PWM mode 3 — 8-bit fast PWM for M3/M4 motor PWM
    // ========================================================================
    //
    //   f_cpu     = 16 MHz
    //   prescaler = 8   →   f_timer = 2 MHz
    //   8-bit TOP = 255  →  f_pwm = 2 MHz / 256 ≈ 7.8 kHz
    //
    //   OC2A (pin 10) → M4_EN, OC2B (pin 9) → M3_EN
    //
    //   Fast PWM mode 3: WGM22=0, WGM21=1, WGM20=1
    //   Non-inverting: COM2A1=1, COM2A0=0, COM2B1=1, COM2B0=0
    //   Clock select: CS22=0, CS21=1, CS20=0 (prescaler 8)
    // ========================================================================

    // Stop timer and disable interrupts during configuration
    TCCR2B = 0;
    TCNT2 = 0;
    OCR2A = 0;
    OCR2B = 0;
    TIMSK2 = 0;

    // Set Fast PWM mode 3 (WGM = 3) with non-inverting OC outputs
    //   TCCR2A bits: [COM2A1 COM2A0 COM2B1 COM2B0 - - WGM21 WGM20]
    //   Set: COM2A1=1, COM2B1=1, WGM21=1, WGM20=1 (all others = 0)
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);

    // Start timer with prescaler 8
    //   TCCR2B bits: [FOC2A FOC2B - - WGM22 CS22 CS21 CS20]
    //   Set: CS21=1 for prescaler 8 (all others = 0)
    TCCR2B = (1 << CS21);
}

void ISRScheduler::configureTimer4PwmOnly() {
    // ========================================================================
    // Timer4: Fast PWM mode 14 — 10 kHz carrier for motor PWM only
    // ========================================================================
    //
    //   ICR4 = (2 000 000 / STEPPER_TIMER_FREQ_HZ) - 1 = 199  →  10 kHz OVF
    //
    //   OC4A (pin 6) always connected: M2_EN (Rev A) or M1_EN (Rev B).
    //   OC4B (pin 7) connected only when PIN_M2_EN_IS_OC4B is defined (Rev B).
    //
    //   Fast PWM mode 14: WGM43:40 = 1110
    //     TCCR4A: WGM41=1
    //     TCCR4B: WGM43=1, WGM42=1, CS41=1 (prescaler 8)
    // ========================================================================

    TCCR4A = (1 << WGM41);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41);
    ICR4   = (uint16_t)((F_CPU / (8UL * STEPPER_TIMER_FREQ_HZ)) - 1);   // 199
    TCNT4  = 0;

    // OC4A: M2_EN (Rev A, pin 6) or M1_EN (Rev B, pin 6) — always connected
    TCCR4A |= (1 << COM4A1);
    OCR4A   = 0;   // Motor off at startup

#if defined(PIN_M2_EN_IS_OC4B)
    // Rev B only: OC4B (pin 7) → M2_EN, non-inverting fast PWM
    TCCR4A |= (1 << COM4B1);
    OCR4B   = 0;   // Motor off at startup
#endif

    TIMSK4 = 0;   // No Timer4 ISR in bring-up profile; keep PWM hardware only
}

void ISRScheduler::init() {
    noInterrupts();
    configureTimer1DcSlotISR();
    configureTimer2PwmOnly();
    configureTimer4PwmOnly();

    interrupts();
}
