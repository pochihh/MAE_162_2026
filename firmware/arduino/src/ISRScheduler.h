/**
 * @file ISRScheduler.h
 * @brief Hard real-time timer configuration for ISR-driven tasks
 *
 * This module configures the three hardware timers that drive the hard
 * real-time ISRs. The ISR bodies themselves live in the files that own their
 * data:
 *
 *   TIMER3_OVF_vect  (10 kHz) — arduino.ino → StepperManager::timerISR()
 *     Generates stepper pulses for all stepper channels.
 *
 * Timer1 runs a short round-robin DC slot ISR in the mixed-control profile.
 * Timer4 still has no overflow ISR and remains in Fast PWM mode so OC outputs
 * work, while the heavier DC compute, sensor I2C/ADC dispatch, and UART
 * handling all run from the soft scheduler.
 *
 * Timer hardware configuration:
 *
 *   Timer1: Fast PWM mode 14 (ICR1 as TOP), prescaler=8
 *           ICR1 = (F_CPU/8 / DC_PID_FREQ_HZ) - 1 = 9999  → 200 Hz OVF
 *           OC1A (pin 11) drives LED_RED if PIN_LED_RED_IS_OC1A is defined (Rev A).
 *
 *   Timer3: Fast PWM mode 14, prescaler=8 (configured by StepperManager)
 *           ICR3 = (F_CPU/8 / STEPPER_TIMER_FREQ_HZ) - 1 = 199  → 10 kHz OVF
 *           OC3A (pin 5) drives M1_EN if PIN_M1_EN_IS_OC3A is defined (Rev A),
 *                    or LED_RED if PIN_LED_RED_IS_OC3A is defined (Rev B).
 *
 *   Timer4: Fast PWM mode 14, prescaler=8
 *           ICR4 = (F_CPU/8 / STEPPER_TIMER_FREQ_HZ) - 1 = 199  → 10 kHz OVF
 *           OC4A (pin 6) always connected — drives M2_EN (Rev A) or M1_EN (Rev B).
 *           OC4B (pin 7) drives M2_EN if PIN_M2_EN_IS_OC4B is defined (Rev B only).
 *
 * Why Fast PWM mode 14 instead of CTC?
 *   In CTC mode the OCRnA register holds the TOP value, so it CANNOT independently
 *   drive hardware PWM on OC output pins at the same time. Fast PWM mode 14 puts
 *   the TOP value in ICR1/3/4, freeing OCRnA/B/C to generate PWM independently
 *   while the OVF interrupt still fires at the ICR-derived rate.
 *
 * Why direct OCRnx writes instead of analogWrite()?
 *   Arduino's analogWrite() blindly reconfigures the timer to 8-bit Fast PWM,
 *   overwriting the ICR-based Fast PWM setup and breaking the ISR timing.
 *   The macros PIN_M1_EN_OCR, PIN_M2_EN_OCR, LED_RED_OCR (defined in pins.h)
 *   let drivers write directly to the correct register with proper scaling.
 *
 * IMPORTANT: Call ISRScheduler::init() as the LAST step in setup() — after
 * all motor, sensor, scheduler, and stepper objects are fully initialised —
 * because it enables interrupts that immediately invoke those objects.
 */

#ifndef ISR_SCHEDULER_H
#define ISR_SCHEDULER_H

#include <Arduino.h>

class ISRScheduler {
public:
    static void attachDcEncoderInterrupts(void (*m1a)(void),
                                          void (*m1b)(void),
                                          void (*m2a)(void),
                                          void (*m2b)(void));
    static void attachDcEncoderPcints();
    static void configureTimer1DcSlotISR();
    static void configureTimer2PwmOnly();
    static void configureTimer4PwmOnly();
    /**
     * @brief Configure Timer1, Timer2, and Timer4 for motor PWM.
     *
     * Timer2 (8-bit) drives M3_EN (pin 9) and M4_EN (pin 10) at ~7.8 kHz.
     * Timer4 (16-bit) drives M1_EN (pin 6) and M2_EN (pin 7) at 10 kHz.
     * Timer3 is configured separately by StepperManager::init() and uses its
     * overflow vector from arduino.ino.
     *
     * Must be called LAST in setup().
     */
    static void init();
};

#endif // ISR_SCHEDULER_H
