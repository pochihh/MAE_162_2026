/**
 * @file arduino.ino
 * @brief Main firmware for Arduino Mega 2560 educational robotics platform
 * @version 0.9.8
 *
 * Educational robotics platform firmware for MAE 162 course.
 * Provides real-time motor control, sensor integration, and
 * communication with Raspberry Pi 5 via TLV protocol over UART.
 *
 * Two-tier scheduling architecture:
 *
 *   Hard real-time (ISR-driven — unaffected by loop() blocking):
 *     TIMER3_OVF_vect  10 kHz  Stepper pulse generation (StepperManager)
 *
 *   Soft real-time (millis-based, runs in loop() with interrupts enabled):
 *     taskUART         100 Hz  UART RX/TX — loop() keeps USART2_RX_vect alive;
 *                              RX/TX stay out of the ISR path entirely
 *     taskMotors       200 Hz  DC control compute + apply + feedback refresh
 *     taskSafety      100 Hz  heartbeat/battery safety checks
 *     taskSensors     100 Hz  IMU, ultrasonic, voltage + input sampling
 *     taskUserIO        20 Hz  LED animations, NeoPixel status
 *
 * Initialization Order (setup):
 *  1. Debug serial (Serial0)
 *  2. Scheduler (millis-based soft scheduler)
 *  3. PersistentStorage (EEPROM-backed settings/calibration)
 *  4. MessageCenter (Serial2 + TLV codec)
 *  5. SensorManager (I2C, ADC)
 *  6. UserIO (GPIO, NeoPixel)
 *  7. ServoController (PCA9685 via I2C)
 *  8. StepperManager (Timer3 — also starts stepper ISR)
 *  9. DC Motors (PWM pins, encoder counters)
 * 10. Attach encoder ISRs (via ISRScheduler helpers)
 * 11. Register periodic and fast-lane scheduler tasks
 * 12. Configure Timer1/Timer4 runtime timer hardware — LAST
 *
 * Main Loop:
 * - Scheduler::serviceFastLane() handles work-available tasks
 * - Scheduler::tickPeriodic() executes one highest-priority ready periodic task
 */

#include <util/atomic.h>
#include <string.h>

// ============================================================================
// INCLUDES
// ============================================================================

// Core configuration
#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/ISRScheduler.h"
#include "src/SystemManager.h"
#include "src/utility.h"

// Communication
#include "src/modules/MessageCenter.h"
#include "src/modules/DebugLog.h"
#include "src/modules/DCMotorBringup.h"
#include "src/modules/LoopMonitor.h"
#include "src/modules/MotorControlCoordinator.h"
#include "src/modules/PersistentStorage.h"
#include "src/modules/SafetyManager.h"
#include "src/modules/StatusReporter.h"
#include "src/messages/TLV_Payloads.h"

// DC motor control
#include "src/modules/EncoderCounter.h"
#include "src/drivers/DCMotor.h"

// Stepper and servo control
#include "src/modules/StepperManager.h"
#include "src/drivers/StepperMotor.h"
#include "src/drivers/ServoController.h"

// Sensors and user I/O
#include "src/modules/SensorManager.h"
#include "src/modules/UserIO.h"
#include "src/drivers/IMUDriver.h"
#include "src/drivers/NeoPixelDriver.h"

// ============================================================================
// Compiler Guards
// ============================================================================
#ifndef SERIAL_RX_BUFFER_SIZE
#error "SERIAL_RX_BUFFER_SIZE not defined. Add -DSERIAL_RX_BUFFER_SIZE=256 to compiler flags."
#elif SERIAL_RX_BUFFER_SIZE < 256
#error "SERIAL_RX_BUFFER_SIZE < 256, which is recommanded for the current high-rate RPi UART link."
#endif

#ifndef SERIAL_TX_BUFFER_SIZE
#error "SERIAL_TX_BUFFER_SIZE not defined. Add -DSERIAL_TX_BUFFER_SIZE=256 to compiler flags."
#elif SERIAL_TX_BUFFER_SIZE < 256
#error "SERIAL_TX_BUFFER_SIZE < 256, which is recommanded for the current high-rate RPi UART link."
#endif

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Encoder instances (2x or 4x mode per config.h)
#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif

#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif

#if ENCODER_3_MODE == ENCODER_2X
EncoderCounter2x encoder3;
#else
EncoderCounter4x encoder3;
#endif

#if ENCODER_4_MODE == ENCODER_2X
EncoderCounter2x encoder4;
#else
EncoderCounter4x encoder4;
#endif

// Motor controller arrays
DCMotor dcMotors[NUM_DC_MOTORS];
static volatile uint32_t g_uartDor2Count = 0;
static volatile uint32_t g_uartFe2Count = 0;

static void snapshotUart2FaultCounts(uint32_t &dor2, uint32_t &fe2) {
  noInterrupts();
  dor2 = g_uartDor2Count;
  fe2 = g_uartFe2Count;
  interrupts();
}

static void appendFaultToken(char *buffer, size_t size, bool &first, const char *token) {
  if (buffer == nullptr || token == nullptr || size == 0U) {
    return;
  }
  if (!first) {
    strlcat(buffer, ",", size);
  }
  strlcat(buffer, token, size);
  first = false;
}

static void formatLoopFaultMask(uint8_t mask, char *buffer, size_t size) {
  if (buffer == nullptr || size == 0U) {
    return;
  }

  buffer[0] = '\0';
  bool first = true;
  if ((mask & LOOP_FAULT_PID_ISR) != 0U) {
    appendFaultToken(buffer, size, first, "pid");
  }
  if ((mask & LOOP_FAULT_STEPPER_ISR) != 0U) {
    appendFaultToken(buffer, size, first, "step");
  }
  if ((mask & LOOP_FAULT_MOTOR_TASK) != 0U) {
    appendFaultToken(buffer, size, first, "motor");
  }
  if ((mask & LOOP_FAULT_SENSOR_ISR) != 0U) {
    appendFaultToken(buffer, size, first, "sensor");
  }
  if ((mask & LOOP_FAULT_UART_TASK) != 0U) {
    appendFaultToken(buffer, size, first, "uart");
  }
  if ((mask & LOOP_FAULT_USERIO) != 0U) {
    appendFaultToken(buffer, size, first, "io");
  }
  if ((mask & LOOP_FAULT_PID_ROUND) != 0U) {
    appendFaultToken(buffer, size, first, "pidr");
  }
  if (first) {
    strlcpy(buffer, "none", size);
  }
}

static void appendLoopFaultDelta(char *buffer, size_t size, bool &first, const char *token, uint16_t delta) {
  if (buffer == nullptr || token == nullptr || size == 0U || delta == 0U) {
    return;
  }
  if (!first) {
    strlcat(buffer, " ", size);
  }
  char tokenBuf[20];
  snprintf(tokenBuf, sizeof(tokenBuf), "%s+%u", token, (unsigned)delta);
  strlcat(buffer, tokenBuf, size);
  first = false;
}

static bool hasActiveDcControl() {
  if (SystemManager::getState() != SYS_STATE_RUNNING) {
    return false;
  }

  for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    if (dcMotors[i].isEnabled()) {
      return true;
    }
  }

  return false;
}

static bool fastDrainUart() {
  int before = RPI_SERIAL.available();
  if (before <= 0) {
    return false;
  }
  MessageCenter::drainUart();
  return RPI_SERIAL.available() != before;
}

static bool fastDrainTx() {
  uint16_t before = MessageCenter::getTxPendingBytes();
  if (before == 0U) {
    return false;
  }
  MessageCenter::drainTx();
  return MessageCenter::getTxPendingBytes() != before;
}

static bool fastMotorCompute() {
  uint8_t seqBefore = MotorControlCoordinator::getComputeSeq();
  bool busyBefore = MotorControlCoordinator::isComputeBusy();
  taskMotors();
  return (MotorControlCoordinator::getComputeSeq() != seqBefore) ||
         (MotorControlCoordinator::isComputeBusy() != busyBefore);
}

static bool fastStatusChunk() {
#if STATUS_REPORTER_ENABLED
  if (hasActiveDcControl() && MotorControlCoordinator::hasPendingRound()) {
    return false;
  }
  uint16_t before = DebugLog::getQueuedBytes();
  StatusReporter::emitChunk();
  return DebugLog::getQueuedBytes() != before;
#else
  return false;
#endif
}

static bool fastFaultEvents() {
#if !FAULT_EVENT_LOG_ENABLED
  return false;
#else
  static uint16_t lastLoopFaultCounts[LOOP_SLOT_COUNT] = {};
  static uint16_t lastPidRoundFaultCount = 0U;
  static uint32_t lastMissedCount = 0U;
  static uint32_t lastLateCount = 0U;
  static uint32_t lastReusedCount = 0U;
  static uint32_t lastCrossCount = 0U;
  static uint32_t lastDorCount = 0U;
  static uint32_t lastFeCount = 0U;
  static uint16_t lastCrcCount = 0U;
  static uint16_t lastFrameCount = 0U;
  static uint16_t lastTlvCount = 0U;
  static uint16_t lastOversizeCount = 0U;
  static uint32_t lastTxDroppedFrames = 0U;
  static uint32_t lastEmitMs = 0U;

  if (DebugLog::getQueuedBytes() > (DEBUG_LOG_BUFFER_SIZE / 2U)) {
    return false;
  }

  uint32_t now = millis();
  if ((uint32_t)(now - lastEmitMs) < FAULT_EVENT_MIN_INTERVAL_MS) {
    return false;
  }

  uint16_t rxAvailable = (uint16_t)RPI_SERIAL.available();
  uint16_t txPending = MessageCenter::getTxPendingBytes();
  uint32_t txDroppedFrames = MessageCenter::getTxDroppedFrames();
  uint32_t txDroppedDelta = txDroppedFrames - lastTxDroppedFrames;

  char loopDeltaBuf[64];
  loopDeltaBuf[0] = '\0';
  bool firstLoopDelta = true;
  uint16_t loopDeltas[LOOP_SLOT_COUNT] = {};
  static const char *const kLoopTokens[LOOP_SLOT_COUNT] = {
    "pid", "step", "motor", "sensor", "uart", "io"
  };
  for (uint8_t i = 0; i < LOOP_SLOT_COUNT; ++i) {
    uint16_t faultCount = LoopMonitor::getFaultCount((LoopSlot)i);
    uint16_t delta = (uint16_t)(faultCount - lastLoopFaultCounts[i]);
    loopDeltas[i] = delta;
    lastLoopFaultCounts[i] = faultCount;
    appendLoopFaultDelta(loopDeltaBuf, sizeof(loopDeltaBuf), firstLoopDelta, kLoopTokens[i], delta);
  }
  uint16_t pidRoundFaultCount = LoopMonitor::getPidRoundFaultCount();
  uint16_t pidRoundDelta = (uint16_t)(pidRoundFaultCount - lastPidRoundFaultCount);
  lastPidRoundFaultCount = pidRoundFaultCount;
  appendLoopFaultDelta(loopDeltaBuf, sizeof(loopDeltaBuf), firstLoopDelta, "pidr", pidRoundDelta);

  if (!firstLoopDelta) {
    DebugLog::printf_P(PSTR("[LOOP] %s | rx=%u tx=%u qfull+%lu\n"),
                       loopDeltaBuf,
                       (unsigned)rxAvailable,
                       (unsigned)txPending,
                       (unsigned long)txDroppedDelta);
    lastTxDroppedFrames = txDroppedFrames;
    lastEmitMs = now;
    return true;
  }

  uint32_t roundCount = 0;
  uint32_t requestedRound = 0;
  uint32_t computedRound = 0;
  uint32_t appliedRound = 0;
  uint8_t slot = 0;
  uint8_t computeSeq = 0;
  uint8_t appliedSeq = 0;
  uint32_t missedCount = 0;
  uint32_t lateCount = 0;
  uint32_t reusedCount = 0;
  uint32_t crossCount = 0;
  bool computeBusy = false;
  MotorControlCoordinator::snapshot(roundCount,
                                    requestedRound,
                                    computedRound,
                                    appliedRound,
                                    slot,
                                    computeSeq,
                                    appliedSeq,
                                    missedCount,
                                    lateCount,
                                    reusedCount,
                                    crossCount,
                                    computeBusy);

  uint32_t missedDelta = missedCount - lastMissedCount;
  uint32_t lateDelta = lateCount - lastLateCount;
  uint32_t reusedDelta = reusedCount - lastReusedCount;
  uint32_t crossDelta = crossCount - lastCrossCount;
  if (missedDelta != 0U || lateDelta != 0U || reusedDelta != 0U || crossDelta != 0U) {
    DebugLog::printf_P(PSTR("[CTRL] miss+%lu late+%lu reuse+%lu cross+%lu | rx=%u tx=%u qfull+%lu\n"),
                       (unsigned long)missedDelta,
                       (unsigned long)lateDelta,
                       (unsigned long)reusedDelta,
                       (unsigned long)crossDelta,
                       (unsigned)rxAvailable,
                       (unsigned)txPending,
                       (unsigned long)txDroppedDelta);
    lastMissedCount = missedCount;
    lastLateCount = lateCount;
    lastReusedCount = reusedCount;
    lastCrossCount = crossCount;
    lastTxDroppedFrames = txDroppedFrames;
    lastEmitMs = now;
    return true;
  }
  lastMissedCount = missedCount;
  lastLateCount = lateCount;
  lastReusedCount = reusedCount;
  lastCrossCount = crossCount;

  uint32_t dorCount = 0;
  uint32_t feCount = 0;
  snapshotUart2FaultCounts(dorCount, feCount);
  uint16_t crcCount = MessageCenter::getCrcErrorCount();
  uint16_t frameCount = MessageCenter::getFrameLenErrorCount();
  uint16_t tlvCount = MessageCenter::getTlvErrorCount();
  uint16_t oversizeCount = MessageCenter::getOversizeErrorCount();
  uint32_t dorDelta = dorCount - lastDorCount;
  uint32_t feDelta = feCount - lastFeCount;
  uint16_t crcDelta = (uint16_t)(crcCount - lastCrcCount);
  uint16_t frameDelta = (uint16_t)(frameCount - lastFrameCount);
  uint16_t tlvDelta = (uint16_t)(tlvCount - lastTlvCount);
  uint16_t oversizeDelta = (uint16_t)(oversizeCount - lastOversizeCount);
  if (dorDelta != 0U || feDelta != 0U || crcDelta != 0U ||
      frameDelta != 0U || tlvDelta != 0U || oversizeDelta != 0U) {
    DebugLog::printf_P(PSTR("[UART] dor+%lu fe+%lu crc+%u frame+%u tlv+%u oversize+%u | rx=%u tx=%u qfull+%lu\n"),
                       (unsigned long)dorDelta,
                       (unsigned long)feDelta,
                       (unsigned)crcDelta,
                       (unsigned)frameDelta,
                       (unsigned)tlvDelta,
                       (unsigned)oversizeDelta,
                       (unsigned)rxAvailable,
                       (unsigned)txPending,
                       (unsigned long)txDroppedDelta);
    lastDorCount = dorCount;
    lastFeCount = feCount;
    lastCrcCount = crcCount;
    lastFrameCount = frameCount;
    lastTlvCount = tlvCount;
    lastOversizeCount = oversizeCount;
    lastTxDroppedFrames = txDroppedFrames;
    lastEmitMs = now;
    return true;
  }
  lastDorCount = dorCount;
  lastFeCount = feCount;
  lastCrcCount = crcCount;
  lastFrameCount = frameCount;
  lastTlvCount = tlvCount;
  lastOversizeCount = oversizeCount;

  const bool rxBacklogged = rxAvailable >= 16U;
  const bool txBacklogged = txPending >= (TX_FRAME_SOFT_LIMIT / 2U);
  const bool sensorResourceDelta = loopDeltas[SLOT_SENSOR_ISR] != 0U;
  const bool uartResourceDelta = loopDeltas[SLOT_UART_TASK] != 0U;
  if (rxBacklogged || txBacklogged || txDroppedDelta != 0U ||
      sensorResourceDelta || uartResourceDelta) {
    DebugLog::printf_P(PSTR("[RES] rx=%u tx=%u qfull+%lu sensor+%u uart+%u loopMax=%uus\n"),
                       (unsigned)rxAvailable,
                       (unsigned)txPending,
                       (unsigned long)txDroppedDelta,
                       (unsigned)loopDeltas[SLOT_SENSOR_ISR],
                       (unsigned)loopDeltas[SLOT_UART_TASK],
                       (unsigned)MessageCenter::getLoopTimeMaxUs());
    lastTxDroppedFrames = txDroppedFrames;
    lastEmitMs = now;
    return true;
  }

  lastTxDroppedFrames = txDroppedFrames;

  return false;
#endif
}

static bool fastDebugFlush() {
  if (hasActiveDcControl() && MotorControlCoordinator::hasPendingRound()) {
    return false;
  }
  uint16_t before = DebugLog::getQueuedBytes();
  if (before == 0U) {
    return false;
  }

  uint32_t flushStartUs = micros();
  DebugLog::flush();
  StatusReporter::recordFlushTimingUs(Utility::clampElapsedUs(micros() - flushStartUs));
  return DebugLog::getQueuedBytes() != before;
}

static void serviceControlFirstFastLaneBurst() {
  // Allow at most two motor compute chunks before forcing a UART service
  // opportunity. This still keeps motor compute highest priority, but avoids
  // long TX/RX service gaps while DC motors are active.
  constexpr uint8_t kMaxControlFastLaneBurst = 2;

  uint8_t burstCount = 0;
  while (Scheduler::serviceFastLane()) {
    if (!MotorControlCoordinator::hasPendingRound()) {
      break;
    }
    if (++burstCount >= kMaxControlFastLaneBurst) {
      fastDrainUart();
      fastDrainTx();
      break;
    }
  }
}

static void servicePeriodicBurst() {
  // One tickPeriodic() call only runs one overdue soft task. In RUNNING, UART,
  // safety, and motor feedback can all be due before the 100 Hz sensor task.
  // Drain a short periodic backlog each loop pass so odometry/IMU work does
  // not fall far behind while the DC controller is active.
  constexpr uint8_t kMaxPeriodicBurst = 4;

  for (uint8_t i = 0; i < kMaxPeriodicBurst; ++i) {
    Scheduler::tickPeriodic();
    if (MotorControlCoordinator::hasPendingRound()) {
      serviceControlFirstFastLaneBurst();
    }
  }
}

/**
 * @brief Timer1 overflow ISR — short round-robin DC apply slot.
 *
 * One motor is serviced per 800 Hz tick, giving each DC motor a 200 Hz
 * latch/apply cadence while keeping the per-slice ISR body comfortably inside
 * the UART safety budget. The matching loop-side compute now runs one motor at
 * a time in the same round-robin order instead of batching all four motors
 * into one 5 ms software round.
 */
ISR(TIMER1_OVF_vect) {
  uint16_t t0 = Utility::readTimer1CounterTicks(); // Sample time stamp for the LoopMonitor
  bool running = hasActiveDcControl();
  uint16_t pidRoundSpanUs =
      MotorControlCoordinator::servicePidIsrSlice(dcMotors, NUM_DC_MOTORS, running);

  if (running && pidRoundSpanUs > 0U) {
    LoopMonitor::recordPidRoundSpan(pidRoundSpanUs, true);
  } else if (!running) {
    LoopMonitor::recordPidRoundSpan(0, false);
  }

  LoopMonitor::record(SLOT_PID_ISR,
                      Utility::timerTicksToUs(
                          (uint16_t)(Utility::readTimer1CounterTicks() - t0)));
}

/**
 * @brief Timer3 overflow ISR — stepper pulse generation only.
 *
 * The actual per-stepper work lives in StepperManager::timerISR(). Keeping the
 * vector in main firmware makes the active hard-RT paths explicit.
 */
ISR(TIMER3_OVF_vect) {
  // Same Timer3 hardware-counter timing pattern as the Timer1 PID ISR above.
  uint16_t t0 = Utility::readTimer3CounterTicks();
  Utility::Uart2FaultEdges uartFaultEdges = Utility::sampleUart2FaultEdges();
  if (uartFaultEdges.dorRising) {
    g_uartDor2Count++;
  }
  if (uartFaultEdges.feRising) {
    g_uartFe2Count++;
  }

  StepperManager::timerISR();
  LoopMonitor::record(SLOT_STEPPER_ISR,
                      Utility::timerTicksToUs(
                          (uint16_t)(Utility::readTimer3CounterTicks() - t0)));
}

// ============================================================================
// SOFT TASK — taskUART (100 Hz, millis-based)
// ============================================================================

/**
 * @brief UART RX/TX task — runs in loop() so USART2_RX_vect stays enabled.
 *
 * Do NOT move into TIMER1_OVF_vect (see ISR comment above for explanation).
 * Registered at prior0 (highest) so it preempts all other soft tasks.
 */ 
void taskUART() {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_TASK, HIGH);
#endif

  uint32_t taskStartUs = micros();

#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_RX, HIGH);
#endif
  MessageCenter::processIncoming();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_RX, LOW);
  digitalWrite(DEBUG_PIN_UART_TX, HIGH);
#endif
  MessageCenter::processDeferred();
  MessageCenter::sendTelemetry();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_TX, LOW);
#endif

  uint16_t elapsed = Utility::clampElapsedUs(micros() - taskStartUs);
  MessageCenter::recordLoopTime(elapsed);
  LoopMonitor::record(SLOT_UART_TASK, elapsed);

#if DEBUG_PINS_ENABLED
  // A9 (UART_LATE): pulse when wall-clock > 10 ms. This fires due to ISR
  // preemption inflation, NOT actual UART slowness — the PID loop is unaffected.
  if (elapsed > UART_TASK_BUDGET_US) {
    digitalWrite(DEBUG_PIN_UART_LATE, HIGH);
    digitalWrite(DEBUG_PIN_UART_LATE, LOW);
  }
  digitalWrite(DEBUG_PIN_UART_TASK, LOW);
#endif
}

// ============================================================================
// SOFT TASK — taskSafety (100 Hz, millis-based)
// ============================================================================

/**
 * @brief Safety checks outside ISR context.
 *
 * SafetyManager owns only fault policy. User button / limit GPIO sampling runs
 * in taskSensors() so task ownership matches subsystem responsibility.
 */
void taskSafety() {
  SafetyManager::check();
}

// ============================================================================
// SOFT TASK — taskMotors (per-slot mixed round-robin, loop-owned)
// ============================================================================

/**
 * @brief Run one software DC compute slot matching the pending ISR request.
 *
 * The Timer1 ISR latches/applies one motor every 800 Hz slot and queues that
 * same motor for one loop-owned compute pass. This keeps each motor at 200 Hz
 * control while avoiding a 4-motor batch compute in a single loop pass.
 */
void taskMotors() {
  if (!hasActiveDcControl()) {
    MotorControlCoordinator::resetForNonRunningTask();
    return;
  }

  uint32_t requestedRound = 0;
  uint8_t slotSnapshot = 0;
  if (!MotorControlCoordinator::beginCompute(requestedRound, slotSnapshot)) {
    return;
  }

  uint32_t taskStartUs = micros();
  do {
    dcMotors[slotSnapshot].service();
    MotorControlCoordinator::finishCompute(requestedRound);

    if (!MotorControlCoordinator::hasPendingRound()) {
      break;
    }

    if (Utility::clampElapsedUs(micros() - taskStartUs) >= MOTOR_TASK_CATCHUP_BUDGET_US) {
      break;
    }
  } while (MotorControlCoordinator::beginCompute(requestedRound, slotSnapshot));

  LoopMonitor::record(SLOT_MOTOR_TASK, Utility::clampElapsedUs(micros() - taskStartUs));
}

// ============================================================================
// SOFT TASK — taskMotorFeedback (fixed-rate DC encoder feedback)
// ============================================================================

/**
 * @brief Refresh encoder position/velocity feedback for all DC motors.
 *
 * This loop-owned task runs at the same 200 Hz cadence as the per-motor mixed
 * control period, but applies to all motors regardless of enable state. It is
 * the single source of truth for cached DC velocity used by PID, telemetry,
 * and the 100 Hz odometry/sensor task.
 */
void taskMotorFeedback() {
  bool odomNeedsReseed = false;
  const uint8_t leftMotorId = RobotKinematics::getLeftMotorId();
  const uint8_t rightMotorId = RobotKinematics::getRightMotorId();
  for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    dcMotors[i].refreshFeedback();
    if ((i == leftMotorId || i == rightMotorId) &&
        dcMotors[i].consumeEncoderResetEvent()) {
      odomNeedsReseed = true;
    }
  }

  const int32_t leftTicks = dcMotors[leftMotorId].getPosition();
  const int32_t rightTicks = dcMotors[rightMotorId].getPosition();
  if (odomNeedsReseed) {
    RobotKinematics::reseed(leftTicks, rightTicks);
  }
}

// ============================================================================
// SOFT TASK — taskSensors (100 Hz, millis-based)
// ============================================================================

/**
 * @brief Sensor dispatch task — runs in loop() so I2C/ADC never execute in ISR context.
 *
 * This is the main bring-up change relative to the older architecture: Timer4
 * still provides motor PWM, but its overflow interrupt is disabled. The same
 * 100 Hz task also integrates odometry and refreshes the cached user button /
 * limit switch states.
 */
void taskSensors() {
  uint32_t t0 = micros();
  SensorManager::tick();

  const uint8_t leftMotorId = RobotKinematics::getLeftMotorId();
  const uint8_t rightMotorId = RobotKinematics::getRightMotorId();
  RobotKinematics::update(
    dcMotors[leftMotorId].getPosition(),
    dcMotors[rightMotorId].getPosition(),
    dcMotors[leftMotorId].getVelocity(),
    dcMotors[rightMotorId].getVelocity());

  UserIO::sampleInputs();
  LoopMonitor::record(SLOT_SENSOR_ISR, Utility::clampElapsedUs(micros() - t0));
}

// ============================================================================
// SOFT TASK — taskUserIO (20 Hz, millis-based)
// ============================================================================

/**
 * @brief User I/O updates (20 Hz, soft scheduler)
 *
 * Runs user-controlled LED animations and queued NeoPixel state rendering.
 * Button / limit sampling is handled in taskSensors() at 100 Hz.
 */
void taskUserIO() {
  uint32_t t0 = micros();
  UserIO::serviceTask();
  LoopMonitor::record(SLOT_USERIO, Utility::clampElapsedUs(micros() - t0));
}

// ============================================================================
// SOFT TASK — taskI2CRetry (1 Hz, lowest priority, millis-based)
// ============================================================================

/**
 * @brief Periodic I2C device retry task
 *
 * Attempts to reinitialize optional I2C devices (e.g., PCA9685 servo driver)
 * that may have been absent at startup but are plugged in during runtime.
 *
 * Runs at very low priority (priority=7, the lowest) and low frequency (1 Hz)
 * to ensure it never blocks motor control, sensor reads, or other critical tasks.
 * If a device is already initialized, this task is a no-op and costs minimal time.
 */
void taskI2CRetry() {
  if (!ServoController::isInitialized()) {
    ServoController::init();
    if (ServoController::isInitialized()) {
      ServoController::enable();
#ifdef DEBUG_SERVO
      DEBUG_SERIAL.println(F("[I2C Retry] PCA9685 servo driver initialized"));
#endif
    }
  }
}

// ============================================================================
// ENCODER ISR TRAMPOLINES
// ============================================================================

/**
 * @brief Encoder ISR wrappers
 *
 * These are minimal ISR wrappers that forward calls to encoder objects.
 * Encoder ISRs must be global functions (not class methods) to use with
 * attachInterrupt().
 */

void encoderISR_M1_A() {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, HIGH);
#endif
  encoder1.onInterruptA();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
#endif
}

void encoderISR_M1_B() {
  encoder1.onInterruptB();
}

void encoderISR_M2_A() {
  encoder2.onInterruptA();
}

void encoderISR_M2_B() {
  encoder2.onInterruptB();
}

// M3/M4 encoder pins (A14, A15, 11, 12) are on PCINT buses, not hardware
// INT pins. attachInterrupt() does not work for them on Mega, so the wrappers
// below are dispatched from the PCINT2/PCINT0 vectors.
void encoderISR_M3() {
  encoder3.onInterruptA();
}

void encoderISR_M4() {
  encoder4.onInterruptA();
}

ISR(PCINT2_vect) {
  encoderISR_M3();
}

ISR(PCINT0_vect) {
  encoderISR_M4();
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  int8_t taskId = -1;
  int8_t fastTaskId = -1;

  // 1. Enter INIT before any module starts.
  SystemManager::init();

  // 2. Bring up debug serial and runtime-owned services.
  DEBUG_SERIAL_PORT.begin(DEBUG_BAUD_RATE);
  while (!DEBUG_SERIAL_PORT && millis() < 2000) {
    ; // Wait for serial port to connect (up to 2 seconds)
  }
  DebugLog::init();
  DebugLog::setPassthrough(true);
  LoopMonitor::init();
  MotorControlCoordinator::init();
  PersistentStorage::init();
  StatusReporter::init(snapshotUart2FaultCounts, MotorControlCoordinator::snapshot);
  Utility::printStartupBanner();

  // 3. Initialize scheduler and optional debug pins.
  DEBUG_SERIAL.println(F("[Setup] Initializing soft scheduler..."));
  Scheduler::init();
  Utility::initDebugPins();

  // 4. Initialize main modules and actuators.
  DEBUG_SERIAL.println(F("[Setup] Initializing UART communication..."));
  MessageCenter::init();

  DEBUG_SERIAL.println(F("[Setup] Initializing sensors..."));
  SensorManager::init();

  DEBUG_SERIAL.println(F("[Setup] Initializing user I/O..."));
  UserIO::init();

#if SERVO_CONTROLLER_ENABLED
  DEBUG_SERIAL.println(F("[Setup] Initializing servo controller..."));
  ServoController::init();
  if (ServoController::isInitialized()) {
    DEBUG_SERIAL.println(F("  - PCA9685 initialized (50Hz PWM)"));
  } else {
    DEBUG_SERIAL.println(F("  - PCA9685 not detected"));
  }
#endif

  DEBUG_SERIAL.println(F("[Setup] Initializing stepper motors..."));
  StepperManager::init();

  // 5. Initialize DC motors, encoders, and their interrupt attachment.
  DEBUG_SERIAL.println(F("[Setup] Initializing DC motors and encoders..."));
  DEBUG_SERIAL.print(F("  - Encoder resolution: "));
  DEBUG_SERIAL.print(DCMotorBringup::countsPerRev());
  DEBUG_SERIAL.println(F(" counts/rev"));
  DCMotorBringup::initAll(dcMotors,
                          encoder1,
                          encoder2,
                          encoder3,
                          encoder4);
  DEBUG_SERIAL.println(F("  - 4 DC motors initialized"));

  DEBUG_SERIAL.println(F("[Setup] Attaching encoder interrupts..."));
  ISRScheduler::attachDcEncoderInterrupts(encoderISR_M1_A,
                                          encoderISR_M1_B,
                                          encoderISR_M2_A,
                                          encoderISR_M2_B);
  DEBUG_SERIAL.println(F("  - Motor 1/2 encoder ISRs attached (hardware INT)"));
  ISRScheduler::attachDcEncoderPcints();
  DEBUG_SERIAL.println(F("  - Motor 3/4 encoder ISRs attached (PCINT)"));

  // 6. Register cooperative fast-lane and periodic soft tasks.
  DEBUG_SERIAL.println(F("[Setup] Registering scheduler tasks..."));

  fastTaskId = Scheduler::registerFastTask(fastMotorCompute, 0);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast MotorCompute: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }

  fastTaskId = Scheduler::registerFastTask(fastDrainUart, 1);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast UART RX: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }

  fastTaskId = Scheduler::registerFastTask(fastDrainTx, 2);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast UART TX: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }

  fastTaskId = Scheduler::registerFastTask(fastFaultEvents, 3);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast FaultEvents: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }

#if STATUS_REPORTER_ENABLED
  fastTaskId = Scheduler::registerFastTask(fastStatusChunk, 4);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast StatusChunk: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }
#endif

  fastTaskId = Scheduler::registerFastTask(fastDebugFlush, 5);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast DebugFlush: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }

  taskId = Scheduler::registerTask(taskUART, 1000 / UART_COMMS_FREQ_HZ, 0);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - UART: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / UART_COMMS_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (50Hz)"));
  }

  taskId = Scheduler::registerTask(taskSafety, 1000 / SENSOR_UPDATE_FREQ_HZ, 1);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Safety: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (100Hz)"));
  }

  DEBUG_SERIAL.println(F("  - Motors: per-slot mixed round-robin from Timer1 requests"));

  taskId = Scheduler::registerTask(taskMotorFeedback, 1000 / MOTOR_UPDATE_FREQ_HZ, 2);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Motor Feedback: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / MOTOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (200Hz)"));
  }

  taskId = Scheduler::registerTask(taskSensors, 1000 / SENSOR_UPDATE_FREQ_HZ, 3);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Sensors: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (100Hz)"));
  }

  taskId = Scheduler::registerTask(taskUserIO, 1000 / USER_IO_FREQ_HZ, 4);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - User I/O: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / USER_IO_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms"));
  }

#if STATUS_REPORTER_ENABLED
  taskId = Scheduler::registerTask(StatusReporter::task, 1000 / STATUS_REPORT_HZ, 5);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Status Reporter: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / STATUS_REPORT_HZ);
    DEBUG_SERIAL.println(F("ms"));
  }
#else
  DEBUG_SERIAL.println(F("  - Status Reporter: disabled"));
#endif

  taskId = Scheduler::registerTask(taskI2CRetry, 1000, 7);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - I2C Retry: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.println(F(" @ 1000ms (1Hz, lowest priority)"));
  }

  // 7. Move into IDLE before enabling hard real-time services.
  if (SystemManager::triggerBootCompleted()) {
    DEBUG_SERIAL.println(F("[Setup] System state -> IDLE"));
  } else {
    DEBUG_SERIAL.println(F("[Setup] System state transition to IDLE rejected"));
  }

  // 8. Start hard real-time timer services last.
  DEBUG_SERIAL.println(F("[Setup] Starting hard real-time ISRs (Timer1 + Timer3; Timer2/Timer4 PWM)..."));
  noInterrupts();
  ISRScheduler::configureTimer1DcSlotISR();
  ISRScheduler::configureTimer2PwmOnly();
  ISRScheduler::configureTimer4PwmOnly();
  interrupts();
  UserIO::syncOutputs();
  Utility::printStartupSummary();
  DebugLog::setPassthrough(false);
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  StatusReporter::recordLoopGap();
  StatusReporter::updateWindowPeaks();
  serviceControlFirstFastLaneBurst();
  servicePeriodicBurst();
  serviceControlFirstFastLaneBurst();
}
