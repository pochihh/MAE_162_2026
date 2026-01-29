/**
 * @file arduino.ino
 * @brief Main entry point for Arduino Mega 2560 firmware
 *
 * Educational robotics platform firmware for MAE 162 course.
 * Provides real-time motor control, sensor integration, and
 * communication with Raspberry Pi 5 via TLV protocol over UART.
 *
 * Architecture:
 * - Cooperative scheduler using Timer1 @ 1kHz
 * - Hardware interrupts for encoders (INT0-INT5)
 * - Timer3 @ 10kHz for stepper pulse generation
 * - Priority-based task execution in main loop
 *
 * Initialization Order (setup):
 * 1. Debug serial (Serial0)
 * 2. Scheduler (Timer1)
 * 3. MessageCenter (Serial2 + TLV codec)
 * 4. SensorManager (I2C, ADC)
 * 5. UserIO (GPIO, NeoPixel)
 * 6. ServoController (PCA9685 via I2C)
 * 7. StepperManager (Timer3)
 * 8. DC Motors (PWM pins, encoder counters)
 * 9. Attach encoder ISRs
 * 10. Register scheduler tasks
 *
 * Main Loop:
 * - Scheduler::tick() executes highest-priority ready task
 * - Task priorities: 0=DC PID, 1=UART Comms, 2=Sensors, 3=User I/O
 */

// ============================================================================
// INCLUDES
// ============================================================================

#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"

// Phase 2+ includes
#include "src/messages/MessageCenter.h"
#include "src/messages/TLV_Payloads.h"

// Phase 3+ includes
#include "src/modules/EncoderCounter.h"
#include "src/modules/VelocityEstimator.h"
#include "src/drivers/DCMotor.h"

// Phase 4+ includes
#include "src/modules/StepperManager.h"
#include "src/drivers/StepperMotor.h"
#include "src/drivers/ServoController.h"

// Phase 5+ includes
#include "src/modules/SensorManager.h"
#include "src/modules/UserIO.h"
#include "src/drivers/IMUDriver.h"
#include "src/drivers/NeoPixelDriver.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Phase 3: DC Motors, Encoders, and Velocity Estimators

// Encoder instances (2x mode for current hardware)
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

// Velocity estimators (edge-time algorithm)
EdgeTimeVelocityEstimator velocityEst1;
EdgeTimeVelocityEstimator velocityEst2;
EdgeTimeVelocityEstimator velocityEst3;
EdgeTimeVelocityEstimator velocityEst4;

// DC motor controllers
DCMotor dcMotors[NUM_DC_MOTORS];

// ============================================================================
// TASK CALLBACKS
// ============================================================================

/**
 * @brief DC Motor PID control loop (200 Hz, Priority 0)
 *
 * Updates all enabled DC motor PID controllers.
 * Runs every 5ms (200Hz) with highest priority for smooth control.
 */
void taskDCMotorPID() {
#ifdef DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_PID_LOOP, HIGH);
#endif

  // Update all enabled DC motor PID controllers
#if DC_MOTOR_1_ENABLED
  dcMotors[0].update();
#endif

#if DC_MOTOR_2_ENABLED
  dcMotors[1].update();
#endif

#if DC_MOTOR_3_ENABLED
  dcMotors[2].update();
#endif

#if DC_MOTOR_4_ENABLED
  dcMotors[3].update();
#endif

#ifdef DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_PID_LOOP, LOW);
#endif
}

/**
 * @brief UART communication and safety timeout (100 Hz, Priority 1)
 *
 * Processes incoming TLV messages from Raspberry Pi.
 * Checks heartbeat timeout and disables motors if expired.
 * Runs every 10ms (100Hz).
 */
void taskUARTComms() {
  // Process incoming/outgoing TLV messages
  MessageCenter::processingTick();

  // Safety timeout check - disable all motors if heartbeat lost
  if (!MessageCenter::isHeartbeatValid()) {
    // Disable DC motors
#if DC_MOTOR_1_ENABLED
    dcMotors[0].disable();
#endif
#if DC_MOTOR_2_ENABLED
    dcMotors[1].disable();
#endif
#if DC_MOTOR_3_ENABLED
    dcMotors[2].disable();
#endif
#if DC_MOTOR_4_ENABLED
    dcMotors[3].disable();
#endif

    // Disable stepper motors
    StepperManager::disableAll();

    // Disable servo outputs
#if SERVO_CONTROLLER_ENABLED
    ServoController::disable();
#endif
  }
}

/**
 * @brief Sensor reading and data transmission (50 Hz, Priority 2)
 *
 * Reads all enabled sensors (IMU, voltage, encoders, etc.).
 * Queues outgoing sensor data packets to Raspberry Pi.
 * Runs every 20ms (50Hz).
 */
void taskSensorRead() {
  // Update all sensors
  SensorManager::update();

  // Future: Queue outgoing sensor data to RPi
  // MessageCenter::sendEncoderData();
  // MessageCenter::sendVoltageData();
  // if (IMU_ENABLED) {
  //   MessageCenter::sendIMUData();
  // }

  // Check for low battery and update status LED
  if (SensorManager::isBatteryLow()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 1000);
  }
}

/**
 * @brief User I/O updates (20 Hz, Priority 3)
 *
 * Updates LED animations (blink, breathe patterns).
 * Reads button states and limit switches.
 * Updates NeoPixel system status indicator.
 * Runs every 50ms (20Hz).
 */
void taskUserIO() {
  // Update all user I/O (buttons, LEDs, NeoPixel)
  UserIO::update();

  // Update system status on NeoPixel
  if (!MessageCenter::isHeartbeatValid()) {
    UserIO::setSystemStatus(STATUS_ERROR);  // Red for timeout
  } else if (SensorManager::isBatteryLow()) {
    UserIO::setSystemStatus(STATUS_WARNING);  // Yellow for low battery
  } else if (StepperManager::anyMoving() || dcMotors[0].getMode() != DC_MODE_DISABLED) {
    UserIO::setSystemStatus(STATUS_BUSY);  // Cyan for active
  } else {
    UserIO::setSystemStatus(STATUS_OK);  // Green for ready
  }
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  // ------------------------------------------------------------------------
  // 1. Initialize Debug Serial (USB)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
  while (!DEBUG_SERIAL && millis() < 2000) {
    ; // Wait for serial port to connect (up to 2 seconds)
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("  MAE 162 Robot Firmware v0.5.0"));
  DEBUG_SERIAL.println(F("  Phase 5: Sensors & User I/O"));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println();

  // ------------------------------------------------------------------------
  // 2. Initialize Scheduler (Timer1 @ 1kHz)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing scheduler..."));
  Scheduler::init();

#ifdef DEBUG_PINS_ENABLED
  // Configure debug pins for oscilloscope timing measurement
  pinMode(DEBUG_PIN_ENCODER_ISR, OUTPUT);
  pinMode(DEBUG_PIN_STEPPER_ISR, OUTPUT);
  pinMode(DEBUG_PIN_PID_LOOP, OUTPUT);
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
  digitalWrite(DEBUG_PIN_STEPPER_ISR, LOW);
  digitalWrite(DEBUG_PIN_PID_LOOP, LOW);
  DEBUG_SERIAL.println(F("[Setup] Debug pins configured (A7-A10)"));
#endif

  // ------------------------------------------------------------------------
  // 3. Initialize MessageCenter (UART + TLV)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing UART communication..."));
  MessageCenter::init();

  // ------------------------------------------------------------------------
  // Phase 5: Initialize SensorManager
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing sensors..."));
  SensorManager::init();

  // ------------------------------------------------------------------------
  // Phase 5: Initialize UserIO (LEDs, Buttons, NeoPixels)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing user I/O..."));
  UserIO::init();

  // ------------------------------------------------------------------------
  // Phase 4: Initialize ServoController (PCA9685)
  // ------------------------------------------------------------------------
#if SERVO_CONTROLLER_ENABLED
  DEBUG_SERIAL.println(F("[Setup] Initializing servo controller..."));
  ServoController::init();
  DEBUG_SERIAL.println(F("  - PCA9685 initialized (50Hz PWM)"));
#endif

  // ------------------------------------------------------------------------
  // Phase 4: Initialize StepperManager (Timer3 @ 10kHz)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing stepper motors..."));
  StepperManager::init();

  // ------------------------------------------------------------------------
  // Phase 3: Initialize DC Motors and Encoders
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing DC motors and encoders..."));

  // Calculate counts per revolution based on encoder mode
  uint16_t countsPerRev = ENCODER_PPR * encoder1.getResolutionMultiplier();
  DEBUG_SERIAL.print(F("  - Encoder resolution: "));
  DEBUG_SERIAL.print(countsPerRev);
  DEBUG_SERIAL.println(F(" counts/rev"));

  // Motor 1
#if DC_MOTOR_1_ENABLED
  encoder1.init(PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED);
  velocityEst1.init(countsPerRev);
  velocityEst1.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst1.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

  dcMotors[0].init(0, &encoder1, &velocityEst1, DC_MOTOR_1_DIR_INVERTED);
  dcMotors[0].setPins(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2);
  dcMotors[0].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[0].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 1 initialized"));
#endif

  // Motor 2
#if DC_MOTOR_2_ENABLED
  encoder2.init(PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED);
  velocityEst2.init(countsPerRev);
  velocityEst2.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst2.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

  dcMotors[1].init(1, &encoder2, &velocityEst2, DC_MOTOR_2_DIR_INVERTED);
  dcMotors[1].setPins(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2);
  dcMotors[1].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[1].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 2 initialized"));
#endif

  // Motor 3
#if DC_MOTOR_3_ENABLED
  encoder3.init(PIN_M3_ENC_A, PIN_M3_ENC_B, ENCODER_3_DIR_INVERTED);
  velocityEst3.init(countsPerRev);
  velocityEst3.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst3.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

  dcMotors[2].init(2, &encoder3, &velocityEst3, DC_MOTOR_3_DIR_INVERTED);
  dcMotors[2].setPins(PIN_M3_EN, PIN_M3_IN1, PIN_M3_IN2);
  dcMotors[2].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[2].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 3 initialized"));
#endif

  // Motor 4
#if DC_MOTOR_4_ENABLED
  encoder4.init(PIN_M4_ENC_A, PIN_M4_ENC_B, ENCODER_4_DIR_INVERTED);
  velocityEst4.init(countsPerRev);
  velocityEst4.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst4.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

  dcMotors[3].init(3, &encoder4, &velocityEst4, DC_MOTOR_4_DIR_INVERTED);
  dcMotors[3].setPins(PIN_M4_EN, PIN_M4_IN1, PIN_M4_IN2);
  dcMotors[3].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[3].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 4 initialized"));
#endif

  // ------------------------------------------------------------------------
  // Phase 3: Attach Encoder ISRs
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Attaching encoder interrupts..."));

#if DC_MOTOR_1_ENABLED
  attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 1 encoder ISR attached"));
#endif

#if DC_MOTOR_2_ENABLED
  attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 2 encoder ISR attached"));
#endif

#if DC_MOTOR_3_ENABLED
  attachInterrupt(digitalPinToInterrupt(PIN_M3_ENC_A), encoderISR_M3, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 3 encoder ISR attached"));
#endif

#if DC_MOTOR_4_ENABLED
  attachInterrupt(digitalPinToInterrupt(PIN_M4_ENC_A), encoderISR_M4, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 4 encoder ISR attached"));
#endif

  // ------------------------------------------------------------------------
  // 3. Register Scheduler Tasks
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Registering scheduler tasks..."));

  // Task registration:
  //   Scheduler::registerTask(callback, periodMs, priority)
  //
  // Priority levels (lower number = higher priority):
  //   0 - DC Motor PID (200 Hz, 5ms period)
  //   1 - UART Communication (100 Hz, 10ms period)
  //   2 - Sensor Reading (50 Hz, 20ms period)
  //   3 - User I/O (20 Hz, 50ms period)

  int8_t taskId;

  taskId = Scheduler::registerTask(taskDCMotorPID, 1000 / DC_PID_FREQ_HZ, 0);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - DC Motor PID: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / DC_PID_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (Priority 0)"));
  }

  taskId = Scheduler::registerTask(taskUARTComms, 1000 / UART_COMMS_FREQ_HZ, 1);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - UART Comms: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / UART_COMMS_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (Priority 1)"));
  }

  taskId = Scheduler::registerTask(taskSensorRead, 1000 / SENSOR_UPDATE_FREQ_HZ, 2);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Sensor Read: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (Priority 2)"));
  }

  taskId = Scheduler::registerTask(taskUserIO, 1000 / USER_IO_FREQ_HZ, 3);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - User I/O: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / USER_IO_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (Priority 3)"));
  }

  // ------------------------------------------------------------------------
  // Setup Complete
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("[Setup] Initialization complete!"));
  DEBUG_SERIAL.println(F("[Setup] Entering main loop..."));
  DEBUG_SERIAL.println();
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  // Execute highest-priority ready task
  Scheduler::tick();

  // Future: Add any non-time-critical housekeeping here
  // (e.g., debug output, watchdog reset, etc.)
}

// ============================================================================
// ENCODER ISR TRAMPOLINES (Phase 3)
// ============================================================================

// These are minimal ISR wrappers that forward calls to encoder objects.
// Encoder ISRs must be global functions (not class methods) to use with
// attachInterrupt().

void encoderISR_M1() {
#ifdef DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, HIGH);
#endif

  encoder1.onInterruptA();

#ifdef DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
#endif
}

void encoderISR_M2() {
  encoder2.onInterruptA();
}

void encoderISR_M3() {
  encoder3.onInterruptA();
}

void encoderISR_M4() {
  encoder4.onInterruptA();
}
