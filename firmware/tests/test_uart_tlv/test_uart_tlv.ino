/**
 * @file test_uart_tlv.ino
 * @brief Test sketch for UART TLV Communication (Phase 2)
 *
 * This test verifies the TLV protocol implementation by:
 * 1. Initializing the MessageCenter
 * 2. Receiving and processing incoming TLV packets (from RPi)
 * 3. Periodically sending status updates (to RPi)
 *
 * Expected Behavior:
 * - Debug output on Serial (115200) showing initialized state
 * - Periodic transmission of SYS_STATUS, ENCODER, and VOLTAGE packets on
 * Serial2
 * - Acknowledgement of received valid TLV packets (debug output)
 *
 * How to Test (Loopback):
 * - If you have a USB-TTL adapter connected to Serial2 (pins 16/17):
 *   - Connect TX to RX for loopback test (send packets to self)
 *   - OR connect to PC and use Python script to send TLV commands
 *
 * Verification:
 * 1. Compile success
 * 2. Debug output shows "MessageCenter Initialized"
 * 3. Sending packets does not crash the system
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/messages/MessageCenter.h"
#include "src/messages/TLV_Payloads.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// Debug output enabled in config.h via DEBUG_TLV_PACKETS

// ============================================================================
// TASKS
// ============================================================================

/**
 * @brief UART Communication Task (100Hz)
 * Handles incoming parsing and outgoing queue flushing
 */
void taskUART() {
  MessageCenter::processingTick();
}

/**
 * @brief Periodic Status Sender (1Hz)
 * Simulates sending telemetry data to RPi
 */
void taskSendStatus() {
  // Send System Status
  MessageCenter::sendSystemStatus();

  // Send Dummy Voltage Data
  MessageCenter::sendVoltageData();

  // Send Dummy Encoder Data
  MessageCenter::sendEncoderData();

  // Debug output to show we are alive
  DEBUG_SERIAL.println(F("[Test] Sent periodic status updates"));

  // Show heartbeat status
  if (MessageCenter::isHeartbeatValid()) {
    DEBUG_SERIAL.print(F("  - Heartbeat OK ("));
    DEBUG_SERIAL.print(MessageCenter::timeSinceHeartbeat());
    DEBUG_SERIAL.println(F("ms ago)"));
  } else {
    DEBUG_SERIAL.println(F("  - Heartbeat TIMEOUT"));
  }
}

/**
 * @brief Communication Quality Statistics (0.1Hz / 10 seconds)
 * Prints decode errors and communication statistics
 */
void taskPrintStats() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("  COMMUNICATION QUALITY STATISTICS"));
  DEBUG_SERIAL.println(F("========================================"));

  uint32_t framesRx = MessageCenter::getFramesReceived();
  uint32_t framesTx = MessageCenter::getMessagesSent();
  uint32_t decodeErr = MessageCenter::getDecodeErrors();
  uint32_t crcErr = MessageCenter::getCRCErrors();

  DEBUG_SERIAL.print(F("Frames Received:  "));
  DEBUG_SERIAL.println(framesRx);

  DEBUG_SERIAL.print(F("Frames Sent:      "));
  DEBUG_SERIAL.println(framesTx);

  DEBUG_SERIAL.print(F("Decode Errors:    "));
  DEBUG_SERIAL.print(decodeErr);
  if (framesRx + decodeErr > 0) {
    float errorRate = 100.0 * decodeErr / (framesRx + decodeErr);
    DEBUG_SERIAL.print(F(" ("));
    DEBUG_SERIAL.print(errorRate, 2);
    DEBUG_SERIAL.print(F("%)"));
  }
  DEBUG_SERIAL.println();

  DEBUG_SERIAL.print(F("CRC Errors:       "));
  DEBUG_SERIAL.print(crcErr);
  if (decodeErr > 0) {
    float crcRate = 100.0 * crcErr / decodeErr;
    DEBUG_SERIAL.print(F(" ("));
    DEBUG_SERIAL.print(crcRate, 1);
    DEBUG_SERIAL.print(F("% of errors)"));
  }
  DEBUG_SERIAL.println();

  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize Debug Serial
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
  while (!DEBUG_SERIAL && millis() < 2000)
    ; // Wait for connection

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("  UART TLV Protocol Test - Phase 2"));
  DEBUG_SERIAL.println(F("========================================"));

  // Initialize Scheduler
  Scheduler::init();
  DEBUG_SERIAL.println(F("[Setup] Scheduler initialized"));

  // Initialize MessageCenter
  MessageCenter::init();
  DEBUG_SERIAL.println(F("[Setup] MessageCenter initialized"));
  DEBUG_SERIAL.print(F("  - Serial2 @ "));
  DEBUG_SERIAL.print(RPI_BAUD_RATE);
  DEBUG_SERIAL.println(F(" baud"));
  DEBUG_SERIAL.print(F("  - Device ID: 0x"));
  DEBUG_SERIAL.println(DEVICE_ID, HEX);

  // Register Tasks
  Scheduler::registerTask(taskUART, 10, 1);          // 10ms period (100Hz)
  Scheduler::registerTask(taskSendStatus, 1000, 2);  // 1000ms period (1Hz)
  Scheduler::registerTask(taskPrintStats, 10000, 3); // 10000ms period (0.1Hz)

  DEBUG_SERIAL.println(F("[Setup] Tasks registered"));
  DEBUG_SERIAL.println(F("[Setup] Starting Main Loop..."));
}

// ============================================================================
// LOOP
// ============================================================================

void loop() { Scheduler::tick(); }
