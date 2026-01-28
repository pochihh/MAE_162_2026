#!/usr/bin/env python3
"""
test_uart_arduino.py - UART TLV Communication Test for Raspberry Pi

This script tests the TLV protocol communication between Raspberry Pi and Arduino
via UART (Serial2 @ 921600 baud on pins 16/17).

Usage:
    python3 test_uart_arduino.py [--port /dev/ttyAMA0] [--baud 921600]

Features:
- Sends periodic heartbeat to Arduino
- Receives and decodes Arduino responses (system status, sensor data)
- Interactive commands to test motor control
- Automatic message display and logging

Requirements:
    pip3 install pyserial

Developed for MAE 162 Educational Robotics Platform (Winter/Spring 2026)
"""

import sys
import time
import argparse
import struct
import ctypes
from typing import Optional

# Add tlvcodec to path

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

from tlvcodec.src.encoder import Encoder
from tlvcodec.src.decoder import Decoder, DecodeErrorCode
from TLV_TypeDefs import *

# ============================================================================
# CONFIGURATION
# ============================================================================

DEFAULT_PORT = '/dev/ttyAMA0'  # Raspberry Pi serial port (GPIO 14/15)
DEFAULT_BAUD = 500000  # Baud rate
DEVICE_ID = 0x01  # RPi device ID
HEARTBEAT_INTERVAL = 0.2  # Send heartbeat every 10 ms
ENABLE_CRC_CHECK = True  # Enable CRC checking

# ============================================================================
# TLV PAYLOAD STRUCTURES (matching Arduino implementation)
# ============================================================================

class PayloadHeartbeat(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp", ctypes.c_uint32),
        ("flags", ctypes.c_uint8),
    ]

class PayloadSystemStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("uptimeMs", ctypes.c_uint32),
        ("lastHeartbeatMs", ctypes.c_uint32),
        ("batteryMv", ctypes.c_uint16),
        ("rail5vMv", ctypes.c_uint16),
        ("servoRailMv", ctypes.c_uint16),
        ("errorFlags", ctypes.c_uint8),
        ("motorEnableMask", ctypes.c_uint8),
        ("stepperEnableMask", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
    ]

class PayloadSensorVoltage(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("batteryMv", ctypes.c_uint16),
        ("rail5vMv", ctypes.c_uint16),
        ("servoRailMv", ctypes.c_uint16),
        ("reserved", ctypes.c_uint16),
    ]

class PayloadSensorEncoder(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("position", ctypes.c_int32 * 4),
        ("velocity", ctypes.c_int32 * 4),
        ("timestamp", ctypes.c_uint32),
    ]

class PayloadDCEnable(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId", ctypes.c_uint8),
        ("enable", ctypes.c_uint8),
        ("mode", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
    ]

class PayloadDCSetVelocity(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
        ("targetVel", ctypes.c_int32),
        ("maxAccel", ctypes.c_int32),
    ]

class PayloadSetNeoPixel(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("index", ctypes.c_uint8),
        ("red", ctypes.c_uint8),
        ("green", ctypes.c_uint8),
        ("blue", ctypes.c_uint8),
    ]

# ============================================================================
# UART TEST CLASS
# ============================================================================

class UARTTest:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.encoder = Encoder(deviceId=DEVICE_ID, bufferSize=4096, crc=ENABLE_CRC_CHECK)
        self.decoder = Decoder(callback=self.decode_callback, crc=ENABLE_CRC_CHECK)

        self.running = True
        self.last_heartbeat_time = 0
        self.message_count = 0
        self.heartbeat_count = 0

        # Statistics
        self.stats = {
            'heartbeats_sent': 0,
            'messages_received': 0,
            'system_status_count': 0,
            'voltage_count': 0,
            'encoder_count': 0,
            'decode_errors': 0,
            'decode_errors_by_type': {},  # Track errors by error code type
        }

    def connect(self) -> bool:
        """Open serial connection to Arduino"""
        try:
            print(f"[UART] Opening {self.port} @ {self.baudrate} baud...")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1  # Non-blocking with short timeout
            )
            print(f"[UART] Connected successfully")
            return True
        except serial.SerialException as e:
            print(f"[UART] ERROR: Failed to open {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[UART] Disconnected")

    def send_heartbeat(self):
        """Send SYS_HEARTBEAT to Arduino"""
        payload = PayloadHeartbeat()
        payload.timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        payload.flags = 0  # No emergency stop

        self.encoder.reset()
        self.encoder.addPacket(SYS_HEARTBEAT, ctypes.sizeof(payload), payload)
        length, buffer = self.encoder.wrapupBuffer()

        self.ser.write(buffer[:length])
        self.stats['heartbeats_sent'] += 1
        self.heartbeat_count += 1

        # print(f"[TX] Heartbeat #{self.heartbeat_count} sent ({length} bytes)")

    def send_dc_enable(self, motor_id: int, enable: bool, mode: int = 2):
        """Send DC_ENABLE command"""
        payload = PayloadDCEnable()
        payload.motorId = motor_id
        payload.enable = 1 if enable else 0
        payload.mode = mode  # 0=disabled, 1=position, 2=velocity
        payload.reserved = 0

        self.encoder.reset()
        self.encoder.addPacket(DC_ENABLE, ctypes.sizeof(payload), payload)
        length, buffer = self.encoder.wrapupBuffer()

        self.ser.write(buffer[:length])
        print(f"[TX] DC_ENABLE: Motor {motor_id}, Enable={enable}, Mode={mode} ({length} bytes)")

    def send_dc_velocity(self, motor_id: int, velocity: int):
        """Send DC_SET_VELOCITY command"""
        payload = PayloadDCSetVelocity()
        payload.motorId = motor_id
        payload.targetVel = velocity
        payload.maxAccel = 0  # No acceleration limit

        self.encoder.reset()
        self.encoder.addPacket(DC_SET_VELOCITY, ctypes.sizeof(payload), payload)
        length, buffer = self.encoder.wrapupBuffer()

        self.ser.write(buffer[:length])
        print(f"[TX] DC_SET_VELOCITY: Motor {motor_id}, Velocity={velocity} ({length} bytes)")

    def send_neopixel(self, index: int, r: int, g: int, b: int):
        """Send IO_SET_NEOPIXEL command"""
        payload = PayloadSetNeoPixel()
        payload.index = index
        payload.red = r
        payload.green = g
        payload.blue = b

        self.encoder.reset()
        self.encoder.addPacket(IO_SET_NEOPIXEL, ctypes.sizeof(payload), payload)
        length, buffer = self.encoder.wrapupBuffer()

        self.ser.write(buffer[:length])
        print(f"[TX] IO_SET_NEOPIXEL: Index={index}, RGB=({r},{g},{b}) ({length} bytes)")

    def decode_callback(self, error_code, frame_header, tlv_list):
        """Callback for decoded TLV messages from Arduino"""
        if error_code != DecodeErrorCode.NoError:
            print(f"[RX] Decode error: {error_code}")
            self.stats['decode_errors'] += 1
            # Track errors by type
            error_name = error_code.name if hasattr(error_code, 'name') else str(error_code)
            if error_name not in self.stats['decode_errors_by_type']:
                self.stats['decode_errors_by_type'][error_name] = 0
            self.stats['decode_errors_by_type'][error_name] += 1
            return

        self.stats['messages_received'] += 1

        print(f"[RX] Frame from device 0x{frame_header.deviceId:02X}, "
              f"frame #{frame_header.frameNum}, {frame_header.numTlvs} TLVs"
              f"; total decode error counts: {self.stats['decode_errors']}")

        for tlv_type, tlv_len, tlv_data in tlv_list:
            self.handle_tlv(tlv_type, tlv_len, tlv_data)

    def handle_tlv(self, tlv_type: int, tlv_len: int, tlv_data: bytes):
        """Handle individual TLV message"""

        if tlv_type == SYS_STATUS:
            self.stats['system_status_count'] += 1
            if tlv_len == ctypes.sizeof(PayloadSystemStatus):
                status = PayloadSystemStatus.from_buffer_copy(tlv_data)
                print(f"  [SYS_STATUS]")
                print(f"    Uptime: {status.uptimeMs / 1000:.1f}s")
                print(f"    Last heartbeat: {status.lastHeartbeatMs}ms ago")
                print(f"    Battery: {status.batteryMv}mV")
                print(f"    5V rail: {status.rail5vMv}mV")
                print(f"    Servo rail: {status.servoRailMv}mV")
                print(f"    Error flags: 0x{status.errorFlags:02X}")
                print(f"    Motor enable mask: 0x{status.motorEnableMask:02X}")
            else:
                print(f"  [SYS_STATUS] Size mismatch: expected {ctypes.sizeof(PayloadSystemStatus)}, got {tlv_len}")

        elif tlv_type == SENSOR_VOLTAGE:
            self.stats['voltage_count'] += 1
            if tlv_len == ctypes.sizeof(PayloadSensorVoltage):
                voltage = PayloadSensorVoltage.from_buffer_copy(tlv_data)
                print(f"  [SENSOR_VOLTAGE]")
                print(f"    Battery: {voltage.batteryMv}mV")
                print(f"    5V rail: {voltage.rail5vMv}mV")
                print(f"    Servo rail: {voltage.servoRailMv}mV")
            else:
                print(f"  [SENSOR_VOLTAGE] Size mismatch")

        elif tlv_type == SENSOR_ENCODER:
            self.stats['encoder_count'] += 1
            if tlv_len == ctypes.sizeof(PayloadSensorEncoder):
                encoder = PayloadSensorEncoder.from_buffer_copy(tlv_data)
                print(f"  [SENSOR_ENCODER]")
                for i in range(4):
                    print(f"    Motor {i}: pos={encoder.position[i]}, vel={encoder.velocity[i]}")
                print(f"    Timestamp: {encoder.timestamp}µs")
            else:
                print(f"  [SENSOR_ENCODER] Size mismatch")

        else:
            print(f"  [TLV Type {tlv_type}] Length={tlv_len} bytes (unknown type)")

    def process_incoming(self):
        """Read and decode incoming serial data"""
        if self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            self.decoder.decode(data)

    def print_stats(self):
        """Print communication statistics"""
        print("\n" + "="*60)
        print("STATISTICS:")
        print(f"  Heartbeats sent: {self.stats['heartbeats_sent']}")
        print(f"  Messages received: {self.stats['messages_received']}")
        print(f"  System status: {self.stats['system_status_count']}")
        print(f"  Voltage data: {self.stats['voltage_count']}")
        print(f"  Encoder data: {self.stats['encoder_count']}")
        print(f"  Total decode errors: {self.stats['decode_errors']}")
        if self.stats['decode_errors_by_type']:
            print("  Decode errors by type:")
            for error_type, count in sorted(self.stats['decode_errors_by_type'].items()):
                print(f"    {error_type}: {count}")
        print("="*60 + "\n")

    def run(self):
        """Main test loop"""
        if not self.connect():
            return

        print("\n" + "="*60)
        print("UART TLV Communication Test")
        print("="*60)
        print("Commands:")
        print("  h - Send heartbeat")
        print("  s - Print statistics")
        print("  e - Enable DC motor 0")
        print("  d - Disable DC motor 0")
        print("  v - Set DC motor 0 velocity to 1000")
        print("  l - Set NeoPixel to green")
        print("  q - Quit")
        print("="*60 + "\n")

        try:
            while self.running:
                # Send periodic heartbeat
                current_time = time.time()
                if current_time - self.last_heartbeat_time >= HEARTBEAT_INTERVAL:
                    self.send_heartbeat()
                    self.last_heartbeat_time = current_time

                # Process incoming messages
                self.process_incoming()

                # Check for user commands (non-blocking)
                import select
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    cmd = sys.stdin.read(1)

                    if cmd == 'h':
                        self.send_heartbeat()
                    elif cmd == 's':
                        self.print_stats()
                    elif cmd == 'e':
                        self.send_dc_enable(0, True, mode=2)
                    elif cmd == 'd':
                        self.send_dc_enable(0, False, mode=0)
                    elif cmd == 'v':
                        self.send_dc_velocity(0, 1000)
                    elif cmd == 'l':
                        self.send_neopixel(0, 0, 255, 0)  # Green
                    elif cmd == 'q':
                        print("\n[Test] Exiting...")
                        self.running = False
                    else:
                        print(f"Unknown command: {cmd}")

                time.sleep(0.01)  # 100Hz loop

        except KeyboardInterrupt:
            print("\n[Test] Interrupted by user")

        finally:
            self.print_stats()
            self.disconnect()

# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='UART TLV Communication Test for Arduino')
    parser.add_argument('--port', type=str, default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')

    args = parser.parse_args()

    test = UARTTest(port=args.port, baudrate=args.baud)
    test.run()

if __name__ == '__main__':
    main()
