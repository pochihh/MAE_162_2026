# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Educational robotics platform for MAE 162 course (Winter/Spring 2026). This is a two-wheeled mobile robot with customizable manipulators designed for hands-on robotics education.

## System Architecture

### Hardware Stack
- **Arduino**: Low-level real-time control layer
  - Motor control (DC motors for wheels)
  - Stepper motor control
  - Servo motor control
  - GPIO management
  - LED indicators
  - Button inputs
  - UART communication to Raspberry Pi

- **Raspberry Pi 5**: High-level decision-making layer
  - ROS2 runtime environment
  - Camera processing
  - GPS integration
  - Additional sensor fusion
  - UART communication to Arduino

- **Custom PCB**: Integrates Arduino, motor drivers, power management, and RPi interface
  - **Critical**: PCB design/manufacturing has longest lead time - prioritize this work

### Communication Architecture
- **Arduino ↔ Raspberry Pi**: UART serial protocol
- Data flow: ROS2 nodes → Serial bridge → Arduino firmware → Hardware actuators
- Sensor data flows in reverse: Hardware sensors → Arduino → UART → ROS2 topics

## Repository Structure (Planned)

```
/firmware          - Arduino firmware (.ino files)
/ros2_ws          - ROS2 workspace
  /src            - ROS2 packages
    /robot_bringup     - Launch files and configuration
    /serial_bridge     - Arduino-ROS2 communication node
    /sensor_drivers    - Camera, GPS drivers
    /control           - High-level control algorithms
/pcb              - PCB design files (schematics, layout, Gerbers, BOM)
/mechanical       - CAD files for chassis and manipulator mounts
/docs             - Assembly instructions and course materials
```

## Development Workflow

### PCB Development (HIGHEST PRIORITY)
1. Schematic design using KiCad or Eagle
2. Component selection (verify availability and educational suitability)
3. PCB layout and routing
4. Generate manufacturing files (Gerbers, drill files, BOM, pick-and-place)
5. Order fabrication (allow 2-4 weeks lead time)

### Arduino Firmware Development
- Target: Arduino Mega or similar (needs multiple serial ports for sensors)
- Must handle real-time motor control loops
- UART protocol must be robust and well-documented for student learning

### ROS2 Development
- Target: ROS2 Humble or Jazzy on Ubuntu (Raspberry Pi OS)
- Serial bridge node is critical - handles all Arduino communication
- Design nodes to be modular for different course modules

### Mechanical Design
- Chassis must support modular top plate for interchangeable manipulators
- Design for ease of assembly by students
- Consider manufacturability (3D printing, laser cutting, or off-the-shelf parts)

## Key Constraints

- **Educational Focus**: Code and hardware must be understandable by students learning robotics
- **Reproducibility**: All designs must be manufacturable in quantity for class
- **Modularity**: Platform must support multiple course modules with different manipulators
- **Robustness**: Hardware/software must survive student handling and experimentation

## UART Protocol Design Notes

Define clear message format between Arduino and ROS2:
- Command structure (motor velocities, servo positions, digital outputs)
- Sensor feedback structure (encoder counts, button states, analog readings)
- Error handling and acknowledgments
- Consider using a standard protocol (e.g., rosserial) vs custom protocol for educational clarity