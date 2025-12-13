# Custom PCB Design

**PRIORITY**: PCB manufacturing has the longest lead time (2-4 weeks).

## Contents

This directory contains all PCB design files and manufacturing outputs.

## File Structure

```
pcb/
├── schematic/          Schematic design files
├── layout/             PCB layout files
├── libraries/          Custom component footprints and symbols
├── fabrication/        Manufacturing outputs
│   ├── gerbers/       Gerber files for PCB fab
│   ├── bom/           Bill of Materials
│   └── assembly/      Pick-and-place files
└── datasheets/         Component datasheets
```

## Key Components

- Microcontroller (Arduino-compatible)
- Motor drivers (H-bridge for DC motors)
- Stepper motor drivers
- Power management (voltage regulators, protection)
- Raspberry Pi interface connector
- Sensor headers and expansion connectors

## Design Tools

Recommended: KiCad 7.x or Eagle
