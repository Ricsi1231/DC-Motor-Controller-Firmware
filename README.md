# DC Motor Controller Firmware

Embedded firmware for closed-loop DC motor position control.  
Developed for ESP32-S3 with support for USB communication, encoder feedback, PWM motor driving, and configurable PID tuning.

---
## Overview

This project implements a modular and extensible motor control stack with:

- **PID Control** for precise position tracking  
- **DRV8876 Motor Driver** integration using PWM  
- **Quadrature Encoder** reading with ESP32 PCNT peripheral  
- **USB CDC Communication** via TinyUSB stack  
---

## Repository Structure

```
ProjectRoot/
├── CMakeLists.txt            # Project-level CMake config
├── LICENSE                   # Licensing information
├── VERSION.txt               # Firmware version tag
├── sdkconfig                 # ESP-IDF configuration
├── sdkconfig.defaults        # Default ESP-IDF settings
├── sdkconfig.old             # Backup of previous settings
├── clang-format              # Code formatting config
├── cppcheck.sh               # Static analysis script
├── format.sh                 # Code formatting helper
├── dependencies.lock         # Dependency lock file
├── docs/                     # Documentation, diagrams, etc.
├── managed_components/       # ESP-IDF managed components
├── components/               # Core firmware components
│   ├── DRV8876/              # PWM motor driver abstraction
│   ├── Encoder/              # Quadrature encoder interface
│   ├── MotorCommHandler/     # USB protocol + command handler
│   ├── PID/                  # PID controller logic
│   ├── USB/                  # TinyUSB interface
│   └── motorControl/         # High-level motor logic
├── main/                     # Entry point (app_main)
│   └── main.cpp              # Application startup
└── Example/                  # Example configurations/tests
    ├── Communication/        # Test communication module
    ├── DRV8876/              # Test for motor driver
    ├── Encoder/              # Test for encoder logic
    ├── Encoder_Motor/        # Integrated encoder + motor test
    └── USB/                  # USB communication examples
```
---
## Getting Started

### Requirements

- ESP32-S3 module (USB CDC capable)  
- ESP-IDF v5.0 or later  
- DRV8876 H-Bridge driver  
- Incremental quadrature encoder  
- CMake build system  
- Python 3 with `idf.py`

## License

This project is provided for educational and prototyping purposes.  
All content is © 2025 Nagy Richárd. Licensing for commercial use may require additional agreements.

---

## Notes

- Designed to integrate with LabVIEW SCADA systems  
- Fully modular and testable structure  
- Suitable for robotic arms, actuators, and educational projects