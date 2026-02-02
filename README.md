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
- **Motion Profiling** with trapezoid and S-curve acceleration shaping
- **Interface Abstractions** for testable, decoupled component design
- **RGB LED** status indicator support

---

## Repository Structure

```
ProjectRoot/
├── CMakeLists.txt
├── LICENSE
├── VERSION.txt
├── sdkconfig
├── sdkconfig.defaults
├── docs/
├── managed_components/
├── components/
│   ├── DRV8876/              # PWM motor driver (owns GPIO, PWM, fault, ramp)
│   ├── Encoder/              # Quadrature encoder (PCNT, speed filter, direction)
│   ├── Interfaces/           # Abstract base classes (IMotorController, IEncoder, etc.)
│   ├── MotorCommHandler/     # USB command protocol handler (owns USB internally)
│   ├── PID/                  # PID controller with anti-windup
│   ├── RGBLed/               # RGB LED control via LEDC PWM
│   ├── USB/                  # TinyUSB CDC interface
│   └── motorControl/         # High-level motor controller (owns motor, encoder, PID)
│       └── src/              # Includes MotionProfiler, StallDetector, SettleDetector,
│                             # SoftLimiter, MotionGuard sub-components
├── main/
│   ├── main.cpp              # Application entry point
│   └── Constants/
│       └── PeripheralSettings.hpp  # Hardware configuration constants
└── Example/
    ├── Communication/        # USB command protocol example
    ├── DRV8876/              # Motor driver example
    ├── Encoder/              # Encoder reading example
    ├── MotorControl/         # Full closed-loop motor control demo
    ├── RGBLed/               # RGB LED effects example
    └── USB/                  # USB loopback example
```

---
## Architecture

Components use an **ownership-based** design:

- `MotorController` takes config structs and internally creates its `DRV8876`, `Encoder`, and `PIDController` instances
- `MotorCommHandler` internally creates its `USB` communication interface
- All components expose their internals via getter methods (`getEncoder()`, `getMotor()`, `getPid()`, `getComm()`)
- Abstract interfaces (`IMotorController`, `IEncoder`, `IMotorDriver`, `IComm`, `IPIDController`) enable decoupled design

---
## Getting Started

### Requirements

- ESP32-S3 module (USB CDC capable)
- ESP-IDF v5.0 or later
- DRV8876 H-Bridge driver
- Incremental quadrature encoder
- CMake build system
- Python 3 with `idf.py`

### Build

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## License

This project is provided for educational and prototyping purposes.
All content is (c) 2026 Nagy Richard. Licensing for commercial use may require additional agreements.

---

## Notes

- Designed to integrate with LabVIEW SCADA systems
- Fully modular and testable structure
- Suitable for robotic arms, actuators, and educational projects
