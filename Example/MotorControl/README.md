# MotorController Example -- Advanced DC Motor Positioning Control

This example demonstrates how to use the `MotorController` class to perform precise closed-loop positioning of a DC motor with an ESP32. It showcases PID control, motion profiling, encoder feedback, event handling, and various advanced features.

The `MotorController` owns its motor driver, encoder, and PID controller internally -- you only pass configuration structs.

---

## What It Does

- Initializes a complete motor control system from config structs
- Demonstrates basic point-to-point positioning with PID control
- Tests both trapezoid and S-curve motion profiling for smooth acceleration
- Shows soft limit enforcement to prevent mechanical damage
- Compares different PID tuning parameters and their effects
- Demonstrates feed-forward control for improved performance
- Provides real-time status monitoring and event callbacks
- Runs continuous back-and-forth motion for ongoing observation

---

## Hardware Configuration

### Motor Driver (DRV8876)

| Function    | GPIO Pin     | Description                    |
|-------------|--------------|--------------------------------|
| Phase (PH)  | GPIO4        | Motor direction control        |
| Enable (EN) | GPIO5        | PWM speed control (20kHz)      |
| Fault       | GPIO6        | Fault status input             |
| Sleep       | GPIO7        | Sleep/wake control             |

### Quadrature Encoder

| Signal      | GPIO Pin     | Description                    |
|-------------|--------------|--------------------------------|
| Channel A   | GPIO2        | Encoder phase A signal         |
| Channel B   | GPIO1        | Encoder phase B signal         |

---

## Configuration Parameters

### PID Controller Settings

```cpp
constexpr PidConfig pidConfig = {
    .kp = 2.5f,                  // Proportional gain
    .ki = 0.08f,                 // Integral gain
    .kd = 0.15f,                 // Derivative gain
    .maxOutput = 100.0f,         // Maximum output (%)
    .maxIntegral = 500.0f,       // Integral windup limit
    .errorEpsilon = 0.1f,        // Position error tolerance (deg)
    .speedEpsilon = 0.2f,        // Velocity tolerance (deg/s)
    .errorTimeoutSec = 2.0f,     // Error timeout
    .stuckTimeoutSec = 1.0f      // Stall detection timeout
};
```

### Motor Controller Settings

```cpp
constexpr MotorControllerConfig motorControllerConfig = {
    .minSpeed = 3.0f,                        // Minimum motor speed (%)
    .maxSpeed = 95.0f,                       // Maximum motor speed (%)
    .minErrorToMove = 0.3f,                  // Minimum error to trigger motion (deg)
    .countsPerRevolution = 1024,             // Encoder resolution
    .Kff_pos = 0.02f,                        // Position feed-forward gain
    .Kff_vel = 0.15f,                        // Velocity feed-forward gain
    .profiler = {
        .enabled = true,
        .type = MotionProfileType::S_CURVE,
        .accelLimitPctPerSec = 200.0f,       // Acceleration limit (%/s)
        .jerkLimitPctPerSec2 = 4000.0f,      // Jerk limit (%/s^2)
        .maxSpeed = 95.0f
    },
    .settle = {
        .posTolDeg = 0.5f,                   // Settle position tolerance (deg)
        .velTolDegPerSec = 2.0f,             // Settle velocity tolerance (deg/s)
        .countLimit = 8                      // Consecutive settle cycles required
    },
    .stall = {
        .stuckPositionEpsilon = 0.1f,        // Stuck detection sensitivity (deg)
        .stuckCountLimit = 30,               // Cycles before declaring stuck
        .pidWarmupLimit = 15,                // PID stabilization cycles
        .minErrorToMove = 0.3f
    },
    .guard = {
        .motionTimeoutMs = 5000,             // Motion timeout (ms)
        .driftDeadband = 0.8f,               // Steady-state drift tolerance (deg)
        .driftHysteresis = 0.4f              // Drift wake-up hysteresis (deg)
    }
};
```

### Encoder Configuration

```cpp
constexpr EncoderConfig encoderConfig = {
    .pinA = ENCODER_A,
    .pinB = ENCODER_B,
    .unitConfig = pcntUnitConfig,
    .pulsesPerRevolution = 1024,
    .filterThresholdNs = 1000,
    .rpmCalcPeriodUs = 100000,
    .maxRpm = 5000,
    .enableWatchPoint = true,
    .watchLowLimit = 0,
    .watchHighLimit = 0,
    .openCollectorInputs = false,
    .rpmBlendThreshold = 10,
    .rpmBlendBand = 3,
    .speedFilter = speedFilterConfig,
    .direction = directionConfig
};
```

---

## Initialization

The `MotorController` creates and owns all hardware objects internally:

```cpp
MotorController motorController(motorConfig, encoderConfig, pidConfig, motorControllerConfig);

esp_err_t result = motorController.init();  // Initializes motor, encoder, starts encoder

motorController.setUpdateHz(100);
motorController.setMotionTimeoutMs(10000);
motorController.configureControlTask(1, 15);
motorController.startTask();
```

---

## Demo Sequence

### 1. Basic Positioning Demo
- Moves to preset positions: 0, 90, -45, 180, -90, 0 degrees
- Demonstrates fundamental PID control and positioning accuracy

### 2. Motion Profiling Demo
- Tests **Trapezoid Profile**: Linear acceleration with velocity limiting
- Tests **S-Curve Profile**: Smooth jerk-limited acceleration for reduced vibration

### 3. Soft Limits Demo
- Sets limits (-120 to 120 degrees)
- Attempts to exceed limits, demonstrates automatic clamping and limit hit callbacks

### 4. PID Tuning Demo
- **Conservative gains**: Slower, more stable response
- **Aggressive gains**: Faster, potentially oscillatory response
- **Balanced gains**: Optimized compromise

### 5. Feed-Forward Demo
- Comparison without feed-forward (pure PID)
- Demonstration with position and velocity feed-forward

### 6. Continuous Operation
- Ongoing back-and-forth motion between +/-45 degrees

---

## Event Callbacks

```cpp
motorController.setOnMotionDone(onMotionDoneCallback, &exampleState);
motorController.setOnStall(onStallCallback, &exampleState);
motorController.setOnLimitHit(onLimitHitCallback, &exampleState);
```

---

## Task Architecture

| Task             | Core | Priority | Frequency | Function                       |
|------------------|------|----------|-----------|--------------------------------|
| Control Task     | 1    | 15       | 100 Hz    | Closed-loop control algorithm  |
| Status Monitor   | 0    | 5        | 2 Hz      | Real-time status logging       |
| Demo Sequence    | 0    | 10       | --        | Orchestrates demo phases       |

---

## Status Output

```
Status - Pos:  45.23 deg | Target:  45.00 deg | Error: -0.23 deg | Vel:   12.4 deg/s | PID:  23.5% | Done: NO
```

---

## Dependencies

- `motorControl`, `DRV8876`, `Encoder`, `PID`, `Interfaces` components
