# MotorController Example – Advanced DC Motor Positioning Control

This example demonstrates how to use the `MotorController` class to perform precise closed-loop positioning of a DC motor with an ESP32. It showcases PID control, motion profiling, encoder feedback, event handling, and various advanced features for professional motor control applications.

---

## What It Does

- Initializes a complete motor control system with encoder feedback and DRV8876 motor driver
- Demonstrates basic point-to-point positioning with PID control
- Tests both trapezoid and S-curve motion profiling for smooth acceleration
- Shows soft limit enforcement to prevent mechanical damage
- Compares different PID tuning parameters and their effects
- Demonstrates feed-forward control for improved performance
- Provides real-time status monitoring and event callbacks
- Runs continuous back-and-forth motion for ongoing observation
- Handles motor stalls, limit violations, and motion completion events

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
| Channel A   | GPIO1        | Encoder phase A signal         |
| Channel B   | GPIO2        | Encoder phase B signal         |

These can be changed in the configuration:

```cpp
constexpr gpio_num_t PH_PIN = GPIO_NUM_4;
constexpr gpio_num_t EN_PIN = GPIO_NUM_5;
constexpr gpio_num_t FAULT_PIN = GPIO_NUM_6;
constexpr gpio_num_t SLEEP_PIN = GPIO_NUM_7;
constexpr gpio_num_t ENCODER_A = GPIO_NUM_1;
constexpr gpio_num_t ENCODER_B = GPIO_NUM_2;
```

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
constexpr MotorControllerConfig motorCfg = {
    .minSpeed = 3.0f,                               // Minimum motor speed (%)
    .maxSpeed = 95.0f,                              // Maximum motor speed (%)
    .minErrorToMove = 0.3f,                         // Minimum error to trigger motion (deg)
    .driftThreshold = 1.0f,                         // Position drift threshold (deg)
    .driftDeadband = 0.8f,                          // Steady-state drift tolerance (deg)
    .driftHysteresis = 0.4f,                        // Drift wake-up hysteresis (deg)
    .stuckPositionEpsilon = 0.1f,                   // Stuck detection sensitivity (deg)
    .stuckCountLimit = 30,                          // Cycles before declaring stuck
    .pidWarmupLimit = 15,                           // PID stabilization cycles
    .countsPerRevolution = 1024,                    // Encoder resolution
    .motionTimeoutMs = 5000,                        // Motion timeout (ms)
    .motionProfileEnabled = true,                   // Enable motion profiling
    .motionProfileType = MotionProfileType::S_CURVE, // Profile type
    .accelLimitPctPerSec = 200.0f,                  // Acceleration limit (%/s)
    .jerkLimitPctPerSec2 = 4000.0f,                 // Jerk limit (%/s²)
    .Kff_pos = 0.02f,                               // Position feed-forward gain
    .Kff_vel = 0.15f,                               // Velocity feed-forward gain
    .settlePosTolDeg = 0.5f,                        // Settle position tolerance (deg)
    .settleVelTolDegPerSec = 2.0f,                  // Settle velocity tolerance (deg/s)
    .settleCountLimit = 8                           // Consecutive settle cycles required
};
```

### Encoder Configuration

```cpp
constexpr EncoderConfig encCfg{
    .pinA = ENCODER_A,
    .pinB = ENCODER_B,
    .unitConfig = unitCfg,
    .pulsesPerRevolution = 1024,        // Encoder PPR
    .filterThresholdNs = 1000,          // Input filter threshold
    .rpmCalcPeriodUs = 100000,          // RPM calculation period
    .maxRpm = 5000,                     // Maximum expected RPM
    .enableWatchPoint = true,           // Enable position watchpoints
    .watchLowLimit = 0,                 // Low limit watchpoint
    .watchHighLimit = 0,                // High limit watchpoint
    .openCollectorInputs = false,       // Input type
    .rpmBlendThreshold = 10,            // RPM blending threshold
    .rpmBlendBand = 3,                  // RPM blending band
    .speedFilter = speedFilterCfg,      // Speed filtering config
    .direction = directionCfg           // Direction detection config
};
```

---

## Demo Sequence

The example runs through several demonstration phases:

### 1. Basic Positioning Demo
- Moves to preset positions: 0°, 90°, -45°, 180°, -90°, 0°
- Demonstrates fundamental PID control and positioning accuracy
- Shows motion completion callbacks in action

### 2. Motion Profiling Demo  
- Tests **Trapezoid Profile**: Linear acceleration with velocity limiting
- Tests **S-Curve Profile**: Smooth jerk-limited acceleration for reduced vibration
- Large moves (270°) to clearly show profiling effects
- Configurable acceleration and jerk limits

### 3. Soft Limits Demo
- Sets conservative limits (-120° to 120°)
- Attempts to exceed limits (150°, -150°)
- Demonstrates automatic position clamping and limit hit callbacks
- Resets to normal operation range

### 4. PID Tuning Demo
- **Conservative gains**: Slower, more stable response
- **Aggressive gains**: Faster, potentially oscillatory response  
- **Balanced gains**: Optimized compromise
- Shows the effect of different tuning approaches on step response

### 5. Feed-Forward Demo
- Comparison without feed-forward (pure PID)
- Demonstration with position and velocity feed-forward
- Shows improved tracking performance and reduced steady-state error

### 6. Continuous Operation
- Ongoing back-and-forth motion between ±45°
- Continuous status monitoring and logging
- Real-time performance observation

---

## Status Monitoring

The example provides comprehensive real-time status information:

```
Status - Pos:  45.23 deg | Target:  45.00 deg | Error: -0.23 deg | Vel:   12.4 deg/s | PID:  23.5% | Done: NO
```

### Status Fields
- **Pos**: Current measured position in degrees
- **Target**: Commanded target position in degrees  
- **Error**: Position error (target - actual) in degrees
- **Vel**: Estimated angular velocity in degrees per second
- **PID**: Raw PID controller output in percent
- **Done**: Motion completion status

---

## Event Callbacks

The system provides three types of event callbacks:

### Motion Done Callback
```cpp
void onMotionDoneCallback(const MotorStatus& status, void* user) {
    ESP_LOGI(TAG, "Motion completed - Position: %.2f deg, Error: %.3f deg", 
             status.position, status.error);
}
```

### Stall Detection Callback
```cpp
void onStallCallback(const MotorStatus& status, void* user) {
    ESP_LOGW(TAG, "Motor stall detected at position %.2f deg, stuck count: %d", 
             status.position, status.stuckCount);
}
```

### Limit Hit Callback
```cpp
void onLimitHitCallback(const MotorStatus& status, void* user) {
    ESP_LOGW(TAG, "Soft limit hit - Position: %.2f deg, Target: %.2f deg", 
             status.position, status.target);
}
```

---

## Task Architecture

The example uses a multi-task architecture for optimal performance:

### Control Task
- **Core**: 1 (dedicated)
- **Priority**: 15 (high priority)
- **Frequency**: 100 Hz
- **Function**: Runs the closed-loop control algorithm

### Status Monitor Task  
- **Core**: 0
- **Priority**: 5 (lower priority)
- **Frequency**: 2 Hz (500ms intervals)
- **Function**: Logs real-time status during motion

### Demo Sequence Task
- **Core**: 0  
- **Priority**: 10 (medium priority)
- **Function**: Orchestrates the demonstration sequence

---

## Key Features Demonstrated

### Motion Control
- Precise positioning with configurable tolerances
- Velocity and acceleration limiting
- Jerk limiting for smooth motion (S-curve profiles)
- Automatic motion completion detection

### Safety Features
- Soft limit enforcement with configurable boundaries
- Motor stall detection and handling
- Motion timeouts to prevent runaway conditions
- Fault monitoring from motor driver

### Advanced Control
- PID control with anti-windup
- Feed-forward compensation for improved tracking
- Motion profiling for industrial-grade smoothness
- Real-time parameter adjustment

### System Integration
- Multi-core task distribution for optimal performance
- Event-driven architecture with callbacks
- Comprehensive status monitoring and logging
- Thread-safe parameter updates

---

## Expected Output

The serial monitor will show:
1. Initialization sequence and hardware setup
2. Each demo phase with descriptive headers
3. Real-time status updates during motion
4. Event notifications (motion done, stalls, limit hits)
5. Continuous operation with periodic status reports

This example serves as both a comprehensive demonstration of the MotorController capabilities and a foundation for developing custom motor control applications.