# DRV8876 Example -- Basic Motor Control

This example demonstrates how to use the `DRV8876` class to control a brushed DC motor with an ESP32-S3 using PWM, direction, and fault monitoring pins.

---

## What It Does

- Initializes the DRV8876 motor driver with full configuration
- Spins the motor **RIGHT** at 60% speed for 3 seconds
- Stops the motor
- Spins the motor **LEFT** at 80% speed for 3 seconds
- Stops the motor again
- Checks for any fault condition (e.g., overcurrent or thermal shutdown)

---

## Pin Configuration

| Signal         | GPIO Pin       |
|----------------|----------------|
| PH (Direction) | GPIO4          |
| EN (PWM)       | GPIO5          |
| nFAULT         | GPIO6          |
| nSLEEP         | GPIO7          |
| PWM Channel    | LEDC_CHANNEL_0 |

---

## Configuration

```cpp
DRV8876Config motorConfig = {
    .phPin = GPIO_NUM_4,
    .enPin = GPIO_NUM_5,
    .nFault = GPIO_NUM_6,
    .nSleep = GPIO_NUM_7,
    .pwmChannel = LEDC_CHANNEL_0,
    .resolution = LEDC_TIMER_10_BIT,
    .frequency = 20000,
    .minFrequency = 100,
    .maxFrequency = 100000,
    .rampStepPercent = 5,
    .rampStepDelayMs = 10,
    .minEffectivePwmPercent = 3,
};

DRV8876 motor(motorConfig);
motor.init();
```

---

## Dependencies

- `DRV8876` component
- ESP-IDF LEDC driver
