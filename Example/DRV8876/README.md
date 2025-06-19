# DRV8876 Example – Basic Motor Control

This example demonstrates how to use the `DRV8876` class to control a brushed DC motor with an ESP32-S3 using PWM, direction, and fault monitoring pins.

---

## What It Does

- Initializes the DRV8876 motor driver
- Spins the motor **RIGHT** at 60% speed for 3 seconds
- Stops the motor
- Spins the motor **LEFT** at 80% speed for 3 seconds
- Stops the motor again
- Checks for any fault condition (e.g., overcurrent or thermal shutdown)

---

## Pin Configuration

| Signal       | GPIO Pin     |
|--------------|--------------|
| PH (Direction) | GPIO4        |
| EN (PWM)       | GPIO5        |
| nFAULT         | GPIO6        |
| PWM Channel    | LEDC_CHANNEL_0 |

You can modify these values inside the example:
```cpp
gpio_num_t PH_PIN = GPIO_NUM_4;
gpio_num_t EN_PIN = GPIO_NUM_5;
gpio_num_t FAULT_PIN = GPIO_NUM_6;
ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;
