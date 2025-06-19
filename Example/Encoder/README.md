# Encoder Example – PCNT Quadrature Signal Simulation

This example demonstrates how to simulate and test a quadrature encoder using GPIOs and the `Encoder` class on ESP32-S3. It emulates rotation signals and reads position, RPM, and direction using the PCNT peripheral and a timer.

---

##  What It Does

- Simulates quadrature encoder signals using GPIOs
- Initializes the `Encoder` class with custom PCNT config
- Logs the following in real-time:
  - Tick count
  - Angular position (degrees)
  - RPM (revolutions per minute)
  - Motor rotation direction
- Resets the encoder position every 10 simulation cycles

---

##  GPIO Mapping

| Purpose              | GPIO         |
|----------------------|--------------|
| Encoder Channel A    | GPIO1 (input), GPIO15 (simulation output) |
| Encoder Channel B    | GPIO2 (input), GPIO16 (simulation output) |

You can modify these in the code:
```cpp
#define ENCODER_A_GPIO GPIO_NUM_15
#define ENCODER_B_GPIO GPIO_NUM_16
Encoder encoder(GPIO_NUM_1, GPIO_NUM_2, pcntUnit, 1024);
