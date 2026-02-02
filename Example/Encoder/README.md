# Encoder Example -- PCNT Quadrature Signal Simulation

This example demonstrates how to simulate and test a quadrature encoder using GPIOs and the `Encoder` class on ESP32-S3. It emulates rotation signals and reads position, RPM, and direction using the PCNT peripheral.

---

## What It Does

- Simulates quadrature encoder signals using GPIOs
- Initializes the `Encoder` class with a full `EncoderConfig` struct
- Logs the following in real-time:
  - Tick count
  - Angular position (degrees)
  - RPM (revolutions per minute)
  - Motor rotation direction
- Resets the encoder position every 10 simulation cycles

---

## GPIO Mapping

| Purpose              | GPIO                                    |
|----------------------|-----------------------------------------|
| Encoder Channel A    | GPIO1 (input), GPIO15 (simulation output) |
| Encoder Channel B    | GPIO2 (input), GPIO16 (simulation output) |

---

## Configuration

```cpp
constexpr pcnt_unit_config_t pcntUnitConfig = {
    .low_limit = -1000, .high_limit = 1000,
    .intr_priority = 0, .flags = {.accum_count = true}
};

constexpr SpeedFilterConfig speedFilterConfig = {
    .filterType = SpeedFilterType::EMA,
    .emaAlpha = 0.3f, .iirCutoffHz = 2.0f, .sampleRateHz = 100
};

constexpr DirectionConfig directionConfig = {
    .hysteresisThreshold = 8, .debounceTimeMs = 100, .enableHysteresis = true
};

constexpr EncoderConfig encoderConfig = {
    .pinA = GPIO_NUM_1,
    .pinB = GPIO_NUM_2,
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

Encoder encoder(encoderConfig);
encoder.init();
encoder.start();
```

---

## Features

- **Speed Filtering**: EMA or IIR low-pass filtering for RPM smoothing
- **Direction Hysteresis**: Debounced direction detection to prevent jitter at low speeds
- **PCNT Accumulation**: Hardware pulse counting with configurable limits and watchpoints
- **Position Reset**: Encoder position can be zeroed at any time

---

## Dependencies

- `Encoder` component
- ESP-IDF PCNT driver
