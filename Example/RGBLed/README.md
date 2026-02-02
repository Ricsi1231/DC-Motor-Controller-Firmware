# RGBLed Example -- Basic RGB LED Control

This example demonstrates how to use the `RGBLed` class to control a 3-channel RGB LED with an ESP32 using PWM (via LEDC) and smooth effects such as blinking, color transitions, and brightness adjustment.

---

## What It Does

- Initializes the RGB LED with configurable pins and PWM channels
- Sets LED brightness to 60%
- Cycles through **preset colors** (red, green, blue, yellow, cyan, magenta, white)
- Blinks the LED twice for each color
- Fades to a soft white between colors
- Turns off the LED at the end of the cycle
- Loops the above sequence indefinitely

---

## Pin Configuration

| LED Color | GPIO Pin | PWM Channel    |
|-----------|----------|----------------|
| Red       | GPIO12   | LEDC_CHANNEL_0 |
| Green     | GPIO13   | LEDC_CHANNEL_1 |
| Blue      | GPIO14   | LEDC_CHANNEL_2 |

---

## Configuration

```cpp
ledIoConfig ledConfig = {
    .pinRed = GPIO_NUM_12,
    .pinGreen = GPIO_NUM_13,
    .pinBlue = GPIO_NUM_14,
    .redPwmChannel = LEDC_CHANNEL_0,
    .greenPwmChannel = LEDC_CHANNEL_1,
    .bluePwmChannel = LEDC_CHANNEL_2,
};

RGBLed led(ledConfig);
led.init();
led.setBrightness(60);
```

---

## Dependencies

- `RGBLed` component
- ESP-IDF LEDC driver
