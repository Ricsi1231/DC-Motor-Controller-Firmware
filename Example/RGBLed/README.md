# RGBLed Example – Basic RGB LED Control

This example demonstrates how to use the `RGBLed` class to control a 3-channel RGB LED with an ESP32 using PWM (via LEDC) and smooth effects such as blinking, color transitions, and brightness adjustment.

---

## What It Does

- Initializes the RGB LED with configurable pins and PWM channels
- Sets LED brightness to 60%
- Cycles through the **preset colors** (red, green, blue, yellow, cyan, magenta, white)
- Blinks the LED twice for each color
- Fades to a soft white between colors
- Turns off the LED at the end of the cycle
- Loops the above sequence indefinitely

---

## Pin Configuration

| LED Color | GPIO Pin     | PWM Channel     |
|-----------|--------------|-----------------|
| Red       | GPIO12       | LEDC_CHANNEL_0  |
| Green     | GPIO13       | LEDC_CHANNEL_1  |
| Blue      | GPIO14       | LEDC_CHANNEL_2  |

These can be changed directly in the example:

```cpp
gpio_num_t RED_PIN = GPIO_NUM_21;
gpio_num_t GREEN_PIN = GPIO_NUM_22;
gpio_num_t BLUE_PIN = GPIO_NUM_23;

ledc_channel_t RED_CH = LEDC_CHANNEL_0;
ledc_channel_t GREEN_CH = LEDC_CHANNEL_1;
ledc_channel_t BLUE_CH = LEDC_CHANNEL_2;
