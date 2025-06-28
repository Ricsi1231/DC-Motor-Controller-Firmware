#pragma once

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

namespace DC_Motor_Controller_Firmware {
namespace RGB {

enum class PresetColor {
    RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE
};

struct ledIoConfig {
    gpio_num_t pinRed;
    gpio_num_t pinGreen;
    gpio_num_t pinBlue;

    ledc_channel_t redPwmChannel;
    ledc_channel_t greenPwmChannel;
    ledc_channel_t bluePwmChannel;
};


class RGBLed {
public:
    explicit RGBLed(ledIoConfig config);
    ~RGBLed();

    esp_err_t init();
    
    void fadeToColor(uint8_t targetRed, uint8_t targetGreen, uint8_t targetBlue, uint16_t durationMs);
    void setColor(PresetColor color);
    void blinkLed(uint8_t blinkDelay, uint8_t blinkTimes);

    void setBrightness(uint8_t percent);

    void turnOffLed();
    void turnOnLed();
    bool isOn() const;

private:
    ledIoConfig config;

    uint8_t currentRed = 0;
    uint8_t currentGreen = 0;
    uint8_t currentBlue = 0;

    uint8_t blinkDelay = 0;

    bool ledStatus = 0;

    bool initalized = false;

    uint8_t fadeSteps = 32;

    float brightness = 1.0f;

    static constexpr const char* TAG = "RGB_LED";

    void fadeRGB(uint8_t targetRed, uint8_t targetGreen, uint8_t targetBlue, uint16_t durationMs);
    void setRGBColor(uint8_t red, uint8_t green, uint8_t blue);
};

} // namespace RGB
} // namespace DC_Motor_Controller_Firmware
