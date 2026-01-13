#include "RGBLed.hpp"
#include "esp_log.h"
#include <algorithm>

namespace DC_Motor_Controller_Firmware {
namespace RGB {
RGBLed::RGBLed(ledIoConfig config) : config(config) {}

RGBLed::~RGBLed() {}

esp_err_t RGBLed::init() {
    esp_err_t errorEsp = ESP_OK;

    ledc_timer_config_t timer = {};
    ledc_channel_config_t channelConf = {};

    if (config.redPwmChannel >= LEDC_CHANNEL_MAX || config.greenPwmChannel >= LEDC_CHANNEL_MAX || config.bluePwmChannel >= LEDC_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Invalid LEDC channel index");
        return ESP_ERR_INVALID_ARG;
    }

    if (config.pinRed == GPIO_NUM_NC || config.pinGreen == GPIO_NUM_NC || config.pinBlue == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "Invalid GPIO pin in config");
        return ESP_ERR_INVALID_ARG;
    }

    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = 20000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    errorEsp = ledc_timer_config(&timer);
    if (errorEsp != ESP_OK) {
        return ESP_FAIL;
    }

    channelConf.gpio_num = config.pinRed;
    channelConf.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConf.channel = config.redPwmChannel;
    channelConf.intr_type = LEDC_INTR_DISABLE;
    channelConf.timer_sel = LEDC_TIMER_0;
    channelConf.duty = 0;
    channelConf.hpoint = 0;
    channelConf.flags.output_invert = false;
    errorEsp = ledc_channel_config(&channelConf);

    if (errorEsp != ESP_OK) {
        ESP_LOGW(TAG, "Failed to init RED RGB channel");
        return ESP_FAIL;
    }

    ledc_channel_config_t greenChannel = channelConf;
    greenChannel.gpio_num = config.pinGreen;
    greenChannel.channel = config.greenPwmChannel;
    errorEsp = ledc_channel_config(&greenChannel);

    if (errorEsp != ESP_OK) {
        ESP_LOGW(TAG, "Failed to init GREEN RGB channel");
        return ESP_FAIL;
    }

    ledc_channel_config_t blueChannel = channelConf;
    blueChannel.gpio_num = config.pinBlue;
    blueChannel.channel = config.bluePwmChannel;
    errorEsp = ledc_channel_config(&blueChannel);

    if (errorEsp != ESP_OK) {
        ESP_LOGW(TAG, "Failed to init BLUE RGB channel");
        return ESP_FAIL;
    }

    initialized = true;

    return ESP_OK;
}

void RGBLed::setColor(PresetColor color) {
    if (initialized == false) {
        ESP_LOGW(TAG, "RGB is not initialized");
        return;
    }

    switch (color) {
        case PresetColor::RED:
            setRGBColor(255, 0, 0);
            break;
        case PresetColor::GREEN:
            setRGBColor(0, 255, 0);
            break;
        case PresetColor::BLUE:
            setRGBColor(0, 0, 255);
            break;
        case PresetColor::YELLOW:
            setRGBColor(255, 255, 0);
            break;
        case PresetColor::CYAN:
            setRGBColor(0, 255, 255);
            break;
        case PresetColor::MAGENTA:
            setRGBColor(255, 0, 255);
            break;
        case PresetColor::WHITE:
            setRGBColor(255, 255, 255);
            break;
        default:
            turnOffLed();
            break;
    }
}

void RGBLed::fadeToColor(uint8_t targetRed, uint8_t targetGreen, uint8_t targetBlue, uint16_t durationMs) {
    if (initialized == false) {
        ESP_LOGW(TAG, "RGB is not initialized");
        return;
    }

    uint8_t percentRed = std::min(targetRed, static_cast<uint8_t>(100));
    uint8_t percentGreen = std::min(targetGreen, static_cast<uint8_t>(100));
    uint8_t percentBlue = std::min(targetBlue, static_cast<uint8_t>(100));

    uint8_t scaledRed = static_cast<uint8_t>((percentRed * 255) / 100);
    uint8_t scaledGreen = static_cast<uint8_t>((percentGreen * 255) / 100);
    uint8_t scaledBlue = static_cast<uint8_t>((percentBlue * 255) / 100);

    fadeRGB(scaledRed, scaledGreen, scaledBlue, durationMs);
}

void RGBLed::blinkLed(uint8_t blinkDelay, uint8_t blinkTimes) {
    if (initialized == false) {
        ESP_LOGW(TAG, "RGB is not initialized");
        return;
    }

    const uint8_t savedRed = currentRed;
    const uint8_t savedGreen = currentGreen;
    const uint8_t savedBlue = currentBlue;

    for (uint8_t i = 0; i < blinkTimes; ++i) {
        setRGBColor(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(blinkDelay));

        setRGBColor(savedRed, savedGreen, savedBlue);
        vTaskDelay(pdMS_TO_TICKS(blinkDelay));
    }
}

void RGBLed::setBrightness(uint8_t percent) {
    if (initialized == false) {
        ESP_LOGW(TAG, "RGB is not initialized");
        return;
    }

    uint8_t clampedPercent = std::min(percent, static_cast<uint8_t>(100));
    brightness = static_cast<float>(clampedPercent) / 100.0f;
    setRGBColor(currentRed, currentGreen, currentBlue);
}

void RGBLed::turnOffLed() {
    if (initialized == false) {
        ESP_LOGW(TAG, "RGB is not initialized");
        return;
    }

    setRGBColor(0, 0, 0);
    ledStatus = 0;
}

void RGBLed::turnOnLed() {
    if (initialized == false) {
        ESP_LOGW(TAG, "RGB is not initialized");
        return;
    }

    setRGBColor(currentRed, currentGreen, currentBlue);
    ledStatus = 1;
}

bool RGBLed::isOn() const { return ledStatus; }

void RGBLed::fadeRGB(uint8_t targetRed, uint8_t targetGreen, uint8_t targetBlue, uint16_t durationMs) {
    const int delayPerStep = durationMs / std::max<uint8_t>(1, fadeSteps);

    uint8_t startRed = currentRed;
    uint8_t startGreen = currentGreen;
    uint8_t startBlue = currentBlue;

    int diffR = static_cast<int>(targetRed) - static_cast<int>(startRed);
    int diffG = static_cast<int>(targetGreen) - static_cast<int>(startGreen);
    int diffB = static_cast<int>(targetBlue) - static_cast<int>(startBlue);

    for (int i = 1; i <= fadeSteps; ++i) {
        uint8_t r = startRed + diffR * i / fadeSteps;
        uint8_t g = startGreen + diffG * i / fadeSteps;
        uint8_t b = startBlue + diffB * i / fadeSteps;

        setRGBColor(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(delayPerStep));
    }

    currentRed = targetRed;
    currentGreen = targetGreen;
    currentBlue = targetBlue;
}

void RGBLed::setRGBColor(uint8_t red, uint8_t green, uint8_t blue) {
    currentRed = red;
    currentGreen = green;
    currentBlue = blue;

    uint32_t scaledRed = static_cast<uint32_t>(red * brightness);
    uint32_t scaledGreen = static_cast<uint32_t>(green * brightness);
    uint32_t scaledBlue = static_cast<uint32_t>(blue * brightness);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.redPwmChannel, scaledRed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.redPwmChannel);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.greenPwmChannel, scaledGreen);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.greenPwmChannel);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.bluePwmChannel, scaledBlue);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.bluePwmChannel);

    ledStatus = (scaledRed || scaledGreen || scaledBlue) ? 1 : 0;
}

}  // namespace RGB
}  // namespace DC_Motor_Controller_Firmware
