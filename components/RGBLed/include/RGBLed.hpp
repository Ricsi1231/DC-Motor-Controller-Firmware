/**
 * @file RGBLed.hpp
 * @brief RGB LED control class using PWM (LEDC) on ESP32.
 *
 * This class provides an interface to control a common-cathode RGB LED
 * using PWM signals with optional fading, brightness control, and color presets.
 */

#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace DC_Motor_Controller_Firmware {
namespace RGB {

/**
 * @enum PresetColor
 * @brief Predefined color preset.
 */
enum class PresetColor {
  RED,
  GREEN,
  BLUE,
  YELLOW,
  CYAN,
  MAGENTA,
  WHITE
};

/**
 * @struct ledIoConfig
 * @brief GPIO and PWM channel configuration for RGB LED control.
 */
struct ledIoConfig {
  gpio_num_t pinRed;           ///< GPIO pin for red channel
  gpio_num_t pinGreen;         ///< GPIO pin for green channel
  gpio_num_t pinBlue;          ///< GPIO pin for blue channel

  ledc_channel_t redPwmChannel;   ///< LEDC PWM channel for red
  ledc_channel_t greenPwmChannel; ///< LEDC PWM channel for green
  ledc_channel_t bluePwmChannel;  ///< LEDC PWM channel for blue
};

/**
 * @class RGBLed
 * @brief Controls a common-cathode RGB LED using PWM fading and color presets.
 */
class RGBLed {
public:
  /**
   * @brief Constructor.
   * @param config LED pin and PWM channel configuration.
   */
  explicit RGBLed(ledIoConfig config);

  /**
   * @brief Destructor.
   */
  ~RGBLed();

  /**
   * @brief Initializes GPIOs and PWM channels for RGB control.
   * @return ESP_OK if successful.
   */
  esp_err_t init();

  /**
   * @brief Smoothly fade to a specific RGB color using percentage input (0–100%).
   *
   * This method maps percentage values to internal 8-bit brightness and
   * performs a smooth transition over the specified duration.
   *
   * @param targetRed Target red brightness (0–100%)
   * @param targetGreen Target green brightness (0–100%)
   * @param targetBlue Target blue brightness (0–100%)
   * @param durationMs Total fade duration in milliseconds
   */
  void fadeToColor(uint8_t targetRed, uint8_t targetGreen, uint8_t targetBlue,
                   uint16_t durationMs);

  /**
   * @brief Set color using a predefined color enum.
   * @param color PresetColor enum (e.g. RED, GREEN, etc.)
   */
  void setColor(PresetColor color);

  /**
   * @brief Blink the LED using the current or last-set color.
   * @param blinkDelay Delay between on/off cycles (ms)
   * @param blinkTimes Number of blinks
   */
  void blinkLed(uint8_t blinkDelay, uint8_t blinkTimes);

  /**
   * @brief Set the global brightness of the LED (affects all channels).
   * @param percent Brightness (0–100%)
   */
  void setBrightness(uint8_t percent);

  /**
   * @brief Turn off all LED channels.
   */
  void turnOffLed();

  /**
   * @brief Restore last color and turn the LED on.
   */
  void turnOnLed();

  /**
   * @brief Returns whether the LED is currently on.
   * @return true if LED is on, false if off.
   */
  bool isOn() const;

private:
  ledIoConfig config; ///< IO + PWM config struct

  uint8_t currentRed = 0;     ///< Last applied red channel value (0–255)
  uint8_t currentGreen = 0;   ///< Last applied green channel value (0–255)
  uint8_t currentBlue = 0;    ///< Last applied blue channel value (0–255)

  uint8_t blinkDelay = 0;     ///< Delay between LED blink on/off states (in ms)
  bool ledStatus = false;     ///< True if LED is currently on, false if off
  bool initialized = false;   ///< Set to true after successful initialization

  uint8_t fadeSteps = 32; ///< Number of steps used during fading
  float brightness = 1.0f; ///< Brightness factor (0.0–1.0)

  static constexpr const char *TAG = "RGB_LED";

  /**
   * @brief Internal helper to gradually fade from current to target RGB values.
   *
   * This version uses full-resolution RGB values (0–255) and applies
   * brightness scaling automatically.
   *
   * @param targetRed Target red brightness (0–255)
   * @param targetGreen Target green brightness (0–255)
   * @param targetBlue Target blue brightness (0–255)
   * @param durationMs Duration of fade in milliseconds
   */
  void fadeRGB(uint8_t targetRed, uint8_t targetGreen, uint8_t targetBlue,
               uint16_t durationMs);

  /**
   * @brief Set RGB output immediately (applies brightness).
   * @param red Red channel (0–255)
   * @param green Green channel (0–255)
   * @param blue Blue channel (0–255)
   */
  void setRGBColor(uint8_t red, uint8_t green, uint8_t blue);
};

} // namespace RGB
} // namespace DC_Motor_Controller_Firmware
