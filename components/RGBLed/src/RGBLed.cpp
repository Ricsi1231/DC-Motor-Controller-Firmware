#include "esp_log.h"
#include "RGBLed.hpp"

namespace DC_Motor_Controller_Firmware {
namespace RGB {
RGBLed::RGBLed(ledIoConfig config)
    : config(config) {
}

esp_err_t RGBLed::init() {
  esp_err_t errorEsp = ESP_OK;

  ledc_timer_config_t timer = {};
  ledc_channel_config_t channelConf = {};

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
    ESP_LOGW(TAG, "Failed to init red RGB led channel");
    return ESP_FAIL;
  }

  channelConf.gpio_num = config.pinGreen;
  channelConf.channel = config.greenPwmChannel;
  errorEsp = ledc_channel_config(&channelConf);
  
  if (errorEsp != ESP_OK) {
    ESP_LOGW(TAG, "Failed to init red RGB green channel");
    return ESP_FAIL;
  }

  channelConf.gpio_num = config.pinBlue;
  channelConf.channel = config.bluePwmChannel;
  errorEsp = ledc_channel_config(&channelConf);
  
  if (errorEsp != ESP_OK) {
    ESP_LOGW(TAG, "Failed to init blue RGB green channel");
    return ESP_FAIL;
  }

  initalized = true;

  return ESP_OK;
}

} // namespace RGB
} // namespace DC_Motor_Controller_Firmware
