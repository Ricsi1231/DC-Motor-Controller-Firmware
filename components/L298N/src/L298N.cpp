#include "L298N.hpp"

namespace DC_Motor_Controller_Firmware {
namespace L298N {

L298N::L298N(gpio_num_t phPin, gpio_num_t enPin, ledc_channel_t pwmChannel)
    : phPin(phPin), enPin(enPin), pwmChannel(pwmChannel) {}

esp_err_t L298N::init() {
  esp_err_t errorEsp;

  gpio_config_t ioConf = {};
  ioConf.intr_type = GPIO_INTR_DISABLE;
  ioConf.mode = GPIO_MODE_OUTPUT;
  ioConf.pin_bit_mask = (1ULL << phPin);
  errorEsp = gpio_config(&ioConf);
  if (errorEsp != ESP_OK) {
    ESP_LOGW(TAG, "Failed to init PH pin");
    return errorEsp;
  }

  gpio_config_t faultConf = {};
  faultConf.intr_type = GPIO_INTR_NEGEDGE;
  faultConf.mode = GPIO_MODE_INPUT;
  faultConf.pin_bit_mask = (1ULL << nFault);
  faultConf.pull_up_en = GPIO_PULLUP_ENABLE;
  errorEsp = gpio_config(&faultConf);
  if (errorEsp != ESP_OK) {
    ESP_LOGW(TAG, "Failed to init nFAULT pin");
    return errorEsp;
  }

  ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                               .duty_resolution = resolution,
                               .timer_num = LEDC_TIMER_0,
                               .freq_hz = frequency,
                               .clk_cfg = LEDC_AUTO_CLK};
  errorEsp = ledc_timer_config(&timer);
  if (errorEsp != ESP_OK)
    return errorEsp;

  ledc_channel_config_t channelConf = {.gpio_num = enPin,
                                       .speed_mode = LEDC_LOW_SPEED_MODE,
                                       .channel = pwmChannel,
                                       .intr_type = LEDC_INTR_DISABLE,
                                       .timer_sel = LEDC_TIMER_0,
                                       .duty = 0,
                                       .hpoint = 0};
  errorEsp = ledc_channel_config(&channelConf);
  if (errorEsp != ESP_OK)
    return errorEsp;

  isInitialized = true;
  return ESP_OK;
}

void L298N::setDirection(Direction direction) {
  if (!isInitialized) {
    ESP_LOGW(TAG, "setDirection - not initialized");
    return;
  }
  motorDirection = direction;
  gpio_set_level(phPin, static_cast<bool>(motorDirection));
}

void L298N::reverseDirection() {
  if (!isInitialized)
    return;
  setDirection(motorDirection == Direction::LEFT ? Direction::RIGHT
                                                 : Direction::LEFT);
}

esp_err_t L298N::setSpeed(uint8_t speed) {
  if (!isInitialized) {
    ESP_LOGW(TAG, "setSpeed - not initialized");
    return ESP_FAIL;
  }

  if (speed < minMotorSpeed || speed > maxMotorSpeed) {
    ESP_LOGW(TAG, "Invalid speed: %d", speed);
    return ESP_FAIL;
  }

  motorSpeed = speed;
  uint8_t pwmVal = (motorSpeed * 255) / 100;
  return setPwmDuty(pwmChannel, pwmVal);
}

void L298N::stop() {
  if (!isInitialized)
    return;
  gpio_set_level(phPin, motorStop);
  setPwmDuty(pwmChannel, motorStop);
}

uint8_t L298N::getMotorSpeed() const { return motorSpeed; }
Direction L298N::getMotorDirection() const { return motorDirection; }
bool L298N::motorIsRunning() const { return motorSpeed > 0; }

esp_err_t L298N::setPwmDuty(ledc_channel_t pwmChannel, uint8_t duty) {
  esp_err_t ret;
  ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, duty);
  if (ret != ESP_OK)
    return ret;

  ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
  return ret;
}

esp_err_t L298N::setPwmValue(ledc_timer_bit_t resolution, uint32_t frequency) {
  if (!isInitialized)
    return ESP_FAIL;
  if (frequency < MIN_PWM_FREQ || frequency > MAX_PWM_FREQ)
    return ESP_FAIL;

  this->resolution = resolution;
  this->frequency = frequency;

  ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                               .duty_resolution = resolution,
                               .timer_num = LEDC_TIMER_0,
                               .freq_hz = frequency,
                               .clk_cfg = LEDC_AUTO_CLK};
  return ledc_timer_config(&timer);
}

} // namespace L298N
} // namespace DC_Motor_Controller_Firmware
