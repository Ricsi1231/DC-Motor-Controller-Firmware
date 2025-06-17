#include "DRV8876.hpp"
#include "esp_log.h"

namespace DC_Motor_Controller_Firmware {
namespace DRV8876 {

DRV8876::DRV8876(gpio_num_t phPin, gpio_num_t enPin, gpio_num_t nFault,
                 ledc_channel_t pwmChannel)
    : phPin(phPin), enPin(enPin), nFault(nFault), pwmChannel(pwmChannel),
      resolution(LEDC_TIMER_8_BIT), frequency(20000) {}

DRV8876::~DRV8876() {
  ESP_LOGI(TAG, "DRV Destructor called");
}

esp_err_t DRV8876::init() {
  esp_err_t errorEsp = ESP_OK;

  gpio_config_t ioConf = {};
  ioConf.intr_type = GPIO_INTR_DISABLE;
  ioConf.mode = GPIO_MODE_OUTPUT;
  ioConf.pin_bit_mask = (1ULL << phPin);
  errorEsp = gpio_config(&ioConf);
  if (errorEsp != ESP_OK)
    return ESP_FAIL;

  gpio_config_t faultConf = {};
  faultConf.intr_type = GPIO_INTR_NEGEDGE;
  faultConf.mode = GPIO_MODE_INPUT;
  faultConf.pin_bit_mask = (1ULL << nFault);
  faultConf.pull_up_en = GPIO_PULLUP_ENABLE;
  errorEsp = gpio_config(&faultConf);
  if (errorEsp != ESP_OK)
    return ESP_FAIL;

  errorEsp = gpio_install_isr_service(0);
  if (errorEsp != ESP_OK)
    return ESP_FAIL;

  errorEsp = gpio_isr_handler_add(nFault, faultISR, this);
  if (errorEsp != ESP_OK)
    return ESP_FAIL;

  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = resolution;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = frequency;
  timer.clk_cfg = LEDC_AUTO_CLK;
  errorEsp = ledc_timer_config(&timer);
  if (errorEsp != ESP_OK)
    return ESP_FAIL;

  ledc_channel_config_t channelConf = {};
  channelConf.gpio_num = enPin;
  channelConf.speed_mode = LEDC_LOW_SPEED_MODE;
  channelConf.channel = pwmChannel;
  channelConf.intr_type = LEDC_INTR_DISABLE;
  channelConf.timer_sel = LEDC_TIMER_0;
  channelConf.duty = 0;
  channelConf.hpoint = 0;
  channelConf.flags.output_invert = false;
  // channelConf.sleep_mode = LEDC_SLEEP_MODE_DISABLE;
  errorEsp = ledc_channel_config(&channelConf);
  if (errorEsp != ESP_OK)
    return ESP_FAIL;

  isInitialized = true;
  return ESP_OK;
}

void DRV8876::setDirection(Direction direction) {
  if (!isInitialized)
    return;
  motorDirection = direction;
  gpio_set_level(phPin, static_cast<bool>(motorDirection));
}

void DRV8876::reverseDirection() {
  if (!isInitialized)
    return;
  motorDirection =
      (motorDirection == Direction::LEFT) ? Direction::RIGHT : Direction::LEFT;
  gpio_set_level(phPin, static_cast<bool>(motorDirection));
}

esp_err_t DRV8876::setSpeed(uint8_t speed) {
  if (!isInitialized)
    return ESP_FAIL;
  if (speed < minMotorSpeed || speed > maxMotorSpeed)
    return ESP_FAIL;

  motorSpeed = speed;
  uint8_t pwmVal = (motorSpeed * 255) / 100;
  return setPwmDuty(pwmChannel, pwmVal);
}

void DRV8876::stop() {
  if (!isInitialized)
    return;
  motorSpeed = 0;
  gpio_set_level(phPin, motorStop);
  setPwmDuty(pwmChannel, motorStop);
}

uint8_t DRV8876::getMotorSpeed() const { return motorSpeed; }
Direction DRV8876::getMotorDirection() const { return motorDirection; }
bool DRV8876::motorIsRunning() const { return motorSpeed > 0; }

esp_err_t DRV8876::setPwmDuty(ledc_channel_t pwmChannel, uint8_t duty) {
  uint32_t scaledDuty = (duty * ((1 << resolution) - 1)) / 255;
  esp_err_t res = ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, scaledDuty);
  if (res != ESP_OK)
    return ESP_FAIL;
  res = ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
  return (res == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t DRV8876::setPwmValue(ledc_timer_bit_t resolution,
                               uint32_t frequency) {
  if (!isInitialized)
    return ESP_FAIL;
  if (frequency < MIN_PWM_FREQ || frequency > MAX_PWM_FREQ)
    return ESP_FAIL;

  this->resolution = resolution;
  this->frequency = frequency;

  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = resolution;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = frequency;
  timer.clk_cfg = LEDC_AUTO_CLK;

  esp_err_t err = ledc_timer_config(&timer);
  if (err != ESP_OK)
    return ESP_FAIL;

  uint8_t pwmVal = (motorSpeed * 255) / 100;
  return setPwmDuty(pwmChannel, pwmVal);
}

void IRAM_ATTR DRV8876::faultISR(void *arg) {
  static_cast<DRV8876 *>(arg)->faultTriggered = true;
}

bool DRV8876::isFaultTriggered() const { return faultTriggered; }
void DRV8876::clearFaultFlag() { faultTriggered = false; }

} // namespace DRV8876
} // namespace DC_Motor_Controller_Firmware
