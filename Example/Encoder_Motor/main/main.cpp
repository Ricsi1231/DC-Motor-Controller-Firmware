#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "DRV8876.hpp"
#include "Encoder.hpp"

using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;

const char *TAG = "MAIN";

gpio_num_t PH_PIN = GPIO_NUM_4;
gpio_num_t EN_PIN = GPIO_NUM_5;
gpio_num_t FAULT_PIN = GPIO_NUM_6;

ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

gpio_num_t ENCODER_A = GPIO_NUM_1;
gpio_num_t ENCODER_B = GPIO_NUM_2;
uint16_t ppr = 1024;

pcnt_unit_config_t pcntUnit = {.low_limit = -1000,
                               .high_limit = 1000,
                               .flags = {
                                   .accum_count = true,
                               }};


DRV8876 motor(PH_PIN, EN_PIN, FAULT_PIN, PWM_CHANNEL);
Encoder encoder(ENCODER_A, ENCODER_B, pcntUnit, ppr);

void testMotor(bool rotation);
                            
extern "C" void app_main() {
  esp_err_t errorStatus = ESP_OK;

  bool testMotor = false;
  bool testEncoder = false;
  bool pidControl = false;
  
  errorStatus = motor.init();
  if (errorStatus != ESP_OK) {
    ESP_LOGD(TAG, "Error with motor init");
  }

  encoder.init();
  if (errorStatus != ESP_OK) {
    ESP_LOGD(TAG, "Error with encoder init");
  }

}

void testMotor(bool rotation) {
  if (rotation) {
    motor.setDirection(Direction::LEFT);
    motor.setSpeed(100);
    motor.stop();

    vTaskDelay(1000);
    motor.setSpeed(60);
    motor.stop();
  } else {
    motor.setDirection(Direction::RIGHT);
    motor.setSpeed(100);
    motor.stop();

    vTaskDelay(1000);
    motor.setSpeed(60);
    motor.stop();
  }
}
