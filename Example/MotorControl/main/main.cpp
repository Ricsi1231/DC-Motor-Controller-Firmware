#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "MotorControl.hpp"
#include "PID.hpp"

const char *TAG = "MAIN";

using namespace DC_Motor_Controller_Firmware::motorControl;
using namespace DC_Motor_Controller_Firmware::PID_Controller;
using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;

#define PH_PIN (GPIO_NUM_5)
#define EN_PIN (GPIO_NUM_4)
#define FAULT_PIN (GPIO_NUM_6)
ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

#define ENCODER_A_PIN (GPIO_NUM_2)
#define ENCODER_B_PIN (GPIO_NUM_1)
#define PPR (1024)

pcnt_unit_config_t pcntUnit = {.low_limit = -1000,
                               .high_limit = 1000,
                               .flags = {
                                   .accum_count = true,
                               }};

esp_err_t status = ESP_OK;

extern "C" void app_main() {
  DRV8876 drv(PH_PIN, EN_PIN, FAULT_PIN, PWM_CHANNEL);
  Encoder encoder(ENCODER_A_PIN, ENCODER_B_PIN, pcntUnit, PPR);
  PID pid;
  MotorControl motor(drv, encoder, pid);

  status = motor.init();

  if (status != ESP_OK) {
    ESP_LOGD(TAG, "Error with motor application init");
    return;
  }

  ESP_LOGD(TAG, "Set motor to 90 degree");
  motor.setPositionDegrees(90);

  vTaskDelay(pdMS_TO_TICKS(1000));

  motor.setPositionDegrees(30);
  ESP_LOGD(TAG, "Set motor to 30 degree");
}
