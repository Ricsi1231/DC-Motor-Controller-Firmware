#include "DRV8876.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace DC_Motor_Controller_Firmware::DRV8876;

const char *TAG = "MAIN";

extern "C" void app_main() {
  gpio_num_t PH_PIN = GPIO_NUM_4;
  gpio_num_t EN_PIN = GPIO_NUM_5;
  gpio_num_t FAULT_PIN = GPIO_NUM_6;
  ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

  DRV8876 motor(PH_PIN, EN_PIN, FAULT_PIN, PWM_CHANNEL);
  motor.init();

  while (1) {
    ESP_LOGI(TAG, "Starting motor...");

    motor.setDirection(Direction::RIGHT);
    motor.setSpeed(60);
    vTaskDelay(pdMS_TO_TICKS(3000));

    motor.stop();

    ESP_LOGI(TAG, "Changing direction...");

    motor.setDirection(Direction::LEFT);
    motor.setSpeed(80);
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Stopping motor...");
    motor.stop();

    if (motor.isFaultTriggered()) {
      ESP_LOGI(TAG, "Fault dedected");

      motor.clearFaultFlag();
    }
  }
}
