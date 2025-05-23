#include "Encoder.hpp"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace DC_Motor_Controller_Firmware::Encoder;

#define ENCODER_A_GPIO GPIO_NUM_15
#define ENCODER_B_GPIO GPIO_NUM_16

#define FORWARD true
#define BACKWARD false

#define ENCODER_STEPS 100
#define ENCODER_DELAY 1000

uint8_t simulationTime = 0;

void initEncoderSimulation() {
  gpio_reset_pin(ENCODER_A_GPIO);
  gpio_reset_pin(ENCODER_B_GPIO);

  gpio_set_direction(ENCODER_A_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(ENCODER_B_GPIO, GPIO_MODE_OUTPUT);
}

void encoderSimulation(int steps, int delay_us = 1000,
                       bool rotationDirection = true) {
  if (rotationDirection == true) {
    const int states[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};

    for (int i = 0; i < steps; ++i) {
      for (int s = 0; s < 4; ++s) {
        gpio_set_level(ENCODER_A_GPIO, states[s][0]);
        esp_rom_delay_us(delay_us);
        gpio_set_level(ENCODER_B_GPIO, states[s][1]);
        esp_rom_delay_us(delay_us);
      }
    }
  } else {
    const int states[4][2] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};

    for (int i = 0; i < steps; ++i) {
      for (int s = 0; s < 4; ++s) {
        gpio_set_level(ENCODER_A_GPIO, states[s][0]);
        esp_rom_delay_us(delay_us);
        gpio_set_level(ENCODER_B_GPIO, states[s][1]);
        esp_rom_delay_us(delay_us);
      }
    }
  }
}

extern "C" void app_main() {
  pcnt_unit_config_t pcntUnit = {};
  pcntUnit.high_limit = 1000;
  pcntUnit.low_limit = -1000;
  pcntUnit.flags.accum_count = true;

  Encoder encoder(GPIO_NUM_1, GPIO_NUM_2, pcntUnit, 1024);
  initEncoderSimulation();

  esp_err_t err = encoder.init();

  if (err != ESP_OK) {
    ESP_LOGE("ENCODER", "Failed to initialize encoder");
    return;
  }

  while (true) {
    encoderSimulation(ENCODER_STEPS, ENCODER_DELAY, BACKWARD);

    uint32_t ticks = encoder.getPositionTicks();
    float degrees = encoder.getPositionInDegrees();
    int32_t rpm = encoder.getPositionInRPM();

    ESP_LOGI("ENCODER", "Ticks: %lu, Degrees: %.2f, RPM: %ld", ticks, degrees,
             rpm);

    if (simulationTime == 10) {
      ESP_LOGI("ENCODER", "Reset Postion");
      encoder.resetPositon();

      simulationTime = 0;
    }

    if (encoder.getMotorDriection() ==
        DC_Motor_Controller_Firmware::Encoder::motorDirection::LEFT) {
      ESP_LOGI("ENCODER", "Motor rotiting to left");
    } else {
      ESP_LOGI("ENCODER", "Motor rotiting to right");
    }

    simulationTime++;

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
