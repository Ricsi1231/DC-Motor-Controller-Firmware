#include "DRV8876.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace DC_Motor_Controller_Firmware::DRV8876;

const char* TAG = "MAIN";

extern "C" void app_main() {
    DRV8876Config motorConfig = {
        .phPin = GPIO_NUM_4,
        .enPin = GPIO_NUM_5,
        .nFault = GPIO_NUM_6,
        .nSleep = GPIO_NUM_7,
        .pwmChannel = LEDC_CHANNEL_0,
        .resolution = LEDC_TIMER_10_BIT,
        .frequency = 20000,
        .minFrequency = 100,
        .maxFrequency = 100000,
        .rampStepPercent = 5,
        .rampStepDelayMs = 10,
        .minEffectivePwmPercent = 3,
    };

    DRV8876 motor(motorConfig);
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
