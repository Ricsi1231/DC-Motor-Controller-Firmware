#include "MotorCommHandler.hpp"
#include "PeripheralSettings.hpp"
#include "motorControl.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <memory>

using namespace DC_Motor_Controller_Firmware;

const char* TAG = "MAIN APP";

std::unique_ptr<Control::MotorController> motorController;
std::unique_ptr<Communication::MotorCommHandler> motorCommHandler;

extern "C" void app_main() {
    motorController = std::make_unique<Control::MotorController>(motorDriverConfig, encoderConfig, pidConfig, motorControllerConfig);

    esp_err_t errorStatus = motorController->init();
    if (errorStatus != ESP_OK) {
        ESP_LOGE(TAG, "MotorController init failed: %s", esp_err_to_name(errorStatus));
        return;
    }

    motorCommHandler = std::make_unique<Communication::MotorCommHandler>(motorController->getMotorControl(), motorController->getEncoder());

    errorStatus = motorCommHandler->init();
    if (errorStatus != ESP_OK) {
        ESP_LOGE(TAG, "MotorCommHandler init failed: %s", esp_err_to_name(errorStatus));
        return;
    }

    motorCommHandler->startTask();
    motorController->startTask();

    ESP_LOGI(TAG, "System initialized and running");
}
