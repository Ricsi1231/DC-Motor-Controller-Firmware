#include "MotorCommHandler.hpp"
#include "PeripheralSettings.hpp"
#include "motorControl.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <memory>

using namespace DC_Motor_Controller_Firmware;

const char* TAG = "MAIN APP";

std::unique_ptr<Control::MotorController> motorControl;
std::unique_ptr<Communication::MotorCommHandler> motorComm;

extern "C" void app_main() {
    motorControl = std::make_unique<Control::MotorController>(motorConfig, encoderConfig, defaultConfig, motorCfg);

    esp_err_t err = motorControl->init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MotorController init failed: %s", esp_err_to_name(err));
        return;
    }

    motorComm = std::make_unique<Communication::MotorCommHandler>(motorControl->getMotorControl(), motorControl->getEncoder());

    err = motorComm->init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MotorCommHandler init failed: %s", esp_err_to_name(err));
        return;
    }

    motorComm->startTask();
    motorControl->startTask();

    ESP_LOGI(TAG, "System initialized and running");
}
