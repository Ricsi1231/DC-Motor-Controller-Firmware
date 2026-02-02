#include "MotorCommHandler.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace DC_Motor_Controller_Firmware;
using namespace DC_Motor_Controller_Firmware::Communication;

static MotorCommHandler motorCommHandler;

float target = 0;
float lastTarget = 1.0f;

float kp, ki, kd;

extern "C" void app_main() {
    esp_err_t errorStatus = motorCommHandler.init();
    if (errorStatus != ESP_OK) {
        ESP_LOGE("MAIN", "MotorCommHandler init failed: %s", esp_err_to_name(errorStatus));
        return;
    }

    while (true) {
        motorCommHandler.process();

        if (motorCommHandler.isNewTargetReceived()) {
            target = motorCommHandler.getTargetDegrees();

            if (target != lastTarget) {
                ESP_LOGI("MAIN", "New target position received: %.2f", target);

                motorCommHandler.sendMotorState(target);

                lastTarget = target;

                ESP_LOGI("MAIN", "Motor response sent for target: %.2f", target);
            } else {
                motorCommHandler.notifyMotorPositionReached();
                ESP_LOGI("MAIN", "Repeated target %.2f ignored", target);
            }
        }

        if (motorCommHandler.isNewPIDReceived()) {
            motorCommHandler.getPIDParams(kp, ki, kd);
            ESP_LOGI("MAIN", "Received PID: %.2f, %.2f, %.2f", kp, ki, kd);
        }

        if (motorCommHandler.wasPIDRequested()) {
            motorCommHandler.getPIDParams(kp, ki, kd);
            ESP_LOGI("MAIN", "PID requested. Sending current values.");

            motorCommHandler.sendPIDParams(kp, ki, kd);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
