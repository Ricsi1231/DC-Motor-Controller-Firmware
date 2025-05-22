#include "USB.hpp"
#include "MotorCommHandler.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

using namespace DC_Motor_Controller_Firmware;
using namespace DC_Motor_Controller_Firmware::Communication;

static USB::USB usb;
static MotorCommHandler motorComm(usb);

float target = 0;
float lastTarget = 1.0f;

float kp, ki, kd;

extern "C" void app_main() {
    usb.init();

    while(true) {
        motorComm.process();

        if(motorComm.isNewTargetReceived()) {
            target = motorComm.getTargetDegrees();
        
            if(target != lastTarget) {
                ESP_LOGI("MAIN", "New target position received: %.2f", target);
                
                motorComm.sendMotorState(target);
        
                lastTarget = target;

                ESP_LOGI("MAIN", "Motor response sent for target: %.2f", target);
            } 
            else {
                motorComm.notifyMotorPositionReached();
                ESP_LOGI("MAIN", "Repeated target %.2f ignored", target);
            }
        }

        if(motorComm.isNewPIDReceived()) {
            motorComm.getPIDParams(kp, ki, kd);
            ESP_LOGI("MAIN", "Received PID: %.2f, %.2f, %.2f", kp, ki, kd);
        }

        if(motorComm.wasPIDRequested()) {
            motorComm.getPIDParams(kp, ki, kd);
            ESP_LOGI("MAIN", "PID requested. Sending current values.");

            motorComm.sendPIDParams(kp, ki, kd);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
