#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "MotorCommHandler.hpp"
#include "PID.hpp"
#include "PeripheralSettings.hpp"
#include "Pinout.hpp"
#include "USB.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "motorControl.hpp"

using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;
using namespace DC_Motor_Controller_Firmware::USB;
using namespace DC_Motor_Controller_Firmware::Communication;
using namespace DC_Motor_Controller_Firmware::PID;
using namespace DC_Motor_Controller_Firmware::Control;

const char *TAG = "MAIN";

USB usb;
MotorCommHandler motorComm(usb);
DRV8876 motor(PH_PIN, EN_PIN, FAULT_PIN, PWM_CHANNEL);
Encoder encoder(ENCODER_A, ENCODER_B, pcntUnit, ppr);
PIDController pid(defaultConfig);
MotorController motorControl(encoder, motor, pid, motorCfg);

esp_err_t errorStatus = ESP_OK;
float kp = 1, ki = 0.04, kd = 0.025;
float targetDegree = 0, current = 0, offset = 0;

extern "C" void app_main() {
  errorStatus = motor.init();
  if (errorStatus != ESP_OK)
    ESP_LOGD(TAG, "Error with motor init");

  errorStatus = encoder.init();
  if (errorStatus != ESP_OK)
    ESP_LOGD(TAG, "Error with encoder init");

  errorStatus = usb.init();
  if (errorStatus != ESP_OK)
    ESP_LOGD(TAG, "Error with USB init");

  while (1) {
    motorComm.process();

    if (motorComm.isNewTargetReceived()) {
      current = encoder.getPositionInDegrees();
      offset = motorComm.getTargetDegrees();
      targetDegree = current + offset;

      motorControl.setTarget(targetDegree);
      motorComm.clearTarget();
    }

    if (motorComm.isNewPIDReceived()) {
      motorComm.getPIDParams(kp, ki, kd);
      motorControl.setPID(kp, ki, kd);
    }

    if (motorComm.wasPIDRequested()) {
      motorComm.sendPIDParams(kp, ki, kd);
    }

    if (fabs(encoder.getPositionInDegrees() == targetDegree) <= 2.0f) {
      // ESP_LOGI(TAG, "At target");
      motorComm.notifyMotorPositionReached();
      // ESP_LOGI(TAG, " data was sent");
    }

    // ESP_LOGI(TAG, " encoder: %f", encoder.getPositionInDegrees());

    motorControl.update();
    vTaskDelay(10);
  }
}