#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "MotorCommHandler.hpp"
#include "PID.hpp"
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

gpio_num_t PH_PIN = GPIO_NUM_4;
gpio_num_t EN_PIN = GPIO_NUM_5;
gpio_num_t FAULT_PIN = GPIO_NUM_6;
ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

gpio_num_t ENCODER_A = GPIO_NUM_1;
gpio_num_t ENCODER_B = GPIO_NUM_2;
uint16_t ppr = 1024;

pcnt_unit_config_t pcntUnit = {
    .low_limit = -1000,
    .high_limit = 1000,
    .flags = {
        .accum_count = true,
    }};

float setpointDegrees = 0;
bool motionDone = true;

DRV8876 motor(PH_PIN, EN_PIN, FAULT_PIN, PWM_CHANNEL);
Encoder encoder(ENCODER_A, ENCODER_B, pcntUnit, ppr);
USB usb;
MotorCommHandler motorComm(usb);

PidConfig defaultConfig = {
    .kp = 1.2f,
    .ki = 0.05f,     
    .kd = 0.02f,       
    .maxOutput = 100.0f,
    .maxIntegral = 300.0f,
    .errorEpsilon = 0.1f,  
    .speedEpsilon = 0.2f,  
    .errorTimeoutSec = 0.5f,
    .stuckTimeoutSec = 0.5f,
};
PIDController pid(defaultConfig);

MotorControllerConfig motorCfg = {
    .minSpeed = 2.0f,
    .maxSpeed = 100.0f,
    .minErrorToMove = 0.2f,
    .driftThreshold = 1.0f,
    .stuckPositionEpsilon = 0.05f,
    .stuckCountLimit = 50,
    .pidWarmupLimit = 10,
};
MotorController motorControl(encoder, motor, pid);

esp_err_t errorStatus = ESP_OK;

extern "C" void app_main() {
  errorStatus = motor.init();
  if (errorStatus != ESP_OK) ESP_LOGD(TAG, "Error with motor init");

  errorStatus = encoder.init();
  if (errorStatus != ESP_OK) ESP_LOGD(TAG, "Error with encoder init");

  errorStatus = usb.init();
  if (errorStatus != ESP_OK) ESP_LOGD(TAG, "Error with USB init");

  while (1) {
    motorComm.process();

  if (motorComm.isNewTargetReceived()) {
    float current = encoder.getPositionInDegrees();
        float offset = motorComm.getTargetDegrees();
        motorControl.setTarget(current + offset);
        motorComm.clearTarget();
    }

    if (motorComm.isNewPIDReceived()) {
        float kp, ki, kd;
        motorComm.getPIDParams(kp, ki, kd);
        motorControl.setPID(kp, ki, kd);
    }

    if (motorComm.wasPIDRequested()) {
        float kp, ki, kd;
        motorControl.getPID(kp, ki, kd);
        motorComm.sendPIDParams(kp, ki, kd);
    }

    motorControl.update();
    vTaskDelay(10);
  }
}