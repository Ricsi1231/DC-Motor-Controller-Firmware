/**
 * @file main.cpp
 * @brief Entry point for the DC Motor Controller firmware.
 *
 * Initializes all hardware components and starts the control tasks:
 * - DRV8876 motor driver
 * - Quadrature encoder via PCNT
 * - PID controller
 * - USB CDC communication interface
 * - Motor command handler (MotorCommHandler)
 * - High-level control logic (MotorController)
 * - Application logic bridge (CommLogicHandler)
 *
 * This file configures the firmware's runtime behavior by starting
 * FreeRTOS tasks for communication, control, and logic orchestration.
 */

#include "CommLogicHandler.hpp"
#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "MotorCommHandler.hpp"
#include "PID.hpp"
#include "PeripheralSettings.hpp"
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
using namespace DC_Motor_Controller_Firmware::Logic;

const char *TAG = "MAIN APP";

USB usb;
MotorCommHandler motorComm(usb);
DRV8876 motor(motorConfig);
Encoder encoder(encoderConfig);
PIDController pid(defaultConfig);
MotorController motorControl(encoder, motor, pid, motorCfg);
CommLogicHandler commLogic(motorComm, motorControl, encoder);

esp_err_t errorStatus = ESP_OK;

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

  motorComm.startTask();
  motorControl.startTask();
  commLogic.startTask();
}