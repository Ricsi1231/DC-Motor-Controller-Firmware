#pragma once

#include "MotorCommHandler.hpp"
#include "motorControl.hpp"
#include "Encoder.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace DC_Motor_Controller_Firmware::Logic {

class CommLogicHandler {
public:
  CommLogicHandler(Communication::MotorCommHandler &comm,
                   Control::MotorController &motor,
                   Encoder::Encoder &enc);

  void startTask();

private:
  Communication::MotorCommHandler &motorComm;
  Control::MotorController &motorControl;
  Encoder::Encoder &encoder;

  static void taskFunc(void *param);
  TaskHandle_t taskHandle = nullptr;

  float kp, ki, kd;
  float current = 0, offset = 0, targetDegree = 0;
  bool settled = false;

  bool getPIDValuesFirsTime = true;
};

} // namespace DC_Motor_Controller_Firmware::Logic
