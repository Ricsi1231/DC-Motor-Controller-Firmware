#pragma once

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "L298N.hpp"
#include "PID.hpp"

namespace DC_Motor_Controller_Firmware {
namespace motorControl {

enum class DriverType { L298N_DRIVER, DRV8876_DRIVER };

class MotorControl {
public:
  MotorControl(DC_Motor_Controller_Firmware::L298N::L298N &driver,
               DC_Motor_Controller_Firmware::Encoder::Encoder &encoder,
               DC_Motor_Controller_Firmware::PID_Controller::PID &pid);

  MotorControl(DC_Motor_Controller_Firmware::DRV8876::DRV8876 &driver,
               DC_Motor_Controller_Firmware::Encoder::Encoder &encoder,
               DC_Motor_Controller_Firmware::PID_Controller::PID &pid);

  esp_err_t init();

  void setPositionDegrees(float degrees);
  void applyOutput(float output);

  void setPidParams(float kp, float ki, float kd, float kf = 0.0f);

  bool atTarget() const;

private:
  DC_Motor_Controller_Firmware::PID_Controller::PID &pid;
  DC_Motor_Controller_Firmware::Encoder::Encoder &encoder;

  DC_Motor_Controller_Firmware::L298N::L298N *l298n = nullptr;
  DC_Motor_Controller_Firmware::DRV8876::DRV8876 *drv = nullptr;

  DriverType driverType;

  float currentPosition = 0.0f;
  float targetPosition = 0.0f;
};

} // namespace motorControl
} // namespace DC_Motor_Controller_Firmware
