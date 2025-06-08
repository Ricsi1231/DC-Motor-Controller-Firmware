#pragma once

#include "Encoder.hpp"
#include "DRV8876.hpp"
#include "PID.hpp"

namespace DC_Motor_Controller_Firmware::Control {

class MotorController {
public:
  MotorController(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pid);

  void setTarget(float degrees);
  void setPID(float kp, float ki, float kd);
  void getPID(float& kp, float& ki, float& kd);
  void update();
  bool isMotionDone() const;

private:
  Encoder::Encoder& encoder;
  DRV8876::DRV8876& motor;
  PID::PIDController& pid;

  float target = 0;
  bool motionDone = true;
  float lastPos = 0;
  int stuckCounter = 0;
  int pidWarmupCounter = 0;
};

} // namespace DC_Motor_Controller_Firmware::Control
