#pragma once

#include "Encoder.hpp"
#include "DRV8876.hpp"
#include "PID.hpp"

namespace DC_Motor_Controller_Firmware::Control {

struct MotorControllerConfig {
  float minSpeed = 2.0f;
  float maxSpeed = 100.0f;
  float minErrorToMove = 0.2f;
  float driftThreshold = 1.0f;
  float stuckPositionEpsilon = 0.05f;
  int stuckCountLimit = 50;
  int pidWarmupLimit = 10;
};

class MotorController {
public:
  MotorController(Encoder::Encoder& enc,
                  DRV8876::DRV8876& drv,
                  PID::PIDController& pid,
                  const MotorControllerConfig& cfg = MotorControllerConfig());

  void setTarget(float degrees);
  void setPID(float kp, float ki, float kd);
  void getPID(float& kp, float& ki, float& kd);
  void update();
  bool isMotionDone() const;

private:
  Encoder::Encoder& encoder;
  DRV8876::DRV8876& motor;
  PID::PIDController& pid;
  MotorControllerConfig config;

  float target = 0;
  bool motionDone = true;
  float lastPos = 0;
  int stuckCounter = 0;
  int pidWarmupCounter = 0;
};

} // namespace DC_Motor_Controller_Firmware::Control
