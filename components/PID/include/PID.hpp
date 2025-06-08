#pragma once

#include <cstdint>

namespace DC_Motor_Controller_Firmware {
namespace PID {

struct PidConfig {
  float kp;
  float ki;
  float kd;
  float maxOutput = 100.0f;
  float maxIntegral = 1000.0f;
  float errorEpsilon = 2.0f;
  float speedEpsilon = 7.0f;
  float errorTimeoutSec = 0.6f;
  float stuckTimeoutSec = 0.5f;
};

class PIDController {
public:
  PIDController(const PidConfig &cfg);
  void reset();

  float compute(float setpoint, float actual);
  void setParameters(float kp, float ki, float kd);
  void getParameters(float &kp, float &ki, float &kd);

  bool isSettled() const;
  float getLastError() const;
  float getLastDerivative() const;

private:
  PidConfig config;

  float integral = 0.0f;
  float lastError = 0.0f;
  float lastDerivative = 0.0f;
  float lastOutput = 0.0f;

  uint64_t lastTimeUs = 0;
  bool settled = true;

  int64_t stuckStartTime = 0;
  int64_t smallErrorStartTime = 0;
};

} // namespace PID
} // namespace DC_Motor_Controller_Firmware
