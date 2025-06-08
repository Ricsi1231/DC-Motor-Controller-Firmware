#pragma once

#include <stdint.h>

namespace DC_Motor_Controller_Firmware {
namespace PID {

struct PIDConfig {
  float kp = 1.0f;
  float ki = 0.0f;
  float kd = 0.025f;
  float outputMax = 100.0f;
  float integralMax = 1000.0f;
  float stopErrorThreshold = 2.0f;
  float minStopSpeed = 7.0f;
  float lowErrorSpeedThreshold = 10.0f;
  float lowErrorTimeout = 0.6f;
  float stuckTimeout = 0.5f;
  float minMotionDelta = 0.05f;
};

class PIDController {
public:
  PIDController(const PIDConfig &config = PIDConfig());

  void setTunings(float p, float i, float d);
  void setOutputLimit(float limit);
  void setIntegralLimit(float limit);
  void reset();

  float compute(float setpoint, float actual, bool &shouldStop);

private:
  PIDConfig cfg;
  float eprev = 0;
  float eintegral = 0;
  float lastPos = 0;

  int64_t prevT = 0;
  int64_t stuckStartTime = 0;
  int64_t smallErrorStartTime = 0;
};

} // namespace PID
} // namespace DC_Motor_Controller_Firmware
