#pragma once

#include <stdint.h>
namespace DC_Motor_Controller_Firmware {
namespace PID_Controller {
class PID {
public:
  PID();
  float Update(float setpoint, float measurement);
  float getOutput(float actual);

  void setParameters(float kp, float ki, float kd, float kf);
  void setOutputLimits(float limMin, float limMax);
  void setMaxIOutput(float maximum);
  void setDirection(bool reversed);
  void setSetpoint(float setpoint);
  void setOutputRampRate(float rate);
  void setSetpointRange(float range);
  void setOutputFilter(float strength);

  void reset();

private:
  void checkSigns();
  float clamp(float value, float min, float max);
  bool bounded(float value, float min, float max);

  bool reversed;
  bool firstRun;

  float kp;
  float ki;
  float kd;
  float kf;

  float errorSum;

  float limMin;
  float limMax;

  float maxIOutput;

  float integrator;
  float prevError;
  float differentiator;

  float setpoint;
  float setpointRange;

  float outputRampRate;
  float outputFilter;

  float prevMeasurement;
  float lastOutput;
  float out;
};
} // namespace PID_Controller
} // namespace DC_Motor_Controller_Firmware