#pragma once

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "PID.hpp"

namespace DC_Motor_Controller_Firmware {
namespace motorControl {

struct PidConfig {
  float kp;
  float ki;
  float kd;
  float kf;

  float outputMin;
  float outputMax;
  float maxIOutput;
  bool reversed;
  float initialSetpoint;
  float rampRate;
  float setpointRange;
  float filterStrength;
};

class MotorControl {
public:
  MotorControl(DRV8876::DRV8876 &driver, Encoder::Encoder &encoder,
               PID_Controller::PID &pid);

  esp_err_t init();

  void setPositionDegrees(float degrees);
  void applyOutput(float output);

  void setPidParams(float kp, float ki, float kd, float kf = 0.0f);
  void setPidParameters(PidConfig pidConfig);
  PidConfig getDefaultPidConfig() const;
  PidConfig getCurrentPidConfig() const;

  bool atTarget() const;

  float currentPositionDeg() const;

private:
  DRV8876::DRV8876 &drv;
  Encoder::Encoder &encoder;
  PID_Controller::PID &pid;

  float currentPosition = 0.0f;
  float targetPosition = 0.0f;

  PidConfig pidConfig;
  PidConfig defaultPidConfig;

  const char *TAG = "MotorControl";
};

} // namespace motorControl
} // namespace DC_Motor_Controller_Firmware
