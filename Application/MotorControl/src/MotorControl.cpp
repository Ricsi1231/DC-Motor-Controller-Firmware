#include "MotorControl.hpp"
#include "MotorControlConfig.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;
using namespace DC_Motor_Controller_Firmware::PID_Controller;
using namespace DC_Motor_Controller_Firmware::motorControl::config;

namespace DC_Motor_Controller_Firmware {
namespace motorControl {

MotorControl::MotorControl(DRV8876::DRV8876 &driver, Encoder::Encoder &encoder,
                           PID_Controller::PID &pid)
    : drv(driver), encoder(encoder), pid(pid) {}

esp_err_t MotorControl::init() {
  esp_err_t status = drv.init();
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "DRV8876 init failed");
    return ESP_FAIL;
  }

  status = encoder.init();
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "Encoder init failed");
    return ESP_FAIL;
  }

  defaultPidConfig.kp              = PID_KP;
  defaultPidConfig.ki              = PID_KI;
  defaultPidConfig.kd              = PID_KD;
  defaultPidConfig.kf              = PID_KF;

  defaultPidConfig.outputMin       = PID_OUTPUT_MIN;
  defaultPidConfig.outputMax       = PID_OUTPUT_MAX;
  defaultPidConfig.maxIOutput      = PID_MAX_I_OUTPUT;
  defaultPidConfig.reversed        = PID_REVERSED;
  defaultPidConfig.initialSetpoint = PID_INITIAL_SETPOINT;
  defaultPidConfig.rampRate        = PID_RAMP_RATE;
  defaultPidConfig.setpointRange   = PID_SETPOINT_RANGE;
  defaultPidConfig.filterStrength  = PID_FILTER_STRENGTH;

  pid.setParameters(defaultPidConfig.kp, defaultPidConfig.ki, defaultPidConfig.kd, defaultPidConfig.kf);
  pid.setOutputLimits(defaultPidConfig.outputMin, defaultPidConfig.outputMax);
  pid.setMaxIOutput(defaultPidConfig.maxIOutput);
  pid.setDirection(defaultPidConfig.reversed);
  pid.setSetpoint(defaultPidConfig.initialSetpoint);
  pid.setOutputRampRate(defaultPidConfig.rampRate);
  pid.setSetpointRange(defaultPidConfig.setpointRange);
  pid.setOutputFilter(defaultPidConfig.filterStrength);

  return ESP_OK;
}

void MotorControl::setPidParams(float kp, float ki, float kd, float kf) {
  pidConfig.kp = kp;
  pidConfig.ki = ki;
  pidConfig.kd = kd;
  pidConfig.kf = kf;

  pid.setParameters(pidConfig.kp, pidConfig.ki, pidConfig.kd, pidConfig.kf);
}

void MotorControl::setPidParameters(PidConfig pidConfig) {
  pid.setParameters(pidConfig.kp, pidConfig.ki, pidConfig.kd, pidConfig.kf);
  pid.setOutputLimits(pidConfig.outputMin, pidConfig.outputMax);
  pid.setMaxIOutput(pidConfig.maxIOutput);
  pid.setDirection(pidConfig.reversed);
  pid.setSetpoint(pidConfig.initialSetpoint);
  pid.setOutputRampRate(pidConfig.rampRate);
  pid.setSetpointRange(pidConfig.setpointRange);
  pid.setOutputFilter(pidConfig.filterStrength);

  this->pidConfig = pidConfig;
}

PidConfig MotorControl::getDefaultPidConfig() const { return defaultPidConfig; }
PidConfig MotorControl::getCurrentPidConfig() const { return pidConfig; }

bool MotorControl::atTarget() const {
  float error = targetPosition - encoder.getPositionInDegrees();
  return fabs(error) < TARGET_THRESHOLD_DEG;
}

void MotorControl::applyOutput(float output) {
  float speed = fabs(output);
  if (speed > 100.0f)
    speed = 100.0f;

  DRV8876::Direction dir;
  if(output >= 0) {
    dir = DRV8876::Direction::LEFT;
  } else {
    dir = DRV8876::Direction::RIGHT;
  }

  drv.setDirection(dir);
  drv.setSpeed(static_cast<uint8_t>(speed));
}

void MotorControl::setPositionDegrees(float degrees) {
  targetPosition = degrees;

  while (!atTarget()) {
    currentPosition = encoder.getPositionInDegrees();
    float output = pid.Update(targetPosition, currentPosition);
    applyOutput(output);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  drv.stop();
  ESP_LOGD(TAG, "Final position: %.2f deg", currentPosition);
}

float MotorControl::currentPositionDeg() const { return currentPosition; }

} // namespace motorControl
} // namespace DC_Motor_Controller_Firmware
