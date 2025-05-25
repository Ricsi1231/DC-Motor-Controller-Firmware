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

  pid.setParameters(PID_KP, PID_KI, PID_KD, PID_KF);
  pid.setOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  pid.setMaxIOutput(PID_MAX_I_OUTPUT);
  pid.setDirection(PID_REVERSED);
  pid.setSetpoint(PID_INITIAL_SETPOINT);
  pid.setOutputRampRate(PID_RAMP_RATE);
  pid.setSetpointRange(PID_SETPOINT_RANGE);
  pid.setOutputFilter(PID_FILTER_STRENGTH);

  return ESP_OK;
}

void MotorControl::setPidParams(float kp, float ki, float kd, float kf) {
  pid.setParameters(kp, ki, kd, kf);
}

bool MotorControl::atTarget() const {
  float error = targetPosition - encoder.getPositionInDegrees();
  return fabs(error) < TARGET_THRESHOLD_DEG;
}

void MotorControl::applyOutput(float output) {
  float speed = fabs(output);
  if (speed > 100.0f)
    speed = 100.0f;

  auto dir =
      (output >= 0) ? DRV8876::Direction::RIGHT : DRV8876::Direction::LEFT;
  drv.setDirection(dir);
  drv.setSpeed(static_cast<uint8_t>(speed));
}

void MotorControl::setPositionDegrees(float degrees) {
  targetPosition = degrees;

  while (!atTarget()) {
    currentPosition = encoder.getPositionInDegrees();
    float output = pid.Update(targetPosition, currentPosition);
    applyOutput(output);
  }

  drv.stop();
  ESP_LOGD(TAG, "Final position: %.2f deg", currentPosition);
}

float MotorControl::currentPositionDeg() const { return currentPosition; }

} // namespace motorControl
} // namespace DC_Motor_Controller_Firmware
