#include "MotorControl.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include "MotorControlConfig.hpp"

using namespace DC_Motor_Controller_Firmware::L298N;
using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;
using namespace DC_Motor_Controller_Firmware::PID_Controller;
using namespace DC_Motor_Controller_Firmware::motorControl::config;

namespace DC_Motor_Controller_Firmware {
namespace motorControl {
MotorControl::MotorControl(L298N &driver, Encoder &encoder, PID &pid)
    : l298n(&driver), encoder(encoder), pid(pid),
      driverType(DriverType::L298N_DRIVER) {}

MotorControl::MotorControl(DRV8876 &driver, Encoder &encoder, PID &pid)
    : drv(&driver), encoder(encoder), pid(pid),
      driverType(DriverType::DRV8876_DRIVER) {}

esp_err_t MotorControl::init() {
   esp_err_t status = ESP_OK; 

  if(DriverType::DRV8876_DRIVER) {
    status = drv.init();

    if(status != ESP_OK) {
      ESP_LOGW(TAG, "motorControl init - Error with DRV IC init");
      return ESP_FAIL;
    }
  } else {
    status = l298n.init();

    if(status != ESP_OK) {
      ESP_LOGW(TAG, "motorControl init - Error with l298n IC init");
      return ESP_FAIL;
    }
  }

  status = encoder.init();

  if(status != ESP_OK) {
      ESP_LOGW(TAG, "motorControl init - Error with encoder init");
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

bool MotorControl::atTarget() const {
  float error = targetPosition - encoder.getPositionInDegrees();
  return fabs(error) < 2.0f;
}

void MotorControl::setPidParams(float kp, float ki, float kd, float kf) {
  pid.setParameters(kp, ki, kd, kf);
}

void MotorControl::applyOutput(float output) {
  float speed = fabs(output);
  if (speed > 100.0f)
    speed = 100.0f;

  Direction dir = (output >= 0) ? Direction::RIGHT : Direction::LEFT;

  if (driverType == DriverType::L298N_DRIVER) {
    l298n.setDirection(dir);
    l298n.setSpeed(static_cast<uint8_t>(speed));
  } else if (driverType == DriverType::DRV8876_DRIVER) {
    drv.setDirection(dir);
    drv.setSpeed(static_cast<uint8_t>(speed));
  }
}

void MotorControl::setPositionDegrees(float degrees) {
  targetPosition = degrees;

  while (!atTarget()) {
    currentPosition = encoder.getPositionInDegrees();
    float output = pid.Update(targetPosition, currentPosition);
    applyOutput(output);
  }

  if (driverType == DriverType::L298N_DRIVER)
    l298n.stop();
  if (driverType == DriverType::DRV8876_DRIVER)
    drv.stop();
}

} // namespace motorControl
} // namespace DC_Motor_Controller_Firmware
