#include "motorControl.hpp"
#include "esp_log.h"
#include <cmath>

using namespace DC_Motor_Controller_Firmware;
using namespace Control;

MotorController::MotorController(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pid)
  : encoder(enc), motor(drv), pid(pid) {}

void MotorController::setTarget(float degrees) {
  target = degrees;
  pid.reset();
  motionDone = false;
  lastPos = encoder.getPositionInDegrees();
  stuckCounter = 0;
  pidWarmupCounter = 0;
}

void MotorController::setPID(float kp, float ki, float kd) {
  pid.setParameters(kp, ki, kd);
}

void MotorController::getPID(float& kp, float& ki, float& kd) {
  pid.getParameters(kp, ki, kd);
}

bool MotorController::isMotionDone() const {
  return motionDone;
}

void MotorController::update() {
  if (motionDone) {
    float drift = target - encoder.getPositionInDegrees();
    if (fabs(drift) > 1.0f) {
      ESP_LOGI("HOLD", "Drift correction: error=%.2f", drift);
      setTarget(target); // Reapply same target
    }
    return;
  }

  float currentPos = encoder.getPositionInDegrees();
  float output = pid.compute(target, currentPos);

  float speed = fabs(output);
  if (speed < 2.0f && fabs(target - currentPos) > 0.2f)
    speed = 2.0f;
  if (speed > 100.0f) speed = 100.0f;

  if (fabs(currentPos - lastPos) < 0.05f)
    stuckCounter++;
  else
    stuckCounter = 0;
  lastPos = currentPos;

  if (stuckCounter > 50 || (pidWarmupCounter > 10 && pid.isSettled())) {
    ESP_LOGW("LABVIEW", "Motion done or stuck → stopping");
    motor.stop();
    motionDone = true;
    return;
  }

  pidWarmupCounter++;
  motor.setDirection(output < 0 ? DRV8876::Direction::LEFT : DRV8876::Direction::RIGHT);
  motor.setSpeed(speed);
}
