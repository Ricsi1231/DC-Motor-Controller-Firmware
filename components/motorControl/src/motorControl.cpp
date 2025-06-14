#include "motorControl.hpp"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

using namespace DC_Motor_Controller_Firmware;
using namespace Control;

MotorController::MotorController(Encoder::Encoder &enc, DRV8876::DRV8876 &drv,
                                 PID::PIDController &pid,
                                 const MotorControllerConfig &cfg)
    : encoder(enc), motor(drv), pid(pid), config(cfg) {}

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

void MotorController::getPID(float &kp, float &ki, float &kd) {
  pid.getParameters(kp, ki, kd);
}

bool MotorController::isMotionDone() const { return motionDone; }

void MotorController::update() {
  if (motionDone) {
    float drift = target - encoder.getPositionInDegrees();
    if (fabsf(drift) > config.driftThreshold) {
      // ESP_LOGI("HOLD", "Drift correction: error=%.2f", drift);
      setTarget(target);
    }
    return;
  }

  float currentPos = encoder.getPositionInDegrees();
  float output = pid.compute(target, currentPos);
  float speed = fabsf(output);

  if (speed < config.minSpeed &&
      fabsf(target - currentPos) > config.minErrorToMove)
    speed = config.minSpeed;
  speed = std::clamp(speed, config.minSpeed, config.maxSpeed);

  if (fabsf(currentPos - lastPos) < config.stuckPositionEpsilon)
    stuckCounter++;
  else
    stuckCounter = 0;
  lastPos = currentPos;

  if (stuckCounter > config.stuckCountLimit ||
      (pidWarmupCounter > config.pidWarmupLimit && pid.isSettled())) {
    // ESP_LOGW("LABVIEW", "Motion done or stuck → stopping");
    motor.stop();
    motionDone = true;
    return;
  }

  pidWarmupCounter++;
  motor.setDirection(output < 0 ? DRV8876::Direction::LEFT
                                : DRV8876::Direction::RIGHT);
  motor.setSpeed(speed);
}
