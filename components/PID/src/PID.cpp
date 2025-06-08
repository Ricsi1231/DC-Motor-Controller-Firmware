#include "PID.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

namespace DC_Motor_Controller_Firmware {
namespace PID {

PIDController::PIDController(const PIDConfig &config) : cfg(config) {}

void PIDController::setTunings(float p, float i, float d) {
  cfg.kp = p;
  cfg.ki = i;
  cfg.kd = d;
}

void PIDController::setOutputLimit(float limit) {
  cfg.outputMax = limit;
}

void PIDController::setIntegralLimit(float limit) {
  cfg.integralMax = limit;
}

void PIDController::reset() {
  eprev = 0;
  eintegral = 0;
  stuckStartTime = 0;
  smallErrorStartTime = 0;
  prevT = esp_timer_get_time();
}

float PIDController::compute(float setpoint, float actual, bool &shouldStop) {
  int64_t currT = esp_timer_get_time();
  float deltaT = (float)(currT - prevT) / 1e6f;
  if (deltaT <= 0) deltaT = 0.001f;
  prevT = currT;

  float e = setpoint - actual;
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;

  if (eintegral > cfg.integralMax) eintegral = cfg.integralMax;
  if (eintegral < -cfg.integralMax) eintegral = -cfg.integralMax;

  float u = cfg.kp * e + cfg.ki * eintegral + cfg.kd * dedt;
  float speed = fabs(u);
  if (speed > cfg.outputMax) speed = cfg.outputMax;

  if (fabs(e) < 5.0f && speed < cfg.lowErrorSpeedThreshold) {
    if (smallErrorStartTime == 0) smallErrorStartTime = currT;
    if ((currT - smallErrorStartTime) / 1e6f > cfg.lowErrorTimeout) {
      shouldStop = true;
      reset();
      ESP_LOGW("PID", "Forced stop due to low error + speed timeout");
      return 0;
    }
  } else {
    smallErrorStartTime = 0;
  }

  if (fabs(actual - lastPos) < cfg.minMotionDelta && speed < cfg.minStopSpeed) {
    if (stuckStartTime == 0) stuckStartTime = currT;
    if ((currT - stuckStartTime) / 1e6f > cfg.stuckTimeout) {
      shouldStop = true;
      reset();
      ESP_LOGW("PID", "Forced stop due to zero motion");
      return 0;
    }
  } else {
    stuckStartTime = 0;
  }

  lastPos = actual;

  if (fabs(e) < cfg.stopErrorThreshold && speed < cfg.minStopSpeed) {
    shouldStop = true;
    reset();
    return 0;
  }

  eprev = e;
  ESP_LOGI("PID", "e=%.2f dedt=%.2f u=%.2f speed=%.2f pos=%.2f", e, dedt, u, speed, actual);
  return u;
}

} // namespace PID
} // namespace DC_Motor_Controller_Firmware
