#include "PID.hpp"
#include "esp_timer.h"
#include "math.h"

using namespace DC_Motor_Controller_Firmware::PID;

PIDController::PIDController(const PidConfig &cfg) : config(cfg) {
  reset();
}

void PIDController::reset() {
  integral = 0.0f;
  lastError = 0.0f;
  lastDerivative = 0.0f;
  lastOutput = 0.0f;
  lastTimeUs = 0;
  stuckStartTime = 0;
  smallErrorStartTime = 0;
  settled = true;
}

void PIDController::setParameters(float kp, float ki, float kd) {
  config.kp = kp;
  config.ki = ki;
  config.kd = kd;
}

void PIDController::getParameters(float &kp, float &ki, float &kd) {
  kp = config.kp;
  ki = config.ki;
  kd = config.kd;
}

bool PIDController::isSettled() const {
  return settled;
}

float PIDController::getLastError() const {
  return lastError;
}

float PIDController::getLastDerivative() const {
  return lastDerivative;
}

float PIDController::compute(float setpoint, float measured) {
  float error = setpoint - measured;
  uint64_t now = esp_timer_get_time();

  if (lastTimeUs == 0) {
    lastTimeUs = now;
    lastError = error;
    lastDerivative = 0.0f;
    return 0.0f;
  }

  float dt = (now - lastTimeUs) / 1e6f;
  if (dt <= 0.000001f) dt = 0.001f;  

  if (fabsf(error) < config.errorEpsilon) {
    error = 0.0f;
    settled = true;
  } else {
    settled = false;
  }

  lastDerivative = (error - lastError) / dt;

  if (fabsf(error) < config.maxOutput) {
    integral += error * dt;
    if (integral > config.maxIntegral) integral = config.maxIntegral;
    if (integral < -config.maxIntegral) integral = -config.maxIntegral;
  }

  float output = config.kp * error + config.ki * integral + config.kd * lastDerivative;

  if (output > config.maxOutput) output = config.maxOutput;
  if (output < -config.maxOutput) output = -config.maxOutput;

  lastOutput = output;
  lastError = error;
  lastTimeUs = now;

  return output;
}

