#include "PID.hpp"
#include <algorithm>

namespace DC_Motor_Controller_Firmware {
namespace PID_Controller {
PID::PID() {
  integrator = 0.0f;
  prevError = 0.0f;
  differentiator = 0.0f;
  prevMeasurement = 0.0f;
  out = 0.0f;

  firstRun = true;
}

float PID::Update(float setpoint, float measurement) {
  double Poutput;
  double Ioutput;
  double Doutput;
  double Foutput;
  float error;

  this->setpoint = setpoint;

  if (setpointRange != 0) {
    setpoint = clamp(setpoint, measurement - setpointRange,
                     measurement + setpointRange);
  }

  error = setpoint - measurement;
  Foutput = kf * setpoint;
  Poutput = kp * error;

  if (firstRun) {
    prevMeasurement = measurement;
    prevError = error;

    firstRun = false;
  }

  Doutput = -kd * (measurement - prevMeasurement);
  Ioutput = ki * errorSum;

  if (maxIOutput != 0) {
    Ioutput = clamp(Ioutput, -maxIOutput, maxIOutput);
  }

  out = Foutput + Poutput + Ioutput + Doutput;

  if ((limMin != limMax) && !bounded(out, limMin, limMax)) {
    errorSum = error;
  } else if (maxIOutput != 0) {
    errorSum = error;
  } else {
    errorSum += error;
  }

  if (outputRampRate != 0) {
    out = clamp(out, lastOutput - outputRampRate, lastOutput + outputRampRate);
  }

  if (limMin != limMax) {
    out = clamp(out, limMin, limMax);
  }

  if (outputFilter != 0) {
    out = lastOutput * outputFilter + out * (1 - outputFilter);
  }

  lastOutput = out;

  return out;
}

float PID::getOutput(float actual) { return Update(setpoint, actual); }

void PID::setParameters(float kp, float ki, float kd, float kf) {
  if (this->ki != 0) {
    errorSum = errorSum * this->ki / ki;
  }

  if (maxIOutput != 0) {
    limMax = maxIOutput / ki;
  }

  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->kf = kf;

  checkSigns();
}

void PID::setOutputLimits(float limMin, float limMax) {
  if (limMin > limMax) {
    std::swap(limMin, limMax);
  }

  this->limMin = limMin;
  this->limMax = limMax;

  if (maxIOutput == 0 || (maxIOutput > (limMax - limMin))) {
    setMaxIOutput(limMax - limMin);
  }
}

void PID::setDirection(bool reversed) { this->reversed = reversed; }

void PID::setSetpoint(float setpoint) { this->setpoint = setpoint; }

void PID::setMaxIOutput(float maximum) {
  maxIOutput = maximum;

  if (ki != 0) {
    limMax = maxIOutput / ki;
  }
}

void PID::setOutputRampRate(float rate) { outputRampRate = rate; }

void PID::setSetpointRange(float range) { setpointRange = range; }

void PID::setOutputFilter(float strength) {
  if (strength == 0 || bounded(strength, 0, 1)) {
    outputFilter = strength;
  }
}

void PID::reset() {
  firstRun = true;
  errorSum = 0;

  integrator = 0;
  prevError = 0;
  prevMeasurement = 0;
  lastOutput = 0;
}

void PID::checkSigns() {
  if (reversed) {
    if (kp > 0)
      kp *= -1;
    if (ki > 0)
      ki *= -1;
    if (kd > 0)
      kd *= -1;
  } else {
    if (kp < 0)
      kp *= -1;
    if (ki < 0)
      ki *= -1;
    if (kd < 0)
      kd *= -1;
  }
}

float PID::clamp(float value, float min, float max) {
  if (value > max) {
    value = max;
  }

  if (value < min) {
    value = min;
  }

  return value;
}

bool PID::bounded(float value, float min, float max) {
  return (min < value) && (value < max);
}
} // namespace PID_Controller
} // namespace DC_Motor_Controller_Firmware