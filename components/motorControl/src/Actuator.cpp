#include "Actuator.hpp"
#include "esp_log.h"
#include <cmath>

namespace DC_Motor_Controller_Firmware::Control {

static constexpr const char* TAG = "Actuator";

Actuator::Actuator(DRV8876::DRV8876& driverRef, const ActuatorConfig& cfg)
    : driver(driverRef), config(cfg) {
}

esp_err_t Actuator::drive(float commandPct, uint64_t nowUs) {
    float slewOut = applySlewLimit(commandPct, nowUs);
    currentOutputPct = slewOut;

    float speedPct = fabsf(slewOut);
    if (speedPct > config.maxSpeed) {
        speedPct = config.maxSpeed;
    }

    DRV8876::Direction dir = (slewOut >= 0.0f) ? DRV8876::Direction::RIGHT : DRV8876::Direction::LEFT;

    driver.setDirection(dir);
    esp_err_t err = driver.setSpeed(static_cast<uint8_t>(speedPct));

    return err;
}

esp_err_t Actuator::stop() {
    currentOutputPct = 0.0f;
    return driver.stop();
}

esp_err_t Actuator::emergencyStop() {
    currentOutputPct = 0.0f;
    lastSlewUs = 0;
    return driver.stop();
}

esp_err_t Actuator::coast() {
    currentOutputPct = 0.0f;
    return driver.coast();
}

esp_err_t Actuator::brake() {
    currentOutputPct = 0.0f;
    return driver.brake();
}

void Actuator::setConfig(const ActuatorConfig& cfg) {
    config = cfg;
}

ActuatorConfig Actuator::getConfig() const noexcept {
    return config;
}

float Actuator::getCurrentOutput() const noexcept {
    return currentOutputPct;
}

DRV8876::Direction Actuator::getDirection() const noexcept {
    return driver.getMotorDirection();
}

bool Actuator::isRunning() const noexcept {
    return driver.motorIsRunning();
}

bool Actuator::isFaultDetected() const noexcept {
    return driver.isFaultTriggered();
}

void Actuator::clearFault() {
    driver.clearFaultFlag();
}

void Actuator::resetSlewState() {
    lastSlewUs = 0;
    currentOutputPct = 0.0f;
}

void Actuator::setSlewRate(float pctPerSec) {
    config.slewRatePctPerSec = pctPerSec;
}

float Actuator::getSlewRate() const noexcept {
    return config.slewRatePctPerSec;
}

void Actuator::setMaxSpeed(float maxSpeed) {
    config.maxSpeed = maxSpeed;
}

float Actuator::applySlewLimit(float targetPct, uint64_t nowUs) {
    if (lastSlewUs == 0) {
        lastSlewUs = nowUs;
        currentOutputPct = 0.0f;
    }

    double dt = static_cast<double>(nowUs - lastSlewUs) / 1e6;
    if (dt < 0.0) {
        dt = 0.0;
    }
    lastSlewUs = nowUs;

    double maxDelta = static_cast<double>(config.slewRatePctPerSec) * dt;
    double reqDelta = static_cast<double>(targetPct) - static_cast<double>(currentOutputPct);

    if (reqDelta > maxDelta) {
        reqDelta = maxDelta;
    }
    if (reqDelta < -maxDelta) {
        reqDelta = -maxDelta;
    }

    return static_cast<float>(static_cast<double>(currentOutputPct) + reqDelta);
}

}  // namespace DC_Motor_Controller_Firmware::Control
