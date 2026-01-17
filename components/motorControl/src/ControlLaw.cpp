#include "ControlLaw.hpp"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

namespace DC_Motor_Controller_Firmware::Control {

static constexpr const char* TAG = "ControlLaw";

ControlLaw::ControlLaw(PID::PIDController& pidRef, const ControlLawConfig& cfg)
    : pid(pidRef), fuzzyPtr(nullptr), useFuzzy(false), config(cfg) {
    profile.configure(config.profileType, config.accelLimitPctPerSec, config.jerkLimitPctPerSec2);
    profile.setEnabled(config.motionProfileEnabled);
    profile.setMaxSpeed(config.maxSpeed);
}

ControlLaw::ControlLaw(PID::PIDController& pidRef, PID::FuzzyPIDController& fuzzy,
                       const ControlLawConfig& cfg)
    : pid(pidRef), fuzzyPtr(&fuzzy), useFuzzy(true), config(cfg) {
    profile.configure(config.profileType, config.accelLimitPctPerSec, config.jerkLimitPctPerSec2);
    profile.setEnabled(config.motionProfileEnabled);
    profile.setMaxSpeed(config.maxSpeed);
}

ControlOutput ControlLaw::compute(float target, float position, float velocity, uint64_t nowUs) {
    ControlOutput output;

    float pidSignal = 0.0f;
    if (useFuzzy && fuzzyPtr != nullptr) {
        pidSignal = fuzzyPtr->compute(target, position);
        output.settled = fuzzyPtr->isSettled();
    } else {
        pidSignal = pid.compute(target, position);
        output.settled = pid.isSettled();
    }

    pidSignal = clampToPercentRange(pidSignal);
    output.rawPidOutput = pidSignal;

    float refVelPct = profile.isEnabled() ? 0.0f : pidSignal;
    float ff = config.Kff_pos * target + config.Kff_vel * refVelPct;

    float combined = clampToPercentRange(pidSignal + ff);

    float error = target - position;
    float speedLimited = applySpeedLimits(combined, error);

    output.commandPct = profile.shape(speedLimited, nowUs);

    return output;
}

void ControlLaw::reset() {
    if (useFuzzy && fuzzyPtr != nullptr) {
        fuzzyPtr->reset();
    } else {
        pid.reset();
    }
    profile.reset();
    clampPrevValue = NAN;
}

void ControlLaw::setConfig(const ControlLawConfig& cfg) {
    config = cfg;
    profile.configure(config.profileType, config.accelLimitPctPerSec, config.jerkLimitPctPerSec2);
    profile.setEnabled(config.motionProfileEnabled);
    profile.setMaxSpeed(config.maxSpeed);
}

ControlLawConfig ControlLaw::getConfig() const noexcept {
    return config;
}

void ControlLaw::setPID(float kp, float ki, float kd) {
    pid.setParameters(kp, ki, kd);
    ESP_LOGI(TAG, "PID set: Kp=%.4f Ki=%.4f Kd=%.4f", kp, ki, kd);
}

void ControlLaw::getPID(float& kp, float& ki, float& kd) const {
    if (useFuzzy && fuzzyPtr != nullptr) {
        fuzzyPtr->getCurrentGains(kp, ki, kd);
    } else {
        pid.getParameters(kp, ki, kd);
    }
}

void ControlLaw::setFeedForward(float kpos, float kvel) {
    config.Kff_pos = kpos;
    config.Kff_vel = kvel;
    ESP_LOGI(TAG, "FF set: Kff_pos=%.4f Kff_vel=%.4f", kpos, kvel);
}

void ControlLaw::getFeedForward(float& kpos, float& kvel) const noexcept {
    kpos = config.Kff_pos;
    kvel = config.Kff_vel;
}

void ControlLaw::enableMotionProfile(bool enable) {
    config.motionProfileEnabled = enable;
    profile.setEnabled(enable);
    ESP_LOGI(TAG, "Profile %s", enable ? "ENABLED" : "DISABLED");
}

void ControlLaw::configureMotionProfile(MotionProfileType type, float accel, float jerk) {
    config.profileType = type;
    config.accelLimitPctPerSec = accel;
    config.jerkLimitPctPerSec2 = jerk;
    profile.configure(type, accel, jerk);
    ESP_LOGI(TAG, "Profile cfg: type=%s accel=%.1f jerk=%.1f",
             type == MotionProfileType::TRAPEZOID ? "TRAP" : "S", accel, jerk);
}

void ControlLaw::setMaxSpeed(float maxSpeed) {
    config.maxSpeed = maxSpeed;
    profile.setMaxSpeed(maxSpeed);
}

bool ControlLaw::isUsingFuzzy() const noexcept {
    return useFuzzy;
}

float ControlLaw::clampToPercentRange(float signal) const {
    float clampedValue = signal;

    if (!std::isfinite(clampedValue)) {
        if (clampedValue != clampPrevValue) {
            ESP_LOGW(TAG, "clamp: non-finite signal -> 0");
        }
        clampPrevValue = 0.0f;
        return 0.0f;
    }

    if (clampedValue > 100.0f) {
        if (clampPrevValue != 100.0f) {
            ESP_LOGW(TAG, "clamp: %.3f -> 100.0", clampedValue);
        }
        clampPrevValue = 100.0f;
        return 100.0f;
    }

    if (clampedValue < -100.0f) {
        if (clampPrevValue != -100.0f) {
            ESP_LOGW(TAG, "clamp: %.3f -> -100.0", clampedValue);
        }
        clampPrevValue = -100.0f;
        return -100.0f;
    }

    clampPrevValue = clampedValue;
    return clampedValue;
}

float ControlLaw::applySpeedLimits(float combined, float error) const {
    float errAbs = fabsf(error);
    float mag = fabsf(combined);

    if (mag < config.minSpeed && errAbs > config.minErrorToMove) {
        mag = config.minSpeed;
    }
    if (mag > config.maxSpeed) {
        mag = config.maxSpeed;
    }

    return (combined >= 0.0f) ? mag : -mag;
}

}  // namespace DC_Motor_Controller_Firmware::Control
