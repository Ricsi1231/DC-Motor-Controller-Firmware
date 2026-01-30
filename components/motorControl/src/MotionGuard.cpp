#include "MotionGuard.hpp"

using namespace DC_Motor_Controller_Firmware::Control;

void MotionGuard::startMotion(uint64_t nowUs) {
    motionStartUs = nowUs;
}

void MotionGuard::reset() {
    motionStartUs = 0;
}

bool MotionGuard::isTimedOut(uint64_t nowUs, const MotionGuardConfig& cfg) const {
    if (cfg.motionTimeoutMs <= 0) {
        return false;
    }

    if (motionStartUs == 0) {
        return false;
    }

    uint64_t elapsedUs = nowUs - motionStartUs;
    uint64_t limitUs = static_cast<uint64_t>(cfg.motionTimeoutMs) * 1000ULL;

    if (elapsedUs > limitUs) {
        ESP_LOGW(TAG, "timeout: %llu ms > %d ms", (unsigned long long)(elapsedUs / 1000ULL), cfg.motionTimeoutMs);
        return true;
    }

    return false;
}

bool MotionGuard::shouldWake(float drift, const MotionGuardConfig& cfg) const {
    float wakeThreshold = cfg.driftDeadband + cfg.driftHysteresis;
    if (wakeThreshold < 0.0f) {
        wakeThreshold = 0.0f;
    }
    return (fabsf(drift) > wakeThreshold);
}

float MotionGuard::getDriftDeadband(const MotionGuardConfig& cfg) {
    return cfg.driftDeadband;
}

uint64_t MotionGuard::getMotionStartUs() const {
    return motionStartUs;
}
