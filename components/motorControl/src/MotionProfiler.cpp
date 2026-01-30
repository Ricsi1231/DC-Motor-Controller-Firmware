#include "MotionProfiler.hpp"

using namespace DC_Motor_Controller_Firmware::Control;

float MotionProfiler::shapeSpeed(float desiredSigned, uint64_t nowUs, const MotionProfilerConfig& cfg) {
    if (cfg.enabled == false) {
        profSpeedPercent = desiredSigned;
        lastProfileUs = nowUs;
        ESP_LOGD(TAG, "profile: OFF desired=%.2f -> prof=%.2f", desiredSigned, profSpeedPercent);
        return profSpeedPercent;
    }

    if (lastProfileUs == 0) {
        lastProfileUs = nowUs;
    }

    double timeStepSec = static_cast<double>(nowUs - lastProfileUs) / 1e6;
    if (timeStepSec < 0.0) {
        timeStepSec = 0.0;
    }

    float desiredClamped = desiredSigned;
    if (desiredClamped > cfg.maxSpeed) {
        desiredClamped = cfg.maxSpeed;
    }
    if (desiredClamped < -cfg.maxSpeed) {
        desiredClamped = -cfg.maxSpeed;
    }

    if (cfg.type == MotionProfileType::TRAPEZOID) {
        double maxDelta = static_cast<double>(cfg.accelLimitPctPerSec) * timeStepSec;
        double delta = static_cast<double>(desiredClamped) - static_cast<double>(profSpeedPercent);

        if (delta > maxDelta) {
            delta = maxDelta;
        }
        if (delta < -maxDelta) {
            delta = -maxDelta;
        }

        profSpeedPercent = static_cast<float>(static_cast<double>(profSpeedPercent) + delta);
        ESP_LOGD(TAG, "profile TRAP: dt=%.4f des=%.2f maxDelta=%.2f -> prof=%.2f", timeStepSec, desiredClamped, maxDelta, profSpeedPercent);
    } else {
        double speedErr = static_cast<double>(desiredClamped) - static_cast<double>(profSpeedPercent);
        double accelTarget = 0.0;

        if (timeStepSec > 0.0) {
            accelTarget = speedErr / timeStepSec;
        }

        double accelCap = static_cast<double>(cfg.accelLimitPctPerSec);
        if (accelTarget > accelCap) {
            accelTarget = accelCap;
        }
        if (accelTarget < -accelCap) {
            accelTarget = -accelCap;
        }

        double jerkCap = static_cast<double>(cfg.jerkLimitPctPerSec2);
        double accelStepCap = jerkCap * timeStepSec;
        double accelStep = accelTarget - static_cast<double>(profAccelPctPerSec);

        if (accelStep > accelStepCap) {
            accelStep = accelStepCap;
        }
        if (accelStep < -accelStepCap) {
            accelStep = -accelStepCap;
        }

        profAccelPctPerSec = static_cast<float>(static_cast<double>(profAccelPctPerSec) + accelStep);

        double newSpeed = static_cast<double>(profSpeedPercent) + static_cast<double>(profAccelPctPerSec) * timeStepSec;

        if ((desiredClamped - profSpeedPercent) > 0.0f && newSpeed > static_cast<double>(desiredClamped)) {
            newSpeed = static_cast<double>(desiredClamped);
            profAccelPctPerSec = 0.0f;
        }
        if ((desiredClamped - profSpeedPercent) < 0.0f && newSpeed < static_cast<double>(desiredClamped)) {
            newSpeed = static_cast<double>(desiredClamped);
            profAccelPctPerSec = 0.0f;
        }

        profSpeedPercent = static_cast<float>(newSpeed);
        ESP_LOGD(TAG, "profile S: dt=%.4f des=%.2f a*=%.2f j*=%.2f -> a=%.2f prof=%.2f", timeStepSec, desiredClamped, accelTarget, jerkCap, profAccelPctPerSec,
                 profSpeedPercent);
    }

    lastProfileUs = nowUs;
    return profSpeedPercent;
}

float MotionProfiler::applySlew(float profiled, uint64_t nowUs, float accelLimitPctPerSec) {
    if (slewLastUs == 0) {
        slewLastUs = nowUs;
        slewLastOutPct = 0.0f;
    }

    double dt = static_cast<double>(nowUs - slewLastUs) / 1e6;
    if (dt < 0.0) {
        dt = 0.0;
    }

    double maxDelta = static_cast<double>(accelLimitPctPerSec) * dt;
    double reqDelta = static_cast<double>(profiled) - static_cast<double>(slewLastOutPct);

    if (reqDelta > maxDelta) {
        reqDelta = maxDelta;
    }
    if (reqDelta < -maxDelta) {
        reqDelta = -maxDelta;
    }

    float slewOut = static_cast<float>(static_cast<double>(slewLastOutPct) + reqDelta);

    slewLastOutPct = slewOut;
    slewLastUs = nowUs;

    return slewOut;
}

void MotionProfiler::reset(uint64_t nowUs) {
    profSpeedPercent = 0.0f;
    profAccelPctPerSec = 0.0f;
    lastProfileUs = nowUs;
    slewLastOutPct = 0.0f;
    slewLastUs = 0;
}

float MotionProfiler::getProfiledSpeed() const {
    return profSpeedPercent;
}
