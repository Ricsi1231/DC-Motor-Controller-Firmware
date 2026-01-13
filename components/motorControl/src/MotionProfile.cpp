#include "MotionProfile.hpp"

namespace DC_Motor_Controller_Firmware::Control {

void MotionProfile::configure(MotionProfileType profileType, float accelLimit, float jerkLimit) {
    type = profileType;
    accelLimitPctPerSec = accelLimit;
    jerkLimitPctPerSec2 = jerkLimit;
}

void MotionProfile::setEnabled(bool enable) {
    enabled = enable;
}

bool MotionProfile::isEnabled() const {
    return enabled;
}

void MotionProfile::setMaxSpeed(float maxSpeed) {
    maxSpeedPct = maxSpeed;
}

float MotionProfile::shape(float desiredSpeed, uint64_t nowUs) {
    if (!enabled) {
        currentSpeed = desiredSpeed;
        lastUpdateUs = nowUs;
        return currentSpeed;
    }

    if (lastUpdateUs == 0) {
        lastUpdateUs = nowUs;
    }

    double timeStepSec = static_cast<double>(nowUs - lastUpdateUs) / 1e6;
    if (timeStepSec < 0.0) {
        timeStepSec = 0.0;
    }

    float desiredClamped = desiredSpeed;
    if (desiredClamped > maxSpeedPct) {
        desiredClamped = maxSpeedPct;
    }
    if (desiredClamped < -maxSpeedPct) {
        desiredClamped = -maxSpeedPct;
    }

    if (type == MotionProfileType::TRAPEZOID) {
        double maxDelta = static_cast<double>(accelLimitPctPerSec) * timeStepSec;
        double delta = static_cast<double>(desiredClamped) - static_cast<double>(currentSpeed);

        if (delta > maxDelta) {
            delta = maxDelta;
        }
        if (delta < -maxDelta) {
            delta = -maxDelta;
        }

        currentSpeed = static_cast<float>(static_cast<double>(currentSpeed) + delta);
    } else {
        double speedErr = static_cast<double>(desiredClamped) - static_cast<double>(currentSpeed);
        double accelTarget = 0.0;

        if (timeStepSec > 0.0) {
            accelTarget = speedErr / timeStepSec;
        }

        double accelCap = static_cast<double>(accelLimitPctPerSec);
        if (accelTarget > accelCap) {
            accelTarget = accelCap;
        }
        if (accelTarget < -accelCap) {
            accelTarget = -accelCap;
        }

        double jerkCap = static_cast<double>(jerkLimitPctPerSec2);
        double accelStepCap = jerkCap * timeStepSec;
        double accelStep = accelTarget - static_cast<double>(currentAccel);

        if (accelStep > accelStepCap) {
            accelStep = accelStepCap;
        }
        if (accelStep < -accelStepCap) {
            accelStep = -accelStepCap;
        }

        currentAccel = static_cast<float>(static_cast<double>(currentAccel) + accelStep);

        double newSpeed = static_cast<double>(currentSpeed) + static_cast<double>(currentAccel) * timeStepSec;

        if ((desiredClamped - currentSpeed) > 0.0f && newSpeed > static_cast<double>(desiredClamped)) {
            newSpeed = static_cast<double>(desiredClamped);
            currentAccel = 0.0f;
        }
        if ((desiredClamped - currentSpeed) < 0.0f && newSpeed < static_cast<double>(desiredClamped)) {
            newSpeed = static_cast<double>(desiredClamped);
            currentAccel = 0.0f;
        }

        currentSpeed = static_cast<float>(newSpeed);
    }

    lastUpdateUs = nowUs;
    return currentSpeed;
}

void MotionProfile::reset() {
    currentSpeed = 0.0f;
    currentAccel = 0.0f;
    lastUpdateUs = 0;
}

}  // namespace DC_Motor_Controller_Firmware::Control
