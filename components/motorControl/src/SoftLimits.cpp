#include "SoftLimits.hpp"

namespace DC_Motor_Controller_Firmware::Control {

void SoftLimits::set(float min, float max, bool enforce) {
    if (min > max) {
        float tmp = min;
        min = max;
        max = tmp;
    }

    minDeg = min;
    maxDeg = max;
    enforced = enforce;
}

float SoftLimits::clampTarget(float target) const {
    if (!enforced) {
        return target;
    }

    if (target < minDeg) {
        return minDeg;
    }
    if (target > maxDeg) {
        return maxDeg;
    }
    return target;
}

bool SoftLimits::isAtLimit(float position, float command) const {
    if (!enforced) {
        return false;
    }

    if (command < 0.0f && position <= minDeg) {
        return true;
    }
    if (command > 0.0f && position >= maxDeg) {
        return true;
    }
    return false;
}

bool SoftLimits::isEnforced() const {
    return enforced;
}

float SoftLimits::getMin() const {
    return minDeg;
}

float SoftLimits::getMax() const {
    return maxDeg;
}

}  // namespace DC_Motor_Controller_Firmware::Control
