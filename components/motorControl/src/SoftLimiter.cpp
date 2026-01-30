#include "SoftLimiter.hpp"

using namespace DC_Motor_Controller_Firmware::Control;

void SoftLimiter::setLimits(float min, float max, bool enforce) {
    if (min > max) {
        float tmp = min;
        min = max;
        max = tmp;
        ESP_LOGW(TAG, "setLimits: min>max, swapped");
    }

    minDeg = min;
    maxDeg = max;
    enforced = enforce;

    ESP_LOGI(TAG, "Soft limits set: [%.3f, %.3f], %s", minDeg, maxDeg, enforced ? "ENFORCED" : "NOT ENFORCED");
}

bool SoftLimiter::isEnforced() const {
    return enforced;
}

float SoftLimiter::clampTarget(float degrees) const {
    if (!enforced) {
        return degrees;
    }

    if (degrees < minDeg) {
        ESP_LOGW(TAG, "Target below softMin (%.3f < %.3f), clamping", degrees, minDeg);
        return minDeg;
    }
    if (degrees > maxDeg) {
        ESP_LOGW(TAG, "Target above softMax (%.3f > %.3f), clamping", degrees, maxDeg);
        return maxDeg;
    }
    return degrees;
}

bool SoftLimiter::isBlocked(float signedCommand, float currentPosDeg) const {
    if (!enforced) {
        return false;
    }

    if (signedCommand < 0.0f && currentPosDeg <= minDeg) {
        ESP_LOGW(TAG, "limit: LEFT at %.3f (softMin %.3f), blocked", currentPosDeg, minDeg);
        return true;
    }
    if (signedCommand > 0.0f && currentPosDeg >= maxDeg) {
        ESP_LOGW(TAG, "limit: RIGHT at %.3f (softMax %.3f), blocked", currentPosDeg, maxDeg);
        return true;
    }
    return false;
}

float SoftLimiter::getMinDeg() const {
    return minDeg;
}

float SoftLimiter::getMaxDeg() const {
    return maxDeg;
}
