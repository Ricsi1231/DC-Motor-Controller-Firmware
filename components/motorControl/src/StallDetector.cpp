#include "StallDetector.hpp"

using namespace DC_Motor_Controller_Firmware::Control;

StallDetector::Result StallDetector::update(float positionDelta, float errorAbs, bool pidIsSettled, const StallDetectorConfig& cfg) {
    lastWarmupLimit = cfg.pidWarmupLimit;

    if (pidWarmupCounter <= cfg.pidWarmupLimit) {
        stuckCounter = 0;
        pidWarmupCounter++;
        return Result::WARMING_UP;
    }

    float moveAbs = fabsf(positionDelta);
    if (errorAbs > cfg.minErrorToMove && moveAbs < cfg.stuckPositionEpsilon) {
        stuckCounter++;
    } else {
        stuckCounter = 0;
    }

    if (stuckCounter > cfg.stuckCountLimit || pidIsSettled) {
        return Result::STALLED;
    }

    return Result::OK;
}

void StallDetector::reset() {
    stuckCounter = 0;
    pidWarmupCounter = 0;
}

int StallDetector::getStuckCount() const { return stuckCounter; }

bool StallDetector::isWarming() const { return (pidWarmupCounter <= lastWarmupLimit); }
