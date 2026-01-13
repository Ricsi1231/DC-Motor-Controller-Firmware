#include "MotionDetector.hpp"

namespace DC_Motor_Controller_Firmware::Control {

void MotionDetector::setConfig(const Config& cfg) {
    config = cfg;
}

void MotionDetector::update(float error, float velocity, float deltaPos) {
    if (!inMotion) {
        return;
    }

    warmupCounter++;

    if (warmupCounter <= config.pidWarmupLimit) {
        stuckCounter = 0;
        settleCounter = 0;
        return;
    }

    float errAbs = fabsf(error);
    float moveAbs = fabsf(deltaPos);

    if (errAbs > config.minErrorToMove && moveAbs < config.stuckPositionEpsilon) {
        stuckCounter++;
    } else {
        stuckCounter = 0;
    }

    bool posOk = (errAbs < config.settlePosTolDeg);
    bool velOk = (fabsf(velocity) < config.settleVelTolDegPerSec);

    if (posOk && velOk) {
        settleCounter++;
    } else {
        settleCounter = 0;
    }
}

bool MotionDetector::isSettled() const {
    return settleCounter >= config.settleCountLimit;
}

bool MotionDetector::isStuck() const {
    return stuckCounter > config.stuckCountLimit;
}

int MotionDetector::getStuckCount() const {
    return stuckCounter;
}

int MotionDetector::getSettleCount() const {
    return settleCounter;
}

void MotionDetector::reset() {
    stuckCounter = 0;
    settleCounter = 0;
    warmupCounter = 0;
    inMotion = false;
}

void MotionDetector::startMotion() {
    reset();
    inMotion = true;
}

}  // namespace DC_Motor_Controller_Firmware::Control
