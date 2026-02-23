#include "SettleDetector.hpp"

using namespace DC_Motor_Controller_Firmware::Control;

bool SettleDetector::update(float errorAbs, float velocityDegPerSec, const SettleDetectorConfig& cfg) {
    bool posOk = (errorAbs < cfg.posTolDeg);
    bool velOk = (fabsf(velocityDegPerSec) < cfg.velTolDegPerSec);

    if (posOk && velOk) {
        settleCounter++;
    } else {
        settleCounter = 0;
    }

    return (settleCounter >= cfg.countLimit);
}

void SettleDetector::reset() { settleCounter = 0; }

int SettleDetector::getCount() const { return settleCounter; }
