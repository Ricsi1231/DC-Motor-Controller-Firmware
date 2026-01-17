#include "MotionMonitor.hpp"
#include "esp_log.h"
#include <cmath>

namespace DC_Motor_Controller_Firmware::Control {

static constexpr const char* TAG = "MotionMonitor";

MotionMonitor::MotionMonitor(const MotionMonitorConfig& cfg) : config(cfg) {
    MotionDetector::Config detectorCfg;
    detectorCfg.minErrorToMove = config.minErrorToMove;
    detectorCfg.stuckPositionEpsilon = config.stuckPositionEpsilon;
    detectorCfg.stuckCountLimit = config.stuckCountLimit;
    detectorCfg.pidWarmupLimit = config.pidWarmupLimit;
    detectorCfg.settlePosTolDeg = config.settlePosTolDeg;
    detectorCfg.settleVelTolDegPerSec = config.settleVelTolDegPerSec;
    detectorCfg.settleCountLimit = config.settleCountLimit;
    detector.setConfig(detectorCfg);

    limits.set(config.softLimitMinDeg, config.softLimitMaxDeg, config.softLimitsEnforced);
}

MotionEvent MotionMonitor::evaluate(const MotionObservation& obs) {
    if (checkTimeout(obs.nowUs)) {
        ESP_LOGW(TAG, "timeout detected");
        return MotionEvent::TIMEOUT;
    }

    if (checkLimitHit(obs.position, obs.commandPct)) {
        ESP_LOGW(TAG, "limit hit at %.3f deg", obs.position);
        return MotionEvent::LIMIT_HIT;
    }

    detector.update(obs.error, obs.velocity, obs.deltaPosition);

    if (detector.isSettled()) {
        ESP_LOGI(TAG, "settled detected");
        return MotionEvent::SETTLED;
    }

    if (detector.isStuck() || obs.pidSettled) {
        ESP_LOGI(TAG, "stall detected (stuck=%d, pidSettled=%d)",
                 detector.isStuck(), obs.pidSettled);
        return MotionEvent::STALLED;
    }

    return MotionEvent::NONE;
}

void MotionMonitor::startMotion(uint64_t nowUs) {
    motionStartUs = nowUs;
    motionActive = true;
    detector.startMotion();
}

void MotionMonitor::reset() {
    motionStartUs = 0;
    motionActive = false;
    detector.reset();
}

void MotionMonitor::setConfig(const MotionMonitorConfig& cfg) {
    config = cfg;

    MotionDetector::Config detectorCfg;
    detectorCfg.minErrorToMove = config.minErrorToMove;
    detectorCfg.stuckPositionEpsilon = config.stuckPositionEpsilon;
    detectorCfg.stuckCountLimit = config.stuckCountLimit;
    detectorCfg.pidWarmupLimit = config.pidWarmupLimit;
    detectorCfg.settlePosTolDeg = config.settlePosTolDeg;
    detectorCfg.settleVelTolDegPerSec = config.settleVelTolDegPerSec;
    detectorCfg.settleCountLimit = config.settleCountLimit;
    detector.setConfig(detectorCfg);

    limits.set(config.softLimitMinDeg, config.softLimitMaxDeg, config.softLimitsEnforced);
}

MotionMonitorConfig MotionMonitor::getConfig() const noexcept {
    return config;
}

void MotionMonitor::setSoftLimits(float minDeg, float maxDeg, bool enforce) {
    config.softLimitMinDeg = minDeg;
    config.softLimitMaxDeg = maxDeg;
    config.softLimitsEnforced = enforce;
    limits.set(minDeg, maxDeg, enforce);
    ESP_LOGI(TAG, "Soft limits set: [%.3f, %.3f] %s", minDeg, maxDeg,
             enforce ? "ENFORCED" : "NOT ENFORCED");
}

float MotionMonitor::clampTarget(float target) const {
    return limits.clampTarget(target);
}

bool MotionMonitor::areSoftLimitsEnforced() const noexcept {
    return limits.isEnforced();
}

float MotionMonitor::getSoftLimitMin() const noexcept {
    return limits.getMin();
}

float MotionMonitor::getSoftLimitMax() const noexcept {
    return limits.getMax();
}

int MotionMonitor::getStuckCount() const noexcept {
    return detector.getStuckCount();
}

int MotionMonitor::getSettleCount() const noexcept {
    return detector.getSettleCount();
}

void MotionMonitor::setMotionTimeoutMs(uint32_t ms) {
    config.motionTimeoutMs = static_cast<int>(ms);
    ESP_LOGI(TAG, "Motion timeout set: %u ms", static_cast<unsigned>(ms));
}

uint32_t MotionMonitor::getMotionTimeoutMs() const noexcept {
    return static_cast<uint32_t>(config.motionTimeoutMs);
}

void MotionMonitor::setPidWarmupCycles(int cycles) {
    if (cycles < 0) {
        cycles = 0;
    }
    config.pidWarmupLimit = cycles;

    MotionDetector::Config detectorCfg;
    detectorCfg.minErrorToMove = config.minErrorToMove;
    detectorCfg.stuckPositionEpsilon = config.stuckPositionEpsilon;
    detectorCfg.stuckCountLimit = config.stuckCountLimit;
    detectorCfg.pidWarmupLimit = cycles;
    detectorCfg.settlePosTolDeg = config.settlePosTolDeg;
    detectorCfg.settleVelTolDegPerSec = config.settleVelTolDegPerSec;
    detectorCfg.settleCountLimit = config.settleCountLimit;
    detector.setConfig(detectorCfg);

    ESP_LOGI(TAG, "PID warmup cycles=%d", cycles);
}

int MotionMonitor::getPidWarmupCycles() const noexcept {
    return config.pidWarmupLimit;
}

bool MotionMonitor::shouldWakeFromIdle(float target, float position) const {
    float drift = target - position;
    float wakeTh = config.driftDeadband + config.driftHysteresis;
    if (wakeTh < 0.0f) {
        wakeTh = 0.0f;
    }
    return fabsf(drift) > wakeTh;
}

bool MotionMonitor::isWithinDriftDeadband(float error) const {
    return fabsf(error) <= config.driftDeadband;
}

bool MotionMonitor::checkTimeout(uint64_t nowUs) const {
    if (config.motionTimeoutMs <= 0) {
        return false;
    }

    if (motionStartUs == 0) {
        return false;
    }

    uint64_t elapsedUs = nowUs - motionStartUs;
    uint64_t limitUs = static_cast<uint64_t>(config.motionTimeoutMs) * 1000ULL;

    if (elapsedUs > limitUs) {
        ESP_LOGW(TAG, "timeout: %llu ms > %u ms",
                 (unsigned long long)(elapsedUs / 1000ULL),
                 static_cast<unsigned int>(config.motionTimeoutMs));
        return true;
    }
    return false;
}

bool MotionMonitor::checkLimitHit(float position, float commandPct) const {
    if (!limits.isEnforced()) {
        return false;
    }
    return limits.isAtLimit(position, commandPct);
}

}  // namespace DC_Motor_Controller_Firmware::Control
