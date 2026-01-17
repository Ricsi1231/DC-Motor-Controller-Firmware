#include "MotorControl.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>

using namespace DC_Motor_Controller_Firmware;
using namespace Control;

static ControlLawConfig buildControlLawConfig(const MotorControlConfig& cfg) {
    ControlLawConfig lawCfg;
    lawCfg.minSpeed = cfg.minSpeed;
    lawCfg.maxSpeed = cfg.maxSpeed;
    lawCfg.minErrorToMove = cfg.minErrorToMove;
    lawCfg.Kff_pos = cfg.Kff_pos;
    lawCfg.Kff_vel = cfg.Kff_vel;
    lawCfg.motionProfileEnabled = cfg.motionProfileEnabled;
    lawCfg.profileType = cfg.motionProfileType;
    lawCfg.accelLimitPctPerSec = cfg.accelLimitPctPerSec;
    lawCfg.jerkLimitPctPerSec2 = cfg.jerkLimitPctPerSec2;
    return lawCfg;
}

static MotionMonitorConfig buildMotionMonitorConfig(const MotorControlConfig& cfg) {
    MotionMonitorConfig monCfg;
    monCfg.minErrorToMove = cfg.minErrorToMove;
    monCfg.stuckPositionEpsilon = cfg.stuckPositionEpsilon;
    monCfg.stuckCountLimit = cfg.stuckCountLimit;
    monCfg.pidWarmupLimit = cfg.pidWarmupLimit;
    monCfg.settlePosTolDeg = cfg.settlePosTolDeg;
    monCfg.settleVelTolDegPerSec = cfg.settleVelTolDegPerSec;
    monCfg.settleCountLimit = cfg.settleCountLimit;
    monCfg.motionTimeoutMs = cfg.motionTimeoutMs;
    monCfg.driftDeadband = cfg.driftDeadband;
    monCfg.driftHysteresis = cfg.driftHysteresis;
    return monCfg;
}

static ActuatorConfig buildActuatorConfig(const MotorControlConfig& cfg) {
    ActuatorConfig actCfg;
    actCfg.slewRatePctPerSec = cfg.accelLimitPctPerSec;
    actCfg.maxSpeed = cfg.maxSpeed;
    return actCfg;
}

MotorControl::MotorControl(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pidRef,
                           const MotorControlConfig& initialConfig)
    : encoder(enc),
      pid(pidRef),
      fuzzy(PID::FuzzyPidConfig{}),
      controlLaw(pidRef, buildControlLawConfig(initialConfig)),
      motionMonitor(buildMotionMonitorConfig(initialConfig)),
      actuator(drv, buildActuatorConfig(initialConfig)),
      controlTask(),
      config(initialConfig) {
    ESP_LOGI(TAG, "Ctor: creating mutexes");
    targetMutex = xSemaphoreCreateMutex();
    if (targetMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create target mutex");
    }

    configMutex = xSemaphoreCreateMutex();
    if (configMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create config mutex");
    }

    controlTask.setUpdateCallback([this]() { this->update(); });

    syncComponentsFromConfig();
    ESP_LOGI(TAG, "Ctor done");
}

MotorControl::MotorControl(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pidRef,
                           PID::FuzzyPIDController& fuzzyRef, const MotorControlConfig& cfg)
    : encoder(enc),
      pid(pidRef),
      fuzzy(fuzzyRef),
      controlLaw(pidRef, fuzzyRef, buildControlLawConfig(cfg)),
      motionMonitor(buildMotionMonitorConfig(cfg)),
      actuator(drv, buildActuatorConfig(cfg)),
      controlTask(),
      config(cfg) {
    ESP_LOGI(TAG, "Ctor: creating mutexes");
    targetMutex = xSemaphoreCreateMutex();
    if (targetMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create target mutex");
    }

    configMutex = xSemaphoreCreateMutex();
    if (configMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create config mutex");
    }

    useFuzzy = true;

    controlTask.setUpdateCallback([this]() { this->update(); });

    syncComponentsFromConfig();
    ESP_LOGI(TAG, "Ctor done");
}

MotorControl::~MotorControl() {
    ESP_LOGI(TAG, "Dtor: stopping task and motor");
    stopTask();
    stop();
    if (targetMutex != nullptr) {
        vSemaphoreDelete(targetMutex);
        targetMutex = nullptr;
        ESP_LOGI(TAG, "Deleted target mutex");
    }
    if (configMutex != nullptr) {
        vSemaphoreDelete(configMutex);
        configMutex = nullptr;
        ESP_LOGI(TAG, "Deleted config mutex");
    }
    ESP_LOGI(TAG, "Dtor done");
}

MotorControl::MotorControl(MotorControl&& other) noexcept
    : encoder(other.encoder),
      pid(other.pid),
      fuzzy(other.fuzzy),
      controlLaw(other.pid, buildControlLawConfig(other.config)),
      motionMonitor(buildMotionMonitorConfig(other.config)),
      actuator(other.actuator),
      controlTask(),
      config(other.config) {
    ESP_LOGI(TAG, "Move Ctor start");
    useFuzzy = other.useFuzzy;
    target = other.target;
    motionDone.store(other.motionDone.load(std::memory_order_relaxed), std::memory_order_relaxed);
    lastPos = other.lastPos;
    lastPidOut = other.lastPidOut;
    lastVelDegPerSec = other.lastVelDegPerSec;

    targetMutex = other.targetMutex;
    configMutex = other.configMutex;

    onMotionDoneCb = other.onMotionDoneCb;
    onMotionDoneUser = other.onMotionDoneUser;
    onStallCb = other.onStallCb;
    onStallUser = other.onStallUser;
    onLimitHitCb = other.onLimitHitCb;
    onLimitHitUser = other.onLimitHitUser;

    lastStateLogUs = other.lastStateLogUs;
    lastStatusLogUs = other.lastStatusLogUs;

    other.targetMutex = nullptr;
    other.configMutex = nullptr;
    other.motionDone.store(true, std::memory_order_relaxed);

    other.onMotionDoneCb = nullptr;
    other.onMotionDoneUser = nullptr;
    other.onStallCb = nullptr;
    other.onStallUser = nullptr;
    other.onLimitHitCb = nullptr;
    other.onLimitHitUser = nullptr;

    other.lastStateLogUs = 0;
    other.lastStatusLogUs = 0;

    controlTask.setUpdateCallback([this]() { this->update(); });

    ESP_LOGI(TAG, "Move Ctor done");
}

void MotorControl::syncComponentsFromConfig() {
    controlLaw.setConfig(buildControlLawConfig(config));
    motionMonitor.setConfig(buildMotionMonitorConfig(config));
    actuator.setConfig(buildActuatorConfig(config));
}

void MotorControl::setTargetDegrees(float degrees) {
    ESP_LOGI(TAG, "setTargetDegrees: req=%.3f", degrees);

    degrees = motionMonitor.clampTarget(degrees);

    if (targetMutex != nullptr && xSemaphoreTake(targetMutex, portMAX_DELAY) == pdTRUE) {
        target = degrees;
        xSemaphoreGive(targetMutex);
    } else {
        target = degrees;
        ESP_LOGW(TAG, "setTargetDegrees without mutex");
    }

    controlLaw.reset();

    motionDone.store(false, std::memory_order_relaxed);
    lastPos = encoder.getPositionInDegrees();

    uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
    motionMonitor.startMotion(nowUs);

    actuator.resetSlewState();

    ESP_LOGI(TAG, "Target set: tgt=%.3f, pos=%.3f", target, lastPos);

    if (controlTask.isNotifyDriven()) {
        BaseType_t higher = pdFALSE;
        notifyControlTaskFromISR(&higher);
        portYIELD_FROM_ISR();
        ESP_LOGI(TAG, "Notified control task after target change");
    }
}

void MotorControl::setTargetTicks(int32_t ticks) {
    MotorControlConfig configSnapshot = getConfig();
    float degrees = (static_cast<float>(ticks) / static_cast<float>(configSnapshot.countsPerRevolution)) * 360.0f;
    ESP_LOGI(TAG, "setTargetTicks: ticks=%" PRId32 " -> deg=%.3f (CPR=%d)", ticks, degrees,
             configSnapshot.countsPerRevolution);
    setTargetDegrees(degrees);
}

void MotorControl::setTarget(float degrees) {
    setTargetDegrees(degrees);
}

void MotorControl::startTask() {
    controlTask.start();
}

void MotorControl::stopTask() {
    controlTask.stop();
}

void MotorControl::stop() {
    ESP_LOGI(TAG, "Stop called");
    actuator.stop();
    motionDone.store(true, std::memory_order_relaxed);
    controlLaw.reset();
    motionMonitor.reset();
}

void MotorControl::setUpdateHz(uint32_t hz) {
    controlTask.setUpdateHz(hz);
}

void MotorControl::setPID(float kp, float ki, float kd) {
    controlLaw.setPID(kp, ki, kd);
}

void MotorControl::getPID(float& kp, float& ki, float& kd) {
    controlLaw.getPID(kp, ki, kd);
}

void MotorControl::setConfig(const MotorControlConfig& newConfig) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config = newConfig;
        xSemaphoreGive(configMutex);
        ESP_LOGI(TAG, "Config updated");
    } else {
        config = newConfig;
        ESP_LOGW(TAG, "Config updated without mutex");
    }

    syncComponentsFromConfig();

    ESP_LOGI(TAG,
             "cfg: minSpd=%.1f maxSpd=%.1f minErr=%.3f driftDead=%.3f hyst=%.3f stuckEps=%.3f stuckN=%d "
             "pidWarmupN=%d CPR=%d timeoutMs=%d profile=%s type=%s accel=%.1f jerk=%.1f Kff_pos=%.3f "
             "Kff_vel=%.3f settlePos=%.3f settleVel=%.3f settleN=%d",
             config.minSpeed, config.maxSpeed, config.minErrorToMove, config.driftDeadband,
             config.driftHysteresis, config.stuckPositionEpsilon, config.stuckCountLimit,
             config.pidWarmupLimit, config.countsPerRevolution, config.motionTimeoutMs,
             config.motionProfileEnabled ? "ON" : "OFF",
             (config.motionProfileType == MotionProfileType::TRAPEZOID ? "TRAP" : "S"),
             config.accelLimitPctPerSec, config.jerkLimitPctPerSec2, config.Kff_pos, config.Kff_vel,
             config.settlePosTolDeg, config.settleVelTolDegPerSec, config.settleCountLimit);
}

MotorControlConfig MotorControl::getConfig() const {
    MotorControlConfig snapshot;
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        snapshot = config;
        xSemaphoreGive(configMutex);
    } else {
        snapshot = config;
        ESP_LOGW(TAG, "getConfig without mutex");
    }
    return snapshot;
}

void MotorControl::setMotionTimeoutMs(uint32_t ms) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.motionTimeoutMs = static_cast<int>(ms);
        xSemaphoreGive(configMutex);
    } else {
        config.motionTimeoutMs = static_cast<int>(ms);
        ESP_LOGW(TAG, "setMotionTimeoutMs without mutex");
    }
    motionMonitor.setMotionTimeoutMs(ms);
    ESP_LOGI(TAG, "Motion timeout set: %u ms", static_cast<unsigned>(ms));
}

uint32_t MotorControl::getMotionTimeoutMs() const {
    return motionMonitor.getMotionTimeoutMs();
}

void MotorControl::enableMotionProfile(bool enable) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.motionProfileEnabled = enable;
        xSemaphoreGive(configMutex);
    } else {
        config.motionProfileEnabled = enable;
        ESP_LOGW(TAG, "enableMotionProfile without mutex");
    }
    controlLaw.enableMotionProfile(enable);
}

void MotorControl::configureMotionProfile(MotionProfileType type, float accelPctPerSec, float jerkPctPerSec2) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.motionProfileType = type;
        config.accelLimitPctPerSec = accelPctPerSec;
        config.jerkLimitPctPerSec2 = jerkPctPerSec2;
        xSemaphoreGive(configMutex);
    } else {
        config.motionProfileType = type;
        config.accelLimitPctPerSec = accelPctPerSec;
        config.jerkLimitPctPerSec2 = jerkPctPerSec2;
        ESP_LOGW(TAG, "configureMotionProfile without mutex");
    }
    controlLaw.configureMotionProfile(type, accelPctPerSec, jerkPctPerSec2);

    actuator.setSlewRate(accelPctPerSec);
}

void MotorControl::setFeedForward(float kpos, float kvel) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.Kff_pos = kpos;
        config.Kff_vel = kvel;
        xSemaphoreGive(configMutex);
    } else {
        config.Kff_pos = kpos;
        config.Kff_vel = kvel;
        ESP_LOGW(TAG, "setFeedForward without mutex");
    }
    controlLaw.setFeedForward(kpos, kvel);
}

void MotorControl::getFeedForward(float& kpos, float& kvel) const {
    controlLaw.getFeedForward(kpos, kvel);
}

void MotorControl::setPidWarmupCycles(int cycles) {
    if (cycles < 0) {
        cycles = 0;
    }
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.pidWarmupLimit = cycles;
        xSemaphoreGive(configMutex);
    } else {
        config.pidWarmupLimit = cycles;
        ESP_LOGW(TAG, "setPidWarmupCycles without mutex");
    }
    motionMonitor.setPidWarmupCycles(cycles);
}

int MotorControl::getPidWarmupCycles() const {
    return motionMonitor.getPidWarmupCycles();
}

void MotorControl::configureControlTask(int coreId, UBaseType_t priority) {
    controlTask.configureTask(coreId, priority);
}

int MotorControl::getControlTaskCore() const {
    return controlTask.getCoreId();
}

UBaseType_t MotorControl::getControlTaskPriority() const {
    return controlTask.getPriority();
}

void MotorControl::enableNotifyDrivenUpdates(bool enable, TickType_t maxBlockTicks) {
    controlTask.enableNotifyDrivenUpdates(enable, maxBlockTicks);

    if (enable && controlTask.isRunning()) {
        BaseType_t higher = pdFALSE;
        notifyControlTaskFromISR(&higher);
        portYIELD_FROM_ISR();
        ESP_LOGI(TAG, "Kicked control task after enabling notify mode");
    }
}

void MotorControl::notifyControlTaskFromISR(BaseType_t* higherPrioTaskWoken) {
    controlTask.notifyFromISR(higherPrioTaskWoken);
}

void MotorControl::setOnMotionDone(MotionEventCallback cb, void* user) {
    onMotionDoneCb = cb;
    onMotionDoneUser = user;
    ESP_LOGI(TAG, "onMotionDone callback %s", cb != nullptr ? "SET" : "CLEARED");
}

void MotorControl::setOnStall(MotionEventCallback cb, void* user) {
    onStallCb = cb;
    onStallUser = user;
    ESP_LOGI(TAG, "onStall callback %s", cb != nullptr ? "SET" : "CLEARED");
}

void MotorControl::setOnLimitHit(MotionEventCallback cb, void* user) {
    onLimitHitCb = cb;
    onLimitHitUser = user;
    ESP_LOGI(TAG, "onLimitHit callback %s", cb != nullptr ? "SET" : "CLEARED");
}

bool MotorControl::isMotionDone() const {
    return motionDone.load(std::memory_order_relaxed);
}

float MotorControl::getTargetThreadSafe() const {
    float currentTarget;
    if (targetMutex == nullptr) {
        ESP_LOGE(TAG, "update: target mutex is null, using cached value");
        currentTarget = target;
    } else if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentTarget = target;
        xSemaphoreGive(targetMutex);
    } else {
        ESP_LOGW(TAG, "update: target mutex timeout, using cached value");
        currentTarget = target;
    }
    return currentTarget;
}

void MotorControl::update() {
    uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());

    auto shouldLog = [](uint64_t& lastLog, uint64_t now, uint32_t intervalMs) {
        if (now - lastLog >= static_cast<uint64_t>(intervalMs) * 1000ULL) {
            lastLog = now;
            return true;
        }
        return false;
    };

    float currentTarget = getTargetThreadSafe();
    float currentPos = encoder.getPositionInDegrees();

    if (shouldLog(lastStateLogUs, nowUs, 500)) {
        ESP_LOGD(TAG, "update: t=%llu us, tgt=%.3f pos=%.3f", (unsigned long long)nowUs, currentTarget,
                 currentPos);
    }

    if (motionDone.load(std::memory_order_relaxed)) {
        handleIdleState(currentTarget, currentPos, nowUs);
        return;
    }

    float dpos = currentPos - lastPos;
    float velocity = dpos * static_cast<float>(controlTask.getUpdateHz());
    lastVelDegPerSec = velocity;

    ControlOutput ctrl = controlLaw.compute(currentTarget, currentPos, velocity, nowUs);
    lastPidOut = ctrl.rawPidOutput;

    MotionObservation obs{};
    obs.error = currentTarget - currentPos;
    obs.velocity = velocity;
    obs.deltaPosition = dpos;
    obs.position = currentPos;
    obs.commandPct = ctrl.commandPct;
    obs.nowUs = nowUs;
    obs.pidSettled = ctrl.settled;

    MotionEvent event = motionMonitor.evaluate(obs);

    if (event != MotionEvent::NONE) {
        handleMotionEvent(event, currentTarget, currentPos);
        lastPos = currentPos;
        return;
    }

    actuator.drive(ctrl.commandPct, nowUs);
    lastPos = currentPos;

    if (shouldLog(lastStatusLogUs, nowUs, 1000)) {
        MotorStatus s = getStatus();
        logMotorStatus(s);
    }
}

void MotorControl::handleIdleState(float currentTarget, float currentPos, uint64_t nowUs) {
    if (motionMonitor.shouldWakeFromIdle(currentTarget, currentPos)) {
        setTargetDegrees(currentTarget);
    }
}

void MotorControl::handleMotionEvent(MotionEvent event, float currentTarget, float currentPos) {
    actuator.stop();

    switch (event) {
        case MotionEvent::SETTLED:
            ESP_LOGI(TAG, "motion DONE: settled");
            motionDone.store(true, std::memory_order_relaxed);
            if (onMotionDoneCb) {
                MotorStatus s = getStatus();
                onMotionDoneCb(s, onMotionDoneUser);
            }
            break;

        case MotionEvent::STALLED: {
            float finalError = currentTarget - currentPos;
            bool doneOk = motionMonitor.isWithinDriftDeadband(finalError);
            motionDone.store(doneOk, std::memory_order_relaxed);
            if (onStallCb) {
                MotorStatus s = getStatus();
                onStallCb(s, onStallUser);
            }
            break;
        }

        case MotionEvent::TIMEOUT:
            ESP_LOGW(TAG, "motion timeout");
            motionDone.store(true, std::memory_order_relaxed);
            if (onStallCb) {
                MotorStatus s = getStatus();
                onStallCb(s, onStallUser);
            }
            break;

        case MotionEvent::LIMIT_HIT:
            ESP_LOGW(TAG, "limit hit at %.3f deg", currentPos);
            motionDone.store(true, std::memory_order_relaxed);
            if (onLimitHitCb) {
                MotorStatus s = getStatus();
                onLimitHitCb(s, onLimitHitUser);
            }
            break;

        case MotionEvent::NONE:
            break;
    }

    controlLaw.reset();
    motionMonitor.reset();
}

MotorStatus MotorControl::getStatus() const {
    MotorStatus status{};

    float currentTarget = getTargetThreadSafe();
    float currentPos = encoder.getPositionInDegrees();

    status.target = currentTarget;
    status.position = currentPos;
    status.error = currentTarget - currentPos;
    status.velocity = lastVelDegPerSec;
    status.pidOutput = lastPidOut;
    status.stuckCount = motionMonitor.getStuckCount();
    status.motionDone = motionDone.load(std::memory_order_relaxed);
    return status;
}

void MotorControl::logMotorStatus(const MotorStatus& status) {
    ESP_LOGI(TAG, "=== Motor Status ===");
    ESP_LOGI(TAG, "Target: %.3f°, Position: %.3f°, Error: %.3f°", status.target, status.position,
             status.error);
    ESP_LOGI(TAG, "Velocity: %.3f°/s, PID Output: %.3f", status.velocity, status.pidOutput);
    ESP_LOGI(TAG, "Stuck Count: %d, Motion Done: %s", status.stuckCount, status.motionDone ? "YES" : "NO");
    ESP_LOGI(TAG, "====================");
}

void MotorControl::setSoftLimits(float minDeg, float maxDeg, bool enforce) {
    ESP_LOGI(TAG, "setSoftLimits: req[%.3f, %.3f], enforce=%d", minDeg, maxDeg, static_cast<int>(enforce));

    motionMonitor.setSoftLimits(minDeg, maxDeg, enforce);

    if (motionMonitor.areSoftLimitsEnforced()) {
        float clamped = motionMonitor.clampTarget(target);
        if (fabsf(clamped - target) > 1e-6f) {
            ESP_LOGW(TAG, "Target clamped to %.3f due to soft limits", clamped);
            setTargetDegrees(clamped);
        }
    }

    ESP_LOGI(TAG, "Soft limits set: [%.3f°, %.3f°], %s",
             motionMonitor.getSoftLimitMin(), motionMonitor.getSoftLimitMax(),
             motionMonitor.areSoftLimitsEnforced() ? "ENFORCED" : "NOT ENFORCED");
}
