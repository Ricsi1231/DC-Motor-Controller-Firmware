#include "MotorControl.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>

using namespace DC_Motor_Controller_Firmware;
using namespace Control;

MotorControl::MotorControl(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pidRef,
                           const MotorControlConfig& initialConfig)
    : encoder(enc), motor(drv), pid(pidRef), fuzzy(PID::FuzzyPidConfig{}), config(initialConfig) {
    ESP_LOGI(TAG, "Ctor: creating mutexes");
    targetMutex = xSemaphoreCreateMutex();
    if (targetMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create target mutex");
    }

    configMutex = xSemaphoreCreateMutex();
    if (configMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create config mutex");
    }

    taskExitSemaphore = xSemaphoreCreateBinary();
    if (taskExitSemaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create task exit semaphore");
    }

    syncComponentsFromConfig();
    ESP_LOGI(TAG, "Ctor done");
}

MotorControl::MotorControl(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pidRef,
                           PID::FuzzyPIDController& fuzzyRef, const MotorControlConfig& cfg)
    : encoder(enc), motor(drv), pid(pidRef), fuzzy(fuzzyRef), config(cfg) {
    ESP_LOGI(TAG, "Ctor: creating mutexes");
    targetMutex = xSemaphoreCreateMutex();
    if (targetMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create target mutex");
    }

    configMutex = xSemaphoreCreateMutex();
    if (configMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create config mutex");
    }

    taskExitSemaphore = xSemaphoreCreateBinary();
    if (taskExitSemaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create task exit semaphore");
    }

    useFuzzy = true;
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
    if (taskExitSemaphore != nullptr) {
        vSemaphoreDelete(taskExitSemaphore);
        taskExitSemaphore = nullptr;
        ESP_LOGI(TAG, "Deleted task exit semaphore");
    }
    ESP_LOGI(TAG, "Dtor done");
}

MotorControl::MotorControl(MotorControl&& other) noexcept
    : encoder(other.encoder),
      motor(other.motor),
      pid(other.pid),
      fuzzy(other.fuzzy),
      config(other.config),
      profile(other.profile),
      detector(other.detector),
      limits(other.limits) {
    ESP_LOGI(TAG, "Move Ctor start");
    useFuzzy = other.useFuzzy;
    target = other.target;
    motionDone.store(other.motionDone.load(std::memory_order_relaxed), std::memory_order_relaxed);
    lastPos = other.lastPos;
    updateHz = other.updateHz;
    lastPidOut = other.lastPidOut;
    lastVelDegPerSec = other.lastVelDegPerSec;

    targetMutex = other.targetMutex;
    configMutex = other.configMutex;
    taskHandle = other.taskHandle;
    taskStopRequested = other.taskStopRequested;
    taskExitSemaphore = other.taskExitSemaphore;

    motionStartUs = other.motionStartUs;

    controlTaskCoreId = other.controlTaskCoreId;
    controlTaskPriority = other.controlTaskPriority;

    notifyDriven = other.notifyDriven;
    notifyBlockTicks = other.notifyBlockTicks;

    onMotionDoneCb = other.onMotionDoneCb;
    onMotionDoneUser = other.onMotionDoneUser;
    onStallCb = other.onStallCb;
    onStallUser = other.onStallUser;
    onLimitHitCb = other.onLimitHitCb;
    onLimitHitUser = other.onLimitHitUser;

    lastPidLogUs = other.lastPidLogUs;
    lastStateLogUs = other.lastStateLogUs;
    lastCmdLogUs = other.lastCmdLogUs;
    lastStatusLogUs = other.lastStatusLogUs;
    slewOutPct = other.slewOutPct;
    lastSlewUs = other.lastSlewUs;
    clampPrevValue = other.clampPrevValue;

    other.targetMutex = nullptr;
    other.configMutex = nullptr;
    other.taskHandle = nullptr;
    other.taskStopRequested = false;
    other.taskExitSemaphore = nullptr;
    other.motionDone.store(true, std::memory_order_relaxed);
    other.motionStartUs = 0;

    other.onMotionDoneCb = nullptr;
    other.onMotionDoneUser = nullptr;
    other.onStallCb = nullptr;
    other.onStallUser = nullptr;
    other.onLimitHitCb = nullptr;
    other.onLimitHitUser = nullptr;

    other.lastPidLogUs = 0;
    other.lastStateLogUs = 0;
    other.lastCmdLogUs = 0;
    other.lastStatusLogUs = 0;
    other.slewOutPct = 0.0f;
    other.lastSlewUs = 0;
    other.clampPrevValue = NAN;

    ESP_LOGI(TAG, "Move Ctor done");
}

void MotorControl::syncComponentsFromConfig() {
    profile.configure(config.motionProfileType, config.accelLimitPctPerSec, config.jerkLimitPctPerSec2);
    profile.setEnabled(config.motionProfileEnabled);
    profile.setMaxSpeed(config.maxSpeed);

    MotionDetector::Config detectorCfg;
    detectorCfg.minErrorToMove = config.minErrorToMove;
    detectorCfg.stuckPositionEpsilon = config.stuckPositionEpsilon;
    detectorCfg.stuckCountLimit = config.stuckCountLimit;
    detectorCfg.pidWarmupLimit = config.pidWarmupLimit;
    detectorCfg.settlePosTolDeg = config.settlePosTolDeg;
    detectorCfg.settleVelTolDegPerSec = config.settleVelTolDegPerSec;
    detectorCfg.settleCountLimit = config.settleCountLimit;
    detector.setConfig(detectorCfg);
}

void MotorControl::setTargetDegrees(float degrees) {
    ESP_LOGI(TAG, "setTargetDegrees: req=%.3f", degrees);

    degrees = limits.clampTarget(degrees);

    if (targetMutex != nullptr && xSemaphoreTake(targetMutex, portMAX_DELAY) == pdTRUE) {
        target = degrees;
        xSemaphoreGive(targetMutex);
    } else {
        target = degrees;
        ESP_LOGW(TAG, "setTargetDegrees without mutex");
    }

    if (useFuzzy) {
        fuzzy.reset();
    } else {
        pid.reset();
    }

    motionDone.store(false, std::memory_order_relaxed);
    lastPos = encoder.getPositionInDegrees();
    detector.startMotion();
    motionStartUs = static_cast<uint64_t>(esp_timer_get_time());

    profile.reset();

    ESP_LOGI(TAG, "Target set: tgt=%.3f, pos=%.3f", target, lastPos);

    if (notifyDriven == true) {
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
    if (taskHandle == nullptr) {
        UBaseType_t priorityToUse = controlTaskPriority;
        if (priorityToUse < 1) {
            priorityToUse = 1;
        }

        BaseType_t coreToUse = controlTaskCoreId;
        if (coreToUse != 0 && coreToUse != 1 && coreToUse != tskNO_AFFINITY) {
            coreToUse = tskNO_AFFINITY;
        }

        ESP_LOGI(TAG, "Starting control task (core=%ld, prio=%u, mode=%s, stack=%u, hz=%u)",
                 static_cast<long>(coreToUse), static_cast<unsigned>(priorityToUse),
                 notifyDriven ? "notify" : "poll", 4096u, static_cast<unsigned>(updateHz));

        BaseType_t res = xTaskCreatePinnedToCore(taskFunc, "MotorControlTask", 4096, this, priorityToUse,
                                                 &taskHandle, coreToUse);

        if (res != pdPASS) {
            ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
            taskHandle = nullptr;
        } else {
            ESP_LOGI(TAG, "Control task started");
        }
    } else {
        ESP_LOGW(TAG, "startTask: task already running");
    }
}

void MotorControl::stopTask() {
    if (taskHandle != nullptr) {
        ESP_LOGI(TAG, "Requesting control task to stop");
        taskStopRequested = true;

        if (notifyDriven && taskHandle != nullptr) {
            xTaskNotifyGive(taskHandle);
        }

        if (taskExitSemaphore != nullptr) {
            BaseType_t taken = xSemaphoreTake(taskExitSemaphore, pdMS_TO_TICKS(2000));
            if (taken != pdTRUE) {
                ESP_LOGW(TAG, "Task did not exit in time, forcing delete");
                vTaskDelete(taskHandle);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
            vTaskDelete(taskHandle);
        }

        taskHandle = nullptr;
        taskStopRequested = false;
        ESP_LOGI(TAG, "Control task stopped");
    } else {
        ESP_LOGW(TAG, "stopTask: no task");
    }
}

void MotorControl::stop() {
    ESP_LOGI(TAG, "Stop called");
    motor.stop();
    motionDone.store(true, std::memory_order_relaxed);
    motionStartUs = 0;
    profile.reset();
    detector.reset();
}

void MotorControl::setUpdateHz(uint32_t hz) {
    if (hz > 0) {
        updateHz = hz;
        ESP_LOGI(TAG, "UpdateHz set to %u", static_cast<unsigned>(hz));
    } else {
        ESP_LOGW(TAG, "Invalid UpdateHz %u", static_cast<unsigned>(hz));
    }
}

void MotorControl::taskFunc(void* param) {
    auto* self = static_cast<MotorControl*>(param);

    TickType_t lastWake = xTaskGetTickCount();
    uint32_t localHz = self->updateHz;
    if (localHz == 0) {
        localHz = 1;
    }

    TickType_t period = pdMS_TO_TICKS(1000 / localHz);
    if (period < 1) {
        period = 1;
    }

    ESP_LOGI(TAG, "Task loop start (mode=%s, periodTicks=%lu)", self->notifyDriven ? "notify" : "poll",
             static_cast<unsigned long>(period));

    while (!self->taskStopRequested) {
        if (self->notifyDriven == true) {
            uint32_t taken = ulTaskNotifyTake(pdTRUE, self->notifyBlockTicks);
            if (taken == 0) {
                continue;
            }
            if (self->taskStopRequested) {
                break;
            }
            self->update();
        } else {
            self->update();
            vTaskDelayUntil(&lastWake, period);
        }
    }

    ESP_LOGI(TAG, "Task loop exiting");
    if (self->taskExitSemaphore != nullptr) {
        xSemaphoreGive(self->taskExitSemaphore);
    }
    vTaskDelete(nullptr);
}

void MotorControl::setPID(float kp, float ki, float kd) {
    pid.setParameters(kp, ki, kd);
    ESP_LOGI(TAG, "PID set: Kp=%.4f Ki=%.4f Kd=%.4f", kp, ki, kd);
}

void MotorControl::getPID(float& kp, float& ki, float& kd) {
    if (useFuzzy) {
        fuzzy.getCurrentGains(kp, ki, kd);
    } else {
        pid.getParameters(kp, ki, kd);
    }
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
    ESP_LOGI(TAG, "Motion timeout set: %u ms", static_cast<unsigned>(ms));
}

uint32_t MotorControl::getMotionTimeoutMs() const {
    uint32_t value = 0;
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        value = static_cast<uint32_t>(config.motionTimeoutMs);
        xSemaphoreGive(configMutex);
    } else {
        value = static_cast<uint32_t>(config.motionTimeoutMs);
        ESP_LOGW(TAG, "getMotionTimeoutMs without mutex");
    }
    return value;
}

void MotorControl::enableMotionProfile(bool enable) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.motionProfileEnabled = enable;
        xSemaphoreGive(configMutex);
    } else {
        config.motionProfileEnabled = enable;
        ESP_LOGW(TAG, "enableMotionProfile without mutex");
    }
    profile.setEnabled(enable);
    ESP_LOGI(TAG, "Profile %s", enable ? "ENABLED" : "DISABLED");
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
    profile.configure(type, accelPctPerSec, jerkPctPerSec2);
    ESP_LOGI(TAG, "Profile cfg: type=%s accel=%.1f jerk=%.1f",
             type == MotionProfileType::TRAPEZOID ? "TRAP" : "S", accelPctPerSec, jerkPctPerSec2);
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
    ESP_LOGI(TAG, "FF set: Kff_pos=%.4f Kff_vel=%.4f", kpos, kvel);
}

void MotorControl::getFeedForward(float& kpos, float& kvel) const {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        kpos = config.Kff_pos;
        kvel = config.Kff_vel;
        xSemaphoreGive(configMutex);
    } else {
        kpos = config.Kff_pos;
        kvel = config.Kff_vel;
        ESP_LOGW(TAG, "getFeedForward without mutex");
    }
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

    MotionDetector::Config detectorCfg;
    detectorCfg.pidWarmupLimit = cycles;
    detector.setConfig(detectorCfg);

    ESP_LOGI(TAG, "PID warmup cycles=%d", cycles);
}

int MotorControl::getPidWarmupCycles() const {
    int value = 0;
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        value = config.pidWarmupLimit;
        xSemaphoreGive(configMutex);
    } else {
        value = config.pidWarmupLimit;
        ESP_LOGW(TAG, "getPidWarmupCycles without mutex");
    }
    return value;
}

void MotorControl::configureControlTask(int coreId, UBaseType_t priority) {
    if (priority < 1) {
        priority = 1;
    }

    BaseType_t coreCandidate = static_cast<BaseType_t>(coreId);
    if (coreCandidate != 0 && coreCandidate != 1 && coreCandidate != tskNO_AFFINITY) {
        coreCandidate = tskNO_AFFINITY;
    }

    controlTaskPriority = priority;
    controlTaskCoreId = coreCandidate;

    ESP_LOGI(TAG, "Control task config: core=%ld priority=%u", static_cast<long>(controlTaskCoreId),
             static_cast<unsigned>(controlTaskPriority));
}

int MotorControl::getControlTaskCore() const {
    return static_cast<int>(controlTaskCoreId);
}

UBaseType_t MotorControl::getControlTaskPriority() const {
    return controlTaskPriority;
}

void MotorControl::enableNotifyDrivenUpdates(bool enable, TickType_t maxBlockTicks) {
    notifyDriven = enable;
    notifyBlockTicks = maxBlockTicks;
    ESP_LOGI(TAG, "Notify-driven updates %s, blockTicks=%lu", notifyDriven ? "ENABLED" : "DISABLED",
             static_cast<unsigned long>(notifyBlockTicks));

    if (notifyDriven == true && taskHandle != nullptr) {
        BaseType_t higher = pdFALSE;
        notifyControlTaskFromISR(&higher);
        portYIELD_FROM_ISR();
        ESP_LOGI(TAG, "Kicked control task after enabling notify mode");
    }
}

void MotorControl::notifyControlTaskFromISR(BaseType_t* higherPrioTaskWoken) {
    if (taskHandle != nullptr) {
        vTaskNotifyGiveFromISR(taskHandle, higherPrioTaskWoken);
    }
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

float MotorControl::clampToPercentRange(float signal) const {
    float clampedValue = signal;

    if (!std::isfinite(clampedValue)) {
        if (clampedValue != clampPrevValue) {
            ESP_LOGW(TAG, "clamp: non-finite signal -> 0");
        }
        clampPrevValue = 0.0f;
        return 0.0f;
    }

    if (clampedValue > 100.0f) {
        if (clampPrevValue != 100.0f) {
            ESP_LOGW(TAG, "clamp: %.3f -> 100.0", clampedValue);
        }
        clampPrevValue = 100.0f;
        return 100.0f;
    }

    if (clampedValue < -100.0f) {
        if (clampPrevValue != -100.0f) {
            ESP_LOGW(TAG, "clamp: %.3f -> -100.0", clampedValue);
        }
        clampPrevValue = -100.0f;
        return -100.0f;
    }

    clampPrevValue = clampedValue;
    return clampedValue;
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

    float currentPos = encoder.getPositionInDegrees();
    if (shouldLog(lastStateLogUs, nowUs, 500)) {
        ESP_LOGD(TAG, "update: t=%llu us, tgt=%.3f pos=%.3f", (unsigned long long)nowUs, currentTarget,
                 currentPos);
    }

    if (motionDone.load(std::memory_order_relaxed)) {
        handleIdleState(currentTarget, currentPos, nowUs);
        return;
    }

    MotorControlConfig cfg = getConfig();

    if (checkTimeout(cfg, nowUs)) {
        return;
    }

    float combined = computeControl(cfg, currentTarget, currentPos, nowUs);

    if (checkLimits(combined, currentPos)) {
        return;
    }

    float profiled = applySpeedLimits(cfg, combined, currentTarget, currentPos, nowUs);

    float dpos = currentPos - lastPos;
    float vel = dpos * static_cast<float>(updateHz);
    lastVelDegPerSec = vel;

    detector.update(currentTarget - currentPos, vel, dpos);

    if (detector.isSettled()) {
        handleSettled();
        lastPos = currentPos;
        return;
    }

    if (detector.isStuck() || checkPidSettled(cfg)) {
        handleStall(cfg, currentTarget, currentPos);
        lastPos = currentPos;
        return;
    }

    applyMotorCommand(cfg, profiled, nowUs);
    lastPos = currentPos;

    if (shouldLog(lastStatusLogUs, nowUs, 1000)) {
        MotorStatus s = getStatus();
        logMotorStatus(s);
    }
}

void MotorControl::handleIdleState(float currentTarget, float currentPos, uint64_t nowUs) {
    MotorControlConfig cfg = getConfig();
    float drift = currentTarget - currentPos;
    float wakeTh = cfg.driftDeadband + cfg.driftHysteresis;
    if (wakeTh < 0.0f) {
        wakeTh = 0.0f;
    }

    if (fabsf(drift) > wakeTh) {
        setTargetDegrees(currentTarget);
    }
}

bool MotorControl::checkTimeout(const MotorControlConfig& cfg, uint64_t nowUs) {
    if (cfg.motionTimeoutMs <= 0) {
        return false;
    }

    if (motionStartUs == 0) {
        motionStartUs = nowUs;
    }

    uint64_t elapsedUs = nowUs - motionStartUs;
    uint64_t limitUs = static_cast<uint64_t>(cfg.motionTimeoutMs) * 1000ULL;

    if (elapsedUs > limitUs) {
        ESP_LOGW(TAG, "timeout: %llu ms > %u ms", (unsigned long long)(elapsedUs / 1000ULL),
                 static_cast<unsigned int>(cfg.motionTimeoutMs));
        motor.stop();
        motionDone.store(true, std::memory_order_relaxed);
        if (onStallCb) {
            MotorStatus s = getStatus();
            onStallCb(s, onStallUser);
        }
        motionStartUs = 0;
        profile.reset();
        detector.reset();
        return true;
    }
    return false;
}

float MotorControl::computeControl(const MotorControlConfig& cfg, float currentTarget, float currentPos,
                                   uint64_t nowUs) {
    float pidSignal = 0;
    if (useFuzzy) {
        pidSignal = fuzzy.compute(currentTarget, currentPos);
    } else {
        pidSignal = pid.compute(currentTarget, currentPos);
    }
    pidSignal = clampToPercentRange(pidSignal);
    lastPidOut = pidSignal;

    float refVelPct = profile.isEnabled() ? 0.0f : pidSignal;
    float ff = cfg.Kff_pos * currentTarget + cfg.Kff_vel * refVelPct;

    return clampToPercentRange(pidSignal + ff);
}

bool MotorControl::checkLimits(float combined, float currentPos) {
    if (!limits.isEnforced()) {
        return false;
    }

    if (limits.isAtLimit(currentPos, combined)) {
        ESP_LOGW(TAG, "limit: at %.3f, stopping", currentPos);
        motor.stop();
        motionDone.store(true, std::memory_order_relaxed);
        if (onLimitHitCb) {
            MotorStatus s = getStatus();
            onLimitHitCb(s, onLimitHitUser);
        }
        profile.reset();
        detector.reset();
        return true;
    }
    return false;
}

float MotorControl::applySpeedLimits(const MotorControlConfig& cfg, float combined, float currentTarget,
                                     float currentPos, uint64_t nowUs) {
    float errAbs = fabsf(currentTarget - currentPos);
    float mag = fabsf(combined);

    if (mag < cfg.minSpeed && errAbs > cfg.minErrorToMove) {
        mag = cfg.minSpeed;
    }
    if (mag > cfg.maxSpeed) {
        mag = cfg.maxSpeed;
    }

    float signedDesired = (combined >= 0.0f) ? mag : -mag;
    return profile.shape(signedDesired, nowUs);
}

bool MotorControl::checkPidSettled(const MotorControlConfig& cfg) {
    if (useFuzzy) {
        return fuzzy.isSettled();
    }
    return pid.isSettled();
}

void MotorControl::handleSettled() {
    ESP_LOGI(TAG, "motion DONE: settled");
    motor.stop();
    motionDone.store(true, std::memory_order_relaxed);
    if (onMotionDoneCb) {
        MotorStatus s = getStatus();
        onMotionDoneCb(s, onMotionDoneUser);
    }
    motionStartUs = 0;
    profile.reset();
    detector.reset();
}

void MotorControl::handleStall(const MotorControlConfig& cfg, float currentTarget, float currentPos) {
    motor.stop();
    float finalDriftAbs = fabsf(currentTarget - currentPos);
    bool doneOk = (finalDriftAbs <= cfg.driftDeadband);
    motionDone.store(doneOk, std::memory_order_relaxed);
    if (onStallCb) {
        MotorStatus s = getStatus();
        onStallCb(s, onStallUser);
    }
    motionStartUs = 0;
    profile.reset();
    detector.reset();
}

void MotorControl::applyMotorCommand(const MotorControlConfig& cfg, float profiled, uint64_t nowUs) {
    if (lastSlewUs == 0) {
        lastSlewUs = nowUs;
        slewOutPct = 0.0f;
    }

    double dt = static_cast<double>(nowUs - lastSlewUs) / 1e6;
    if (dt < 0.0) {
        dt = 0.0;
    }

    double maxDelta = static_cast<double>(cfg.accelLimitPctPerSec) * dt;
    double reqDelta = static_cast<double>(profiled) - static_cast<double>(slewOutPct);

    if (reqDelta > maxDelta) {
        reqDelta = maxDelta;
    }
    if (reqDelta < -maxDelta) {
        reqDelta = -maxDelta;
    }

    float slewOut = static_cast<float>(static_cast<double>(slewOutPct) + reqDelta);
    slewOutPct = slewOut;
    lastSlewUs = nowUs;

    float speedPct = fabsf(slewOut);
    if (speedPct > cfg.maxSpeed) {
        speedPct = cfg.maxSpeed;
    }

    DRV8876::Direction dir = (slewOut >= 0.0f) ? DRV8876::Direction::RIGHT : DRV8876::Direction::LEFT;
    motor.setDirection(dir);
    motor.setSpeed(speedPct);

    auto shouldLog = [](uint64_t& lastLog, uint64_t now, uint32_t intervalMs) {
        if (now - lastLog >= static_cast<uint64_t>(intervalMs) * 1000ULL) {
            lastLog = now;
            return true;
        }
        return false;
    };

    if (shouldLog(lastCmdLogUs, nowUs, 250)) {
        const char* dirStr = (slewOut >= 0.0f) ? "RIGHT" : "LEFT";
        ESP_LOGI(TAG, "cmd: dir=%s speed=%.2f%%", dirStr, speedPct);
    }
}

MotorStatus MotorControl::getStatus() const {
    MotorStatus status{};

    float currentTarget;
    if (targetMutex != nullptr && xSemaphoreTake(targetMutex, portMAX_DELAY) == pdTRUE) {
        currentTarget = target;
        xSemaphoreGive(targetMutex);
    } else {
        currentTarget = target;
    }

    float currentPos = encoder.getPositionInDegrees();

    status.target = currentTarget;
    status.position = currentPos;
    status.error = currentTarget - currentPos;
    status.velocity = lastVelDegPerSec;
    status.pidOutput = lastPidOut;
    status.stuckCount = detector.getStuckCount();
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

    limits.set(minDeg, maxDeg, enforce);

    if (limits.isEnforced()) {
        float clamped = limits.clampTarget(target);
        if (fabsf(clamped - target) > 1e-6f) {
            ESP_LOGW(TAG, "Target clamped to %.3f due to soft limits", clamped);
            setTargetDegrees(clamped);
        }
    }

    ESP_LOGI(TAG, "Soft limits set: [%.3f°, %.3f°], %s", limits.getMin(), limits.getMax(),
             limits.isEnforced() ? "ENFORCED" : "NOT ENFORCED");
}
