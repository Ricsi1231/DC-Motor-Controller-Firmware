#include "motorControl.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>

using namespace DC_Motor_Controller_Firmware;
using namespace Control;

MotorController::MotorController(const DRV8876::DRV8876Config& motorDriverConfig, const Encoder::EncoderConfig& encoderConfig, const PID::PidConfig& pidConfig,
                                 const MotorControllerConfig& controllerConfig)
    : motor(std::make_unique<DRV8876::DRV8876>(motorDriverConfig)),
      encoder(std::make_unique<Encoder::Encoder>(encoderConfig)),
      pid(std::make_unique<PID::PIDController>(pidConfig)),
      config(controllerConfig) {
    ESP_LOGI(TAG, "Ctor: creating mutexes");
    targetMutex = xSemaphoreCreateMutex();
    if (targetMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create target mutex");
    }

    configMutex = xSemaphoreCreateMutex();
    if (configMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create config mutex");
    }

    ESP_LOGI(TAG, "Ctor done");
}

MotorController::~MotorController() {
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

MotorController::MotorController(MotorController&& other) noexcept
    : motor(std::move(other.motor)),
      encoder(std::move(other.encoder)),
      pid(std::move(other.pid)),
      config(other.config),
      profiler(other.profiler),
      settleDetector(other.settleDetector),
      stallDetector(other.stallDetector),
      softLimiter(other.softLimiter),
      motionGuard(other.motionGuard) {
    ESP_LOGI(TAG, "Move Ctor start");
    target = other.target;
    motionDone.store(other.motionDone.load(std::memory_order_relaxed), std::memory_order_relaxed);
    lastPos = other.lastPos;
    updateHz = other.updateHz;
    lastPidOut = other.lastPidOut;
    lastVelDegPerSec = other.lastVelDegPerSec;

    targetMutex = other.targetMutex;
    configMutex = other.configMutex;
    taskHandle = other.taskHandle;

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

    other.targetMutex = nullptr;
    other.configMutex = nullptr;
    other.taskHandle = nullptr;
    other.motionDone.store(true, std::memory_order_relaxed);

    other.onMotionDoneCb = nullptr;
    other.onMotionDoneUser = nullptr;
    other.onStallCb = nullptr;
    other.onStallUser = nullptr;
    other.onLimitHitCb = nullptr;
    other.onLimitHitUser = nullptr;

    ESP_LOGI(TAG, "Move Ctor done");
}

esp_err_t MotorController::init() {
    esp_err_t errorStatus = motor->init();
    if (errorStatus != ESP_OK) {
        ESP_LOGE(TAG, "Motor init failed: %s", esp_err_to_name(errorStatus));
        return errorStatus;
    }

    errorStatus = encoder->init();
    if (errorStatus != ESP_OK) {
        ESP_LOGE(TAG, "Encoder init failed: %s", esp_err_to_name(errorStatus));
        return errorStatus;
    }

    errorStatus = encoder->start();
    if (errorStatus != ESP_OK) {
        ESP_LOGE(TAG, "Encoder start failed: %s", esp_err_to_name(errorStatus));
        return errorStatus;
    }

    ESP_LOGI(TAG, "Init done");
    return ESP_OK;
}

IEncoder* MotorController::getEncoder() const { return encoder.get(); }

IMotorDriver* MotorController::getMotor() const { return motor.get(); }

IPIDController* MotorController::getPid() const { return pid.get(); }

IMotorController* MotorController::getMotorControl() { return this; }

void MotorController::resetMotionState(uint64_t nowUs) {
    profiler.reset(nowUs);
    settleDetector.reset();
    stallDetector.reset();
    motionGuard.reset();
}

void MotorController::setTargetDegrees(float degrees) {
    ESP_LOGI(TAG, "setTargetDegrees: req=%.3f", degrees);

    degrees = softLimiter.clampTarget(degrees);

    if (targetMutex != nullptr && xSemaphoreTake(targetMutex, portMAX_DELAY) == pdTRUE) {
        target = degrees;
        xSemaphoreGive(targetMutex);
    } else {
        target = degrees;
        ESP_LOGW(TAG, "setTargetDegrees without mutex");
    }

    pid->reset();

    uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());

    motionDone.store(false, std::memory_order_relaxed);
    lastPos = encoder->getPositionInDegrees();
    resetMotionState(nowUs);
    motionGuard.startMotion(nowUs);

    ESP_LOGI(TAG, "Target set: tgt=%.3f, pos=%.3f", target, lastPos);

    if (notifyDriven == true) {
        BaseType_t higher = pdFALSE;
        notifyControlTaskFromISR(&higher);
        portYIELD_FROM_ISR();
        ESP_LOGI(TAG, "Notified control task after target change");
    }
}

void MotorController::setTargetTicks(int32_t ticks) {
    MotorControllerConfig configSnapshot = getConfig();
    float degrees = (static_cast<float>(ticks) / static_cast<float>(configSnapshot.countsPerRevolution)) * 360.0f;
    ESP_LOGI(TAG, "setTargetTicks: ticks=%" PRId32 " -> deg=%.3f (CPR=%d)", ticks, degrees, configSnapshot.countsPerRevolution);
    setTargetDegrees(degrees);
}

void MotorController::setTarget(float degrees) { setTargetDegrees(degrees); }

void MotorController::startTask() {
    if (taskHandle == nullptr) {
        UBaseType_t priorityToUse = controlTaskPriority;
        if (priorityToUse < 1) {
            priorityToUse = 1;
        }

        BaseType_t coreToUse = controlTaskCoreId;
        if (coreToUse != 0 && coreToUse != 1 && coreToUse != tskNO_AFFINITY) {
            coreToUse = tskNO_AFFINITY;
        }

        ESP_LOGI(TAG, "Starting control task (core=%ld, prio=%u, mode=%s, stack=%u, hz=%u)", static_cast<long>(coreToUse), static_cast<unsigned>(priorityToUse),
                 notifyDriven ? "notify" : "poll", 4096u, static_cast<unsigned>(updateHz));

        BaseType_t createResult = xTaskCreatePinnedToCore(taskFunc, "MotorControlTask", 4096, this, priorityToUse, &taskHandle, coreToUse);

        if (createResult != pdPASS) {
            ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
            taskHandle = nullptr;
        } else {
            ESP_LOGI(TAG, "Control task started");
        }
    } else {
        ESP_LOGW(TAG, "startTask: task already running");
    }
}

void MotorController::stopTask() {
    if (taskHandle != nullptr) {
        ESP_LOGI(TAG, "Stopping control task");
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
        ESP_LOGI(TAG, "Control task stopped");
    } else {
        ESP_LOGW(TAG, "stopTask: no task");
    }
}

void MotorController::stop() {
    ESP_LOGI(TAG, "Stop called");
    motor->stop();
    motionDone.store(true, std::memory_order_relaxed);
    resetMotionState(0);
}

void MotorController::setUpdateHz(uint32_t hz) {
    if (hz > 0) {
        updateHz = hz;
        ESP_LOGI(TAG, "UpdateHz set to %u", static_cast<unsigned>(hz));
    } else {
        ESP_LOGW(TAG, "Invalid UpdateHz %u", static_cast<unsigned>(hz));
    }
}

void MotorController::taskFunc(void* param) {
    auto* controller = static_cast<MotorController*>(param);

    TickType_t lastWake = xTaskGetTickCount();
    uint32_t localHz = controller->updateHz;
    if (localHz == 0) {
        localHz = 1;
    }

    TickType_t period = pdMS_TO_TICKS(1000 / localHz);
    if (period < 1) {
        period = 1;
    }

    ESP_LOGI(TAG, "Task loop start (mode=%s, periodTicks=%lu)", controller->notifyDriven ? "notify" : "poll", static_cast<unsigned long>(period));

    while (true) {
        if (controller->notifyDriven == true) {
            uint32_t taken = ulTaskNotifyTake(pdTRUE, controller->notifyBlockTicks);
            if (taken == 0) {
                continue;
            }
            controller->update();
        } else {
            controller->update();
            vTaskDelayUntil(&lastWake, period);
        }
    }
}

void MotorController::setPID(float kp, float ki, float kd) {
    pid->setParameters(kp, ki, kd);
    ESP_LOGI(TAG, "PID set: Kp=%.4f Ki=%.4f Kd=%.4f", kp, ki, kd);
}

void MotorController::getPID(float& kp, float& ki, float& kd) { pid->getParameters(kp, ki, kd); }

void MotorController::setConfig(const MotorControllerConfig& newConfig) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config = newConfig;
        xSemaphoreGive(configMutex);
        ESP_LOGI(TAG, "Config updated");
    } else {
        config = newConfig;
        ESP_LOGW(TAG, "Config updated without mutex");
    }

    ESP_LOGI(TAG,
             "cfg: minSpd=%.1f maxSpd=%.1f minErr=%.3f CPR=%d "
             "profile=%s type=%s accel=%.1f jerk=%.1f Kff_pos=%.3f Kff_vel=%.3f "
             "settlePos=%.3f settleVel=%.3f settleN=%d "
             "stuckEps=%.3f stuckN=%d pidWarmupN=%d "
             "timeoutMs=%d driftDead=%.3f hyst=%.3f",
             config.minSpeed, config.maxSpeed, config.minErrorToMove, config.countsPerRevolution, config.profiler.enabled ? "ON" : "OFF",
             (config.profiler.type == MotionProfileType::TRAPEZOID ? "TRAP" : "S"), config.profiler.accelLimitPctPerSec, config.profiler.jerkLimitPctPerSec2,
             config.Kff_pos, config.Kff_vel, config.settle.posTolDeg, config.settle.velTolDegPerSec, config.settle.countLimit,
             config.stall.stuckPositionEpsilon, config.stall.stuckCountLimit, config.stall.pidWarmupLimit, config.guard.motionTimeoutMs,
             config.guard.driftDeadband, config.guard.driftHysteresis);
}

MotorControllerConfig MotorController::getConfig() const {
    MotorControllerConfig snapshot;
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        snapshot = config;
        xSemaphoreGive(configMutex);
    } else {
        snapshot = config;
        ESP_LOGW(TAG, "getConfig without mutex");
    }
    return snapshot;
}

void MotorController::setMotionTimeoutMs(uint32_t ms) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.guard.motionTimeoutMs = static_cast<int>(ms);
        xSemaphoreGive(configMutex);
    } else {
        config.guard.motionTimeoutMs = static_cast<int>(ms);
        ESP_LOGW(TAG, "setMotionTimeoutMs without mutex");
    }
    ESP_LOGI(TAG, "Motion timeout set: %u ms", static_cast<unsigned>(ms));
}

uint32_t MotorController::getMotionTimeoutMs() const {
    uint32_t value = 0;
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        value = static_cast<uint32_t>(config.guard.motionTimeoutMs);
        xSemaphoreGive(configMutex);
    } else {
        value = static_cast<uint32_t>(config.guard.motionTimeoutMs);
        ESP_LOGW(TAG, "getMotionTimeoutMs without mutex");
    }
    return value;
}

void MotorController::enableMotionProfile(bool enable) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.profiler.enabled = enable;
        xSemaphoreGive(configMutex);
    } else {
        config.profiler.enabled = enable;
        ESP_LOGW(TAG, "enableMotionProfile without mutex");
    }
    ESP_LOGI(TAG, "Profile %s", config.profiler.enabled ? "ENABLED" : "DISABLED");
}

void MotorController::configureMotionProfile(MotionProfileType type, float accelPctPerSec, float jerkPctPerSec2) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.profiler.type = type;
        config.profiler.accelLimitPctPerSec = accelPctPerSec;
        config.profiler.jerkLimitPctPerSec2 = jerkPctPerSec2;
        xSemaphoreGive(configMutex);
    } else {
        config.profiler.type = type;
        config.profiler.accelLimitPctPerSec = accelPctPerSec;
        config.profiler.jerkLimitPctPerSec2 = jerkPctPerSec2;
        ESP_LOGW(TAG, "configureMotionProfile without mutex");
    }
    ESP_LOGI(TAG, "Profile cfg: type=%s accel=%.1f jerk=%.1f", type == MotionProfileType::TRAPEZOID ? "TRAP" : "S", accelPctPerSec, jerkPctPerSec2);
}

void MotorController::setFeedForward(float kpos, float kvel) {
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

void MotorController::getFeedForward(float& kpos, float& kvel) const {
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

void MotorController::setPidWarmupCycles(int cycles) {
    if (cycles < 0) {
        cycles = 0;
    }
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.stall.pidWarmupLimit = cycles;
        xSemaphoreGive(configMutex);
    } else {
        config.stall.pidWarmupLimit = cycles;
        ESP_LOGW(TAG, "setPidWarmupCycles without mutex");
    }
    stallDetector.reset();
    ESP_LOGI(TAG, "PID warmup cycles=%d", cycles);
}

int MotorController::getPidWarmupCycles() const {
    int value = 0;
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        value = config.stall.pidWarmupLimit;
        xSemaphoreGive(configMutex);
    } else {
        value = config.stall.pidWarmupLimit;
        ESP_LOGW(TAG, "getPidWarmupCycles without mutex");
    }
    return value;
}

void MotorController::configureControlTask(int coreId, UBaseType_t priority) {
    if (priority < 1) {
        priority = 1;
    }

    BaseType_t coreCandidate = static_cast<BaseType_t>(coreId);
    if (coreCandidate != 0 && coreCandidate != 1 && coreCandidate != tskNO_AFFINITY) {
        coreCandidate = tskNO_AFFINITY;
    }

    controlTaskPriority = priority;
    controlTaskCoreId = coreCandidate;

    ESP_LOGI(TAG, "Control task config: core=%ld priority=%u", static_cast<long>(controlTaskCoreId), static_cast<unsigned>(controlTaskPriority));
}

int MotorController::getControlTaskCore() const { return static_cast<int>(controlTaskCoreId); }

UBaseType_t MotorController::getControlTaskPriority() const { return controlTaskPriority; }

void MotorController::enableNotifyDrivenUpdates(bool enable, TickType_t maxBlockTicks) {
    notifyDriven = enable;
    notifyBlockTicks = maxBlockTicks;
    ESP_LOGI(TAG, "Notify-driven updates %s, blockTicks=%lu", notifyDriven ? "ENABLED" : "DISABLED", static_cast<unsigned long>(notifyBlockTicks));

    if (notifyDriven == true && taskHandle != nullptr) {
        BaseType_t higher = pdFALSE;
        notifyControlTaskFromISR(&higher);
        portYIELD_FROM_ISR();
        ESP_LOGI(TAG, "Kicked control task after enabling notify mode");
    }
}

void MotorController::notifyControlTaskFromISR(BaseType_t* higherPrioTaskWoken) {
    if (taskHandle != nullptr) {
        vTaskNotifyGiveFromISR(taskHandle, higherPrioTaskWoken);
    }
}

void MotorController::setOnMotionDone(MotionEventCallback cb, void* user) {
    onMotionDoneCb = cb;
    onMotionDoneUser = user;
    ESP_LOGI(TAG, "onMotionDone callback %s", cb != nullptr ? "SET" : "CLEARED");
}

void MotorController::setOnStall(MotionEventCallback cb, void* user) {
    onStallCb = cb;
    onStallUser = user;
    ESP_LOGI(TAG, "onStall callback %s", cb != nullptr ? "SET" : "CLEARED");
}

void MotorController::setOnLimitHit(MotionEventCallback cb, void* user) {
    onLimitHitCb = cb;
    onLimitHitUser = user;
    ESP_LOGI(TAG, "onLimitHit callback %s", cb != nullptr ? "SET" : "CLEARED");
}

bool MotorController::isMotionDone() const { return motionDone.load(std::memory_order_relaxed); }

float MotorController::clampToPercentRange(float signal) const {
    float clampedValue = signal;

    if (!std::isfinite(clampedValue)) {
        if (clampedValue != clampPreviousValue) ESP_LOGW(TAG, "clamp: non-finite signal -> 0");
        clampPreviousValue = 0.0f;
        return 0.0f;
    }

    if (clampedValue > 100.0f) {
        if (clampPreviousValue != 100.0f) ESP_LOGW(TAG, "clamp: %.3f -> 100.0", clampedValue);
        clampPreviousValue = 100.0f;
        return 100.0f;
    }

    if (clampedValue < -100.0f) {
        if (clampPreviousValue != -100.0f) ESP_LOGW(TAG, "clamp: %.3f -> -100.0", clampedValue);
        clampPreviousValue = -100.0f;
        return -100.0f;
    }

    clampPreviousValue = clampedValue;
    return clampedValue;
}

void MotorController::update() {
    uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());

    auto hit = [](uint64_t& lastLogUs, uint64_t now, uint32_t ms) {
        if (now - lastLogUs >= static_cast<uint64_t>(ms) * 1000ULL) {
            lastLogUs = now;
            return true;
        }
        return false;
    };

    float currentTarget;
    if (targetMutex != nullptr && xSemaphoreTake(targetMutex, portMAX_DELAY) == pdTRUE) {
        currentTarget = target;
        xSemaphoreGive(targetMutex);
    } else {
        currentTarget = target;
        ESP_LOGW(TAG, "update: target read without mutex");
    }

    float currentPos = encoder->getPositionInDegrees();
    if (hit(lastStateLogUs, nowUs, 500)) {
        ESP_LOGD(TAG, "update: t=%llu us, tgt=%.3f pos=%.3f", (unsigned long long)nowUs, currentTarget, currentPos);
    }

    if (motionDone.load(std::memory_order_relaxed)) {
        MotorControllerConfig idleConfig = getConfig();
        float drift = currentTarget - currentPos;
        if (hit(lastStateLogUs, nowUs, 500)) {
            float wakeThreshold = idleConfig.guard.driftDeadband + idleConfig.guard.driftHysteresis;
            if (wakeThreshold < 0.0f) wakeThreshold = 0.0f;
            ESP_LOGD(TAG, "idle: drift=%.3f wakeThreshold=%.3f", drift, wakeThreshold);
        }
        if (motionGuard.shouldWake(drift, idleConfig.guard)) {
            setTargetDegrees(currentTarget);
        }
        return;
    }

    MotorControllerConfig controlConfig = getConfig();

    if (motionGuard.isTimedOut(nowUs, controlConfig.guard)) {
        motor->stop();
        motionDone.store(true, std::memory_order_relaxed);
        if (onStallCb) {
            MotorStatus statusSnapshot = getStatus();
            onStallCb(statusSnapshot, onStallUser);
        }
        resetMotionState(0);
        return;
    }

    float pidSignal = pid->compute(currentTarget, currentPos);
    pidSignal = clampToPercentRange(pidSignal);
    lastPidOut = pidSignal;

    if (hit(lastPidLogUs, nowUs, 500)) {
        float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;
        getPID(Kp, Ki, Kd);
        ESP_LOGD(TAG, "pid: Kp=%.4f Ki=%.4f Kd=%.4f out=%.3f", Kp, Ki, Kd, pidSignal);
    }

    float referencePosition = currentTarget;
    float referenceVelocityPercent = profiler.getProfiledSpeed();
    float feedForward = controlConfig.Kff_pos * referencePosition + controlConfig.Kff_vel * referenceVelocityPercent;

    if (hit(lastPidLogUs, nowUs, 500)) {
        ESP_LOGV(TAG, "ff: Kpos=%.4f Kvel=%.4f refPos=%.3f refVel%%=%.2f ff=%.3f", controlConfig.Kff_pos, controlConfig.Kff_vel, referencePosition, referenceVelocityPercent, feedForward);
    }

    float combined = clampToPercentRange(pidSignal + feedForward);

    if (softLimiter.isBlocked(combined, currentPos)) {
        motor->stop();
        motionDone.store(true, std::memory_order_relaxed);
        if (onLimitHitCb) {
            MotorStatus statusSnapshot = getStatus();
            onLimitHitCb(statusSnapshot, onLimitHitUser);
        }
        resetMotionState(0);
        return;
    }

    float absoluteError = fabsf(currentTarget - currentPos);
    float magnitude = fabsf(combined);
    if (magnitude < controlConfig.minSpeed && absoluteError > controlConfig.minErrorToMove) {
        magnitude = controlConfig.minSpeed;
    }
    if (magnitude > controlConfig.maxSpeed) {
        magnitude = controlConfig.maxSpeed;
    }

    float signedDesired = (combined >= 0.0f) ? magnitude : -magnitude;

    float profiled = profiler.shapeSpeed(signedDesired, nowUs, controlConfig.profiler);

    float deltaPosition = currentPos - lastPos;
    float velocity = deltaPosition * static_cast<float>(updateHz);
    lastVelDegPerSec = velocity;
    if (hit(lastStateLogUs, nowUs, 500)) {
        ESP_LOGD(TAG, "state: err=%.3f vel=%.3f deg/s", currentTarget - currentPos, velocity);
    }

    bool pidIsSettled = false;
    if (!stallDetector.isWarming()) {
        pidIsSettled = pid->isSettled();
    }
    StallDetector::Result stallResult = stallDetector.update(deltaPosition, absoluteError, pidIsSettled, controlConfig.stall);

    bool settled = false;
    if (stallResult != StallDetector::Result::WARMING_UP) {
        settled = settleDetector.update(absoluteError, velocity, controlConfig.settle);
        if (hit(lastStateLogUs, nowUs, 500)) {
            ESP_LOGD(TAG, "settle: cnt=%d | stuckCnt=%d move=%.4f eps=%.4f", settleDetector.getCount(), stallDetector.getStuckCount(), fabsf(deltaPosition),
                     controlConfig.stall.stuckPositionEpsilon);
        }
    }

    if (settled) {
        ESP_LOGI(TAG, "motion DONE: settleCnt=%d", settleDetector.getCount());
        motor->stop();
        motionDone.store(true, std::memory_order_relaxed);
        if (onMotionDoneCb) {
            MotorStatus statusSnapshot = getStatus();
            onMotionDoneCb(statusSnapshot, onMotionDoneUser);
        }
        resetMotionState(0);
        lastPos = currentPos;
        return;
    }

    if (stallResult == StallDetector::Result::STALLED) {
        motor->stop();
        if (targetMutex != nullptr && xSemaphoreTake(targetMutex, portMAX_DELAY) == pdTRUE) {
            target = currentPos;
            xSemaphoreGive(targetMutex);
        } else {
            target = currentPos;
        }
        motionDone.store(true, std::memory_order_relaxed);
        if (onStallCb) {
            MotorStatus statusSnapshot = getStatus();
            onStallCb(statusSnapshot, onStallUser);
        }
        profiler.reset(0);
        settleDetector.reset();
        motionGuard.reset();
        lastPos = currentPos;
        return;
    }

    float slewOut = profiler.applySlew(profiled, nowUs, controlConfig.profiler.accelLimitPctPerSec);

    float speedPercent = fabsf(slewOut);
    if (speedPercent > controlConfig.maxSpeed) {
        speedPercent = controlConfig.maxSpeed;
    }

    MotorDirection dir = MotorDirection::RIGHT;
    if (slewOut < 0.0f) {
        dir = MotorDirection::LEFT;
    }
    motor->setDirection(dir);
    motor->setSpeed(speedPercent);

    if (hit(lastCmdLogUs, nowUs, 250)) {
        const char* directionString = (slewOut < 0.0f) ? "LEFT" : "RIGHT";
        ESP_LOGI(TAG, "cmd: dir=%s speed=%.2f%%", directionString, speedPercent);
    }

    lastPos = currentPos;

    if (hit(lastStatusLogUs, nowUs, 1000)) {
        MotorStatus statusSnapshot = getStatus();
        logMotorStatus(statusSnapshot);
    }
}

MotorStatus MotorController::getStatus() const {
    MotorStatus status{};

    float currentTarget;
    if (targetMutex != nullptr && xSemaphoreTake(targetMutex, portMAX_DELAY) == pdTRUE) {
        currentTarget = target;
        xSemaphoreGive(targetMutex);
    } else {
        currentTarget = target;
    }

    float currentPos = encoder->getPositionInDegrees();

    status.target = currentTarget;
    status.position = currentPos;
    status.error = currentTarget - currentPos;
    status.velocity = lastVelDegPerSec;
    status.pidOutput = lastPidOut;
    status.stuckCount = stallDetector.getStuckCount();
    status.motionDone = motionDone.load(std::memory_order_relaxed);
    return status;
}

void MotorController::logMotorStatus(const MotorStatus& status) {
    ESP_LOGI(TAG, "=== Motor Status ===");
    ESP_LOGI(TAG, "Target: %.3f, Position: %.3f, Error: %.3f", status.target, status.position, status.error);
    ESP_LOGI(TAG, "Velocity: %.3f/s, PID Output: %.3f", status.velocity, status.pidOutput);
    ESP_LOGI(TAG, "Stuck Count: %d, Motion Done: %s", status.stuckCount, status.motionDone ? "YES" : "NO");
    ESP_LOGI(TAG, "====================");
}

void MotorController::setSoftLimits(float minDeg, float maxDeg, bool enforce) {
    ESP_LOGI(TAG, "setSoftLimits: req[%.3f, %.3f], enforce=%d", minDeg, maxDeg, static_cast<int>(enforce));

    softLimiter.setLimits(minDeg, maxDeg, enforce);

    if (softLimiter.isEnforced()) {
        float clamped = softLimiter.clampTarget(target);
        if (fabsf(clamped - target) > 1e-6f) {
            ESP_LOGW(TAG, "Target clamped to %.3f due to soft limits", clamped);
            setTargetDegrees(clamped);
        }
    }

    ESP_LOGI(TAG, "Soft limits set: [%.3f, %.3f], %s", softLimiter.getMinDeg(), softLimiter.getMaxDeg(),
             softLimiter.isEnforced() ? "ENFORCED" : "NOT ENFORCED");
}
