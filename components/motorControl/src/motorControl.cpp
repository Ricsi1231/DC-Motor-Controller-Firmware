#include "motorControl.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>

using namespace DC_Motor_Controller_Firmware;
using namespace Control;

MotorController::MotorController(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pidRef, const MotorControllerConfig& initialConfig)
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

    ESP_LOGI(TAG, "Ctor done");
}

MotorController::MotorController(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pidRef, PID::FuzzyPIDController& fuzzyRef,
                                 const MotorControllerConfig& cfg)
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

    useFuzzy = true;
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
    : encoder(other.encoder), motor(other.motor), pid(other.pid), fuzzy(other.fuzzy), config(other.config) {
    ESP_LOGI(TAG, "Move Ctor start");
    useFuzzy = other.useFuzzy;
    target = other.target;
    motionDone.store(other.motionDone.load(std::memory_order_relaxed), std::memory_order_relaxed);
    lastPos = other.lastPos;
    stuckCounter = other.stuckCounter;
    pidWarmupCounter = other.pidWarmupCounter;
    updateHz = other.updateHz;
    lastPidOut = other.lastPidOut;
    lastVelDegPerSec = other.lastVelDegPerSec;

    targetMutex = other.targetMutex;
    configMutex = other.configMutex;
    taskHandle = other.taskHandle;

    motionStartUs = other.motionStartUs;

    profSpeedPercent = other.profSpeedPercent;
    profAccelPctPerSec = other.profAccelPctPerSec;
    lastProfileUs = other.lastProfileUs;

    settleCounter = other.settleCounter;

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
    other.motionDone.store(true, std::memory_order_relaxed);
    other.motionStartUs = 0;
    other.profSpeedPercent = 0.0f;
    other.profAccelPctPerSec = 0.0f;
    other.lastProfileUs = 0;
    other.settleCounter = 0;

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

void MotorController::setTargetDegrees(float degrees) {
    ESP_LOGI(TAG, "setTargetDegrees: req=%.3f", degrees);

    if (softLimitsEnforced == true) {
        if (softMinDeg > softMaxDeg) {
            float swapTmp = softMinDeg;
            softMinDeg = softMaxDeg;
            softMaxDeg = swapTmp;
            ESP_LOGW(TAG, "Soft limits swapped (min>max)");
        }
        if (degrees < softMinDeg) {
            ESP_LOGW(TAG, "Target below softMin (%.3f < %.3f), clamping", degrees, softMinDeg);
            degrees = softMinDeg;
        }
        if (degrees > softMaxDeg) {
            ESP_LOGW(TAG, "Target above softMax (%.3f > %.3f), clamping", degrees, softMaxDeg);
            degrees = softMaxDeg;
        }
    }

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
    stuckCounter = 0;
    pidWarmupCounter = 0;
    settleCounter = 0;
    motionStartUs = static_cast<uint64_t>(esp_timer_get_time());

    profSpeedPercent = 0.0f;
    profAccelPctPerSec = 0.0f;
    lastProfileUs = motionStartUs;

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

        BaseType_t res = xTaskCreatePinnedToCore(taskFunc, "MotorControlTask", 4096, this, priorityToUse, &taskHandle, coreToUse);

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
    motor.stop();
    motionDone.store(true, std::memory_order_relaxed);
    motionStartUs = 0;
    profSpeedPercent = 0.0f;
    profAccelPctPerSec = 0.0f;
    lastProfileUs = 0;
    settleCounter = 0;
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
    auto* self = static_cast<MotorController*>(param);

    TickType_t lastWake = xTaskGetTickCount();
    uint32_t localHz = self->updateHz;
    if (localHz == 0) {
        localHz = 1;
    }

    TickType_t period = pdMS_TO_TICKS(1000 / localHz);
    if (period < 1) {
        period = 1;
    }

    ESP_LOGI(TAG, "Task loop start (mode=%s, periodTicks=%lu)", self->notifyDriven ? "notify" : "poll", static_cast<unsigned long>(period));

    while (true) {
        if (self->notifyDriven == true) {
            uint32_t taken = ulTaskNotifyTake(pdTRUE, self->notifyBlockTicks);
            if (taken == 0) {
                continue;
            }
            self->update();
        } else {
            self->update();
            vTaskDelayUntil(&lastWake, period);
        }
    }
}

void MotorController::setPID(float kp, float ki, float kd) {
    pid.setParameters(kp, ki, kd);
    ESP_LOGI(TAG, "PID set: Kp=%.4f Ki=%.4f Kd=%.4f", kp, ki, kd);
}

void MotorController::getPID(float& kp, float& ki, float& kd) {
    if (useFuzzy) {
        fuzzy.getCurrentGains(kp, ki, kd);
    } else {
        pid.getParameters(kp, ki, kd);
    }
}

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
             "cfg: minSpd=%.1f maxSpd=%.1f minErr=%.3f driftDead=%.3f hyst=%.3f stuckEps=%.3f stuckN=%d pidWarmupN=%d "
             "CPR=%d timeoutMs=%d profile=%s type=%s accel=%.1f jerk=%.1f Kff_pos=%.3f Kff_vel=%.3f "
             "settlePos=%.3f settleVel=%.3f settleN=%d",
             config.minSpeed, config.maxSpeed, config.minErrorToMove, config.driftDeadband, config.driftHysteresis, config.stuckPositionEpsilon,
             config.stuckCountLimit, config.pidWarmupLimit, config.countsPerRevolution, config.motionTimeoutMs, config.motionProfileEnabled ? "ON" : "OFF",
             (config.motionProfileType == MotionProfileType::TRAPEZOID ? "TRAP" : "S"), config.accelLimitPctPerSec, config.jerkLimitPctPerSec2, config.Kff_pos,
             config.Kff_vel, config.settlePosTolDeg, config.settleVelTolDegPerSec, config.settleCountLimit);
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
        config.motionTimeoutMs = static_cast<int>(ms);
        xSemaphoreGive(configMutex);
    } else {
        config.motionTimeoutMs = static_cast<int>(ms);
        ESP_LOGW(TAG, "setMotionTimeoutMs without mutex");
    }
    ESP_LOGI(TAG, "Motion timeout set: %u ms", static_cast<unsigned>(ms));
}

uint32_t MotorController::getMotionTimeoutMs() const {
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

void MotorController::enableMotionProfile(bool enable) {
    if (configMutex != nullptr && xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
        config.motionProfileEnabled = enable ? true : false;
        xSemaphoreGive(configMutex);
    } else {
        config.motionProfileEnabled = enable ? true : false;
        ESP_LOGW(TAG, "enableMotionProfile without mutex");
    }
    ESP_LOGI(TAG, "Profile %s", config.motionProfileEnabled ? "ENABLED" : "DISABLED");
}

void MotorController::configureMotionProfile(MotionProfileType type, float accelPctPerSec, float jerkPctPerSec2) {
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
        config.pidWarmupLimit = cycles;
        xSemaphoreGive(configMutex);
    } else {
        config.pidWarmupLimit = cycles;
        ESP_LOGW(TAG, "setPidWarmupCycles without mutex");
    }
    pidWarmupCounter = 0;
    ESP_LOGI(TAG, "PID warmup cycles=%d", cycles);
}

int MotorController::getPidWarmupCycles() const {
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
    notifyDriven = enable ? true : false;
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

float MotorController::shapeSpeedWithProfile(float desiredSigned, uint64_t nowUs) {
    MotorControllerConfig configSnapshot = getConfig();

    if (configSnapshot.motionProfileEnabled == false) {
        profSpeedPercent = desiredSigned;
        lastProfileUs = nowUs;
        ESP_LOGD(TAG, "profile: OFF desired=%.2f -> prof=%.2f", desiredSigned, profSpeedPercent);
        return profSpeedPercent;
    }

    if (lastProfileUs == 0) {
        lastProfileUs = nowUs;
    }

    double timeStepSec = static_cast<double>(nowUs - lastProfileUs) / 1e6;
    if (timeStepSec < 0.0) {
        timeStepSec = 0.0;
    }

    float desiredClamped = desiredSigned;
    if (desiredClamped > configSnapshot.maxSpeed) {
        desiredClamped = configSnapshot.maxSpeed;
    }
    if (desiredClamped < -configSnapshot.maxSpeed) {
        desiredClamped = -configSnapshot.maxSpeed;
    }

    if (configSnapshot.motionProfileType == MotionProfileType::TRAPEZOID) {
        double maxDelta = static_cast<double>(configSnapshot.accelLimitPctPerSec) * timeStepSec;
        double delta = static_cast<double>(desiredClamped) - static_cast<double>(profSpeedPercent);

        if (delta > maxDelta) {
            delta = maxDelta;
        }
        if (delta < -maxDelta) {
            delta = -maxDelta;
        }

        profSpeedPercent = static_cast<float>(static_cast<double>(profSpeedPercent) + delta);
        ESP_LOGD(TAG, "profile TRAP: dt=%.4f des=%.2f maxΔ=%.2f -> prof=%.2f", timeStepSec, desiredClamped, maxDelta, profSpeedPercent);
    } else {
        double speedErr = static_cast<double>(desiredClamped) - static_cast<double>(profSpeedPercent);
        double accelTarget = 0.0;

        if (timeStepSec > 0.0) {
            accelTarget = speedErr / timeStepSec;
        }

        double accelCap = static_cast<double>(configSnapshot.accelLimitPctPerSec);
        if (accelTarget > accelCap) {
            accelTarget = accelCap;
        }
        if (accelTarget < -accelCap) {
            accelTarget = -accelCap;
        }

        double jerkCap = static_cast<double>(configSnapshot.jerkLimitPctPerSec2);
        double accelStepCap = jerkCap * timeStepSec;
        double accelStep = accelTarget - static_cast<double>(profAccelPctPerSec);

        if (accelStep > accelStepCap) {
            accelStep = accelStepCap;
        }
        if (accelStep < -accelStepCap) {
            accelStep = -accelStepCap;
        }

        profAccelPctPerSec = static_cast<float>(static_cast<double>(profAccelPctPerSec) + accelStep);

        double newSpeed = static_cast<double>(profSpeedPercent) + static_cast<double>(profAccelPctPerSec) * timeStepSec;

        if ((desiredClamped - profSpeedPercent) > 0.0f && newSpeed > static_cast<double>(desiredClamped)) {
            newSpeed = static_cast<double>(desiredClamped);
            profAccelPctPerSec = 0.0f;
        }
        if ((desiredClamped - profSpeedPercent) < 0.0f && newSpeed < static_cast<double>(desiredClamped)) {
            newSpeed = static_cast<double>(desiredClamped);
            profAccelPctPerSec = 0.0f;
        }

        profSpeedPercent = static_cast<float>(newSpeed);
        ESP_LOGD(TAG, "profile S: dt=%.4f des=%.2f a*=%.2f j*=%.2f -> a=%.2f prof=%.2f", timeStepSec, desiredClamped, accelTarget, jerkCap, profAccelPctPerSec,
                 profSpeedPercent);
    }

    lastProfileUs = nowUs;
    return profSpeedPercent;
}

void MotorController::update() {
    uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());

    auto hit = [](uint64_t& t_last, uint64_t now, uint32_t ms) {
        if (now - t_last >= (uint64_t)ms * 1000ULL) {
            t_last = now;
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

    float currentPos = encoder.getPositionInDegrees();
    if (hit(lastStateLogUs, nowUs, 500)) {
        ESP_LOGD(TAG, "update: t=%llu us, tgt=%.3f pos=%.3f", (unsigned long long)nowUs, currentTarget, currentPos);
    }

    if (motionDone.load(std::memory_order_relaxed)) {
        MotorControllerConfig cfg0 = getConfig();
        float drift = currentTarget - currentPos;
        float wakeTh = cfg0.driftDeadband + cfg0.driftHysteresis;
        float wakeThLog = wakeTh;
        if (wakeThLog < 0.0f) {
            wakeThLog = 0.0f;
        }
        if (hit(lastStateLogUs, nowUs, 500)) {
            ESP_LOGD(TAG, "idle: drift=%.3f wakeTh=%.3f", drift, wakeThLog);
        }
        float wakeThreshold = wakeTh;
        if (wakeThreshold < 0.0f) {
            wakeThreshold = 0.0f;
        }
        if (fabsf(drift) > wakeThreshold) {
            setTargetDegrees(currentTarget);
        }
        return;
    }

    MotorControllerConfig cfg = getConfig();

    if (cfg.motionTimeoutMs > 0) {
        if (motionStartUs == 0) {
            motionStartUs = nowUs;
        }
        uint64_t elapsedUs = nowUs - motionStartUs;
        uint64_t limitUs = (uint64_t)cfg.motionTimeoutMs * 1000ULL;
        if (elapsedUs > limitUs) {
            ESP_LOGW(TAG, "timeout: %llu ms > %u ms", (unsigned long long)(elapsedUs / 1000ULL), (unsigned int)cfg.motionTimeoutMs);
            motor.stop();
            motionDone.store(true, std::memory_order_relaxed);
            if (onStallCb) {
                MotorStatus s0 = getStatus();
                onStallCb(s0, onStallUser);
            }
            motionStartUs = 0;
            profSpeedPercent = 0.0f;
            profAccelPctPerSec = 0.0f;
            lastProfileUs = 0;
            settleCounter = 0;
            return;
        }
    }

    float pidSignal = 0;
    if (useFuzzy == true) {
        pidSignal = fuzzy.compute(currentTarget, currentPos);
    } else {
        pidSignal = pid.compute(currentTarget, currentPos);
    }
    pidSignal = clampToPercentRange(pidSignal);
    lastPidOut = pidSignal;

    if (hit(lastPidLogUs, nowUs, 500)) {
        float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;
        if (useFuzzy == true) {
            fuzzy.getCurrentGains(Kp, Ki, Kd);
        } else {
            getPID(Kp, Ki, Kd);
        }
        ESP_LOGD(TAG, "pid: Kp=%.4f Ki=%.4f Kd=%.4f out=%.3f", Kp, Ki, Kd, pidSignal);
    }

    float refPos = currentTarget;
    float refVelPct = profSpeedPercent;
    float ff = cfg.Kff_pos * refPos + cfg.Kff_vel * refVelPct;

    if (hit(lastPidLogUs, nowUs, 500)) {
        ESP_LOGV(TAG, "ff: Kpos=%.4f Kvel=%.4f refPos=%.3f refVel%%=%.2f ff=%.3f", cfg.Kff_pos, cfg.Kff_vel, refPos, refVelPct, ff);
    }

    float combined = clampToPercentRange(pidSignal + ff);
    if (softLimitsEnforced == true) {
        if (combined < 0.0f && currentPos <= softMinDeg) {
            ESP_LOGW(TAG, "limit: LEFT at %.3f (softMin %.3f), stopping", currentPos, softMinDeg);
            motor.stop();
            motionDone.store(true, std::memory_order_relaxed);
            if (onLimitHitCb) {
                MotorStatus s1 = getStatus();
                onLimitHitCb(s1, onLimitHitUser);
            }
            profSpeedPercent = 0.0f;
            profAccelPctPerSec = 0.0f;
            lastProfileUs = 0;
            settleCounter = 0;
            return;
        }
        if (combined > 0.0f && currentPos >= softMaxDeg) {
            ESP_LOGW(TAG, "limit: RIGHT at %.3f (softMax %.3f), stopping", currentPos, softMaxDeg);
            motor.stop();
            motionDone.store(true, std::memory_order_relaxed);
            if (onLimitHitCb) {
                MotorStatus s2 = getStatus();
                onLimitHitCb(s2, onLimitHitUser);
            }
            profSpeedPercent = 0.0f;
            profAccelPctPerSec = 0.0f;
            lastProfileUs = 0;
            settleCounter = 0;
            return;
        }
    }

    float errAbs = fabsf(currentTarget - currentPos);
    float mag = fabsf(combined);
    if (mag < cfg.minSpeed && errAbs > cfg.minErrorToMove) {
        mag = cfg.minSpeed;
    }
    if (mag > cfg.maxSpeed) {
        mag = cfg.maxSpeed;
    }

    float signedDesired;
    if (combined >= 0.0f) {
        signedDesired = mag;
    } else {
        signedDesired = -mag;
    }
    float profiled = shapeSpeedWithProfile(signedDesired, nowUs);

    float dpos = currentPos - lastPos;
    float vel = dpos * (float)updateHz;
    lastVelDegPerSec = vel;
    if (hit(lastStateLogUs, nowUs, 500)) {
        ESP_LOGD(TAG, "state: err=%.3f vel=%.3f deg/s", currentTarget - currentPos, vel);
    }

    if (pidWarmupCounter <= cfg.pidWarmupLimit) {
        stuckCounter = 0;
        settleCounter = 0;
    } else {
        float moveAbs = fabsf(dpos);
        if (errAbs > cfg.minErrorToMove && moveAbs < cfg.stuckPositionEpsilon) {
            stuckCounter++;
        } else {
            stuckCounter = 0;
        }
        bool posOk = (errAbs < cfg.settlePosTolDeg);
        bool velOk = (fabsf(vel) < cfg.settleVelTolDegPerSec);
        if (posOk && velOk) {
            settleCounter++;
        } else {
            settleCounter = 0;
        }
        if (hit(lastStateLogUs, nowUs, 500)) {
            ESP_LOGD(TAG, "settle: posOk=%d velOk=%d cnt=%d | stuckCnt=%d move=%.4f eps=%.4f", (int)posOk, (int)velOk, settleCounter, stuckCounter, moveAbs,
                     cfg.stuckPositionEpsilon);
        }
    }

    if (settleCounter >= cfg.settleCountLimit) {
        ESP_LOGI(TAG, "motion DONE: settleCnt=%d", settleCounter);
        motor.stop();
        motionDone.store(true, std::memory_order_relaxed);
        if (onMotionDoneCb) {
            MotorStatus s3 = getStatus();
            onMotionDoneCb(s3, onMotionDoneUser);
        }
        motionStartUs = 0;
        profSpeedPercent = 0.0f;
        profAccelPctPerSec = 0.0f;
        lastProfileUs = 0;
        settleCounter = 0;
        lastPos = currentPos;
        return;
    }

    bool pidSettledFlag = false;
    if (pidWarmupCounter > cfg.pidWarmupLimit) {
        if (useFuzzy == true) {
            pidSettledFlag = fuzzy.isSettled();
        } else {
            pidSettledFlag = pid.isSettled();
        }
    }

    if (stuckCounter > cfg.stuckCountLimit || (pidWarmupCounter > cfg.pidWarmupLimit && pidSettledFlag == true)) {
        // ESP_LOGW(TAG, "stall: stuckCnt=%d pidSettled=%d", stuckCounter, (int)pidSettledFlag);
        motor.stop();
        float finalDriftAbs = fabsf(currentTarget - currentPos);
        bool doneOk = (finalDriftAbs <= cfg.driftDeadband);
        motionDone.store(doneOk, std::memory_order_relaxed);
        if (onStallCb) {
            MotorStatus s4 = getStatus();
            onStallCb(s4, onStallUser);
        }
        motionStartUs = 0;
        profSpeedPercent = 0.0f;
        profAccelPctPerSec = 0.0f;
        lastProfileUs = 0;
        settleCounter = 0;
        lastPos = currentPos;
        return;
    }

    pidWarmupCounter++;

    if (lastSlewUs == 0) {
        lastSlewUs = nowUs;
        slewOutPct = 0.0f;
    }

    double dt = (double)(nowUs - lastSlewUs) / 1e6;
    if (dt < 0.0) {
        dt = 0.0;
    }
    double maxDelta = (double)cfg.accelLimitPctPerSec * dt;
    double reqDelta = (double)profiled - (double)slewOutPct;
    if (reqDelta > maxDelta) {
        reqDelta = maxDelta;
    }
    if (reqDelta < -maxDelta) {
        reqDelta = -maxDelta;
    }
    float slewOut = (float)((double)slewOutPct + reqDelta);

    slewOutPct = slewOut;
    lastSlewUs = nowUs;

    float speedPct = fabsf(slewOut);
    if (speedPct > cfg.maxSpeed) {
        speedPct = cfg.maxSpeed;
    }

    DRV8876::Direction dir = DRV8876::Direction::RIGHT;
    if (slewOut < 0.0f) {
        dir = DRV8876::Direction::LEFT;
    }
    motor.setDirection(dir);
    motor.setSpeed(speedPct);

    if (hit(lastCmdLogUs, nowUs, 250)) {
        const char* dirStr = "RIGHT";
        if (slewOut < 0.0f) {
            dirStr = "LEFT";
        }
        ESP_LOGI(TAG, "cmd: dir=%s speed=%.2f%%", dirStr, speedPct);
    }

    lastPos = currentPos;

    if (hit(lastStatusLogUs, nowUs, 1000)) {
        MotorStatus s = getStatus();
        logMotorStatus(s);
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

    float currentPos = encoder.getPositionInDegrees();

    status.target = currentTarget;
    status.position = currentPos;
    status.error = currentTarget - currentPos;
    status.velocity = lastVelDegPerSec;
    status.pidOutput = lastPidOut;
    status.stuckCount = stuckCounter;
    status.motionDone = motionDone.load(std::memory_order_relaxed);
    return status;
}

void MotorController::logMotorStatus(const MotorStatus& status) {
    ESP_LOGI(TAG, "=== Motor Status ===");
    ESP_LOGI(TAG, "Target: %.3f°, Position: %.3f°, Error: %.3f°", status.target, status.position, status.error);
    ESP_LOGI(TAG, "Velocity: %.3f°/s, PID Output: %.3f", status.velocity, status.pidOutput);
    ESP_LOGI(TAG, "Stuck Count: %d, Motion Done: %s", status.stuckCount, status.motionDone ? "YES" : "NO");
    ESP_LOGI(TAG, "====================");
}

void MotorController::setSoftLimits(float minDeg, float maxDeg, bool enforce) {
    ESP_LOGI(TAG, "setSoftLimits: req[%.3f, %.3f], enforce=%d", minDeg, maxDeg, static_cast<int>(enforce));

    if (minDeg > maxDeg) {
        float tmp = minDeg;
        minDeg = maxDeg;
        maxDeg = tmp;
        ESP_LOGW(TAG, "setSoftLimits: min>max, swapped");
    }

    softMinDeg = minDeg;
    softMaxDeg = maxDeg;
    softLimitsEnforced = enforce ? true : false;

    if (softLimitsEnforced == true) {
        float clamped = target;

        if (clamped < softMinDeg) {
            clamped = softMinDeg;
        }
        if (clamped > softMaxDeg) {
            clamped = softMaxDeg;
        }

        if (fabsf(clamped - target) > 1e-6f) {
            ESP_LOGW(TAG, "Target clamped to %.3f due to soft limits", clamped);
        }
        setTargetDegrees(clamped);
    }

    ESP_LOGI(TAG, "Soft limits set: [%.3f°, %.3f°], %s", softMinDeg, softMaxDeg, softLimitsEnforced ? "ENFORCED" : "NOT ENFORCED");
}
