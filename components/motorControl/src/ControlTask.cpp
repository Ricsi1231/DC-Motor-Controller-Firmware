#include "ControlTask.hpp"
#include "esp_log.h"

namespace DC_Motor_Controller_Firmware::Control {

static constexpr const char* TAG = "ControlTask";

ControlTask::ControlTask(const ControlTaskConfig& cfg) : config(cfg) {
    exitSemaphore = xSemaphoreCreateBinary();
    if (exitSemaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create exit semaphore");
    }
}

ControlTask::~ControlTask() {
    stop();
    if (exitSemaphore != nullptr) {
        vSemaphoreDelete(exitSemaphore);
        exitSemaphore = nullptr;
    }
}

void ControlTask::setUpdateCallback(UpdateCallback cb) {
    updateCallback = cb;
}

bool ControlTask::start() {
    if (taskHandle != nullptr) {
        ESP_LOGW(TAG, "Task already running");
        return false;
    }

    if (!updateCallback) {
        ESP_LOGE(TAG, "No update callback set");
        return false;
    }

    UBaseType_t priorityToUse = config.priority;
    if (priorityToUse < 1) {
        priorityToUse = 1;
    }

    BaseType_t coreToUse = config.coreId;
    if (coreToUse != 0 && coreToUse != 1 && coreToUse != tskNO_AFFINITY) {
        coreToUse = tskNO_AFFINITY;
    }

    stopRequested.store(false, std::memory_order_relaxed);

    ESP_LOGI(TAG, "Starting task (core=%ld, prio=%u, mode=%s, stack=%u, hz=%u)",
             static_cast<long>(coreToUse), static_cast<unsigned>(priorityToUse),
             config.notifyDriven ? "notify" : "poll",
             static_cast<unsigned>(config.stackSize),
             static_cast<unsigned>(config.updateHz));

    BaseType_t res = xTaskCreatePinnedToCore(
        taskFunction,
        "ControlTask",
        config.stackSize,
        this,
        priorityToUse,
        &taskHandle,
        coreToUse
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        taskHandle = nullptr;
        return false;
    }

    running.store(true, std::memory_order_relaxed);
    ESP_LOGI(TAG, "Task started");
    return true;
}

bool ControlTask::stop(uint32_t timeoutMs) {
    if (taskHandle == nullptr) {
        ESP_LOGW(TAG, "No task to stop");
        return true;
    }

    ESP_LOGI(TAG, "Requesting task stop");
    stopRequested.store(true, std::memory_order_relaxed);

    if (config.notifyDriven && taskHandle != nullptr) {
        xTaskNotifyGive(taskHandle);
    }

    if (exitSemaphore != nullptr) {
        BaseType_t taken = xSemaphoreTake(exitSemaphore, pdMS_TO_TICKS(timeoutMs));
        if (taken != pdTRUE) {
            ESP_LOGW(TAG, "Task did not exit in time, forcing delete");
            vTaskDelete(taskHandle);
        }
    } else {
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(taskHandle);
    }

    taskHandle = nullptr;
    stopRequested.store(false, std::memory_order_relaxed);
    running.store(false, std::memory_order_relaxed);
    ESP_LOGI(TAG, "Task stopped");
    return true;
}

bool ControlTask::isRunning() const noexcept {
    return running.load(std::memory_order_relaxed);
}

void ControlTask::notifyFromISR(BaseType_t* higherPrioTaskWoken) {
    if (taskHandle != nullptr) {
        vTaskNotifyGiveFromISR(taskHandle, higherPrioTaskWoken);
    }
}

void ControlTask::notify() {
    if (taskHandle != nullptr) {
        xTaskNotifyGive(taskHandle);
    }
}

void ControlTask::setConfig(const ControlTaskConfig& cfg) {
    config = cfg;
}

ControlTaskConfig ControlTask::getConfig() const noexcept {
    return config;
}

void ControlTask::setUpdateHz(uint32_t hz) {
    if (hz > 0) {
        config.updateHz = hz;
        ESP_LOGI(TAG, "UpdateHz set to %u", static_cast<unsigned>(hz));
    } else {
        ESP_LOGW(TAG, "Invalid UpdateHz %u", static_cast<unsigned>(hz));
    }
}

uint32_t ControlTask::getUpdateHz() const noexcept {
    return config.updateHz;
}

void ControlTask::configureTask(int coreId, UBaseType_t priority) {
    if (priority < 1) {
        priority = 1;
    }

    BaseType_t coreCandidate = static_cast<BaseType_t>(coreId);
    if (coreCandidate != 0 && coreCandidate != 1 && coreCandidate != tskNO_AFFINITY) {
        coreCandidate = tskNO_AFFINITY;
    }

    config.priority = priority;
    config.coreId = coreCandidate;

    ESP_LOGI(TAG, "Task config: core=%ld priority=%u",
             static_cast<long>(config.coreId),
             static_cast<unsigned>(config.priority));
}

int ControlTask::getCoreId() const noexcept {
    return static_cast<int>(config.coreId);
}

UBaseType_t ControlTask::getPriority() const noexcept {
    return config.priority;
}

void ControlTask::enableNotifyDrivenUpdates(bool enable, TickType_t maxBlockTicks) {
    config.notifyDriven = enable;
    config.notifyBlockTicks = maxBlockTicks;
    ESP_LOGI(TAG, "Notify-driven updates %s, blockTicks=%lu",
             config.notifyDriven ? "ENABLED" : "DISABLED",
             static_cast<unsigned long>(config.notifyBlockTicks));

    if (enable && taskHandle != nullptr) {
        xTaskNotifyGive(taskHandle);
        ESP_LOGI(TAG, "Kicked task after enabling notify mode");
    }
}

bool ControlTask::isNotifyDriven() const noexcept {
    return config.notifyDriven;
}

TaskHandle_t ControlTask::getTaskHandle() const noexcept {
    return taskHandle;
}

void ControlTask::taskFunction(void* param) {
    auto* self = static_cast<ControlTask*>(param);
    self->runLoop();
}

void ControlTask::runLoop() {
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t localHz = config.updateHz;
    if (localHz == 0) {
        localHz = 1;
    }

    TickType_t period = pdMS_TO_TICKS(1000 / localHz);
    if (period < 1) {
        period = 1;
    }

    ESP_LOGI(TAG, "Task loop start (mode=%s, periodTicks=%lu)",
             config.notifyDriven ? "notify" : "poll",
             static_cast<unsigned long>(period));

    while (!stopRequested.load(std::memory_order_relaxed)) {
        if (config.notifyDriven) {
            uint32_t taken = ulTaskNotifyTake(pdTRUE, config.notifyBlockTicks);
            if (taken == 0) {
                continue;
            }
            if (stopRequested.load(std::memory_order_relaxed)) {
                break;
            }
            if (updateCallback) {
                updateCallback();
            }
        } else {
            if (updateCallback) {
                updateCallback();
            }
            vTaskDelayUntil(&lastWake, period);
        }
    }

    ESP_LOGI(TAG, "Task loop exiting");
    running.store(false, std::memory_order_relaxed);
    if (exitSemaphore != nullptr) {
        xSemaphoreGive(exitSemaphore);
    }
    vTaskDelete(nullptr);
}

}  // namespace DC_Motor_Controller_Firmware::Control
