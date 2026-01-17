/**
 * @file ControlTask.hpp
 * @brief FreeRTOS task wrapper for motor control timing.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <atomic>
#include <cstdint>
#include <functional>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @struct ControlTaskConfig
 * @brief Configuration for control task scheduling.
 */
struct ControlTaskConfig {
    uint32_t updateHz = 100;                        ///< Update rate (Hz)
    BaseType_t coreId = tskNO_AFFINITY;             ///< Core affinity (-1 = any)
    UBaseType_t priority = 10;                      ///< Task priority
    uint32_t stackSize = 4096;                      ///< Stack size (bytes)
    bool notifyDriven = false;                      ///< Use task notifications vs polling
    TickType_t notifyBlockTicks = portMAX_DELAY;    ///< Max ticks to block on notify
};

/**
 * @brief Callback type for the update function.
 */
using UpdateCallback = std::function<void()>;

/**
 * @class ControlTask
 * @brief Owns the FreeRTOS task for periodic control execution.
 */
class ControlTask {
  public:
    /**
     * @brief Construct control task.
     * @param cfg Task configuration
     */
    explicit ControlTask(const ControlTaskConfig& cfg = ControlTaskConfig());

    /**
     * @brief Destructor. Stops task if running.
     */
    ~ControlTask();

    ControlTask(const ControlTask&) = delete;
    ControlTask& operator=(const ControlTask&) = delete;
    ControlTask(ControlTask&&) = delete;
    ControlTask& operator=(ControlTask&&) = delete;

    /**
     * @brief Set the update callback function.
     * @param cb Function to call on each update cycle
     */
    void setUpdateCallback(UpdateCallback cb);

    /**
     * @brief Start the control task.
     * @return true if started successfully
     */
    bool start();

    /**
     * @brief Stop the control task.
     * @param timeoutMs Timeout to wait for graceful exit
     * @return true if stopped successfully
     */
    bool stop(uint32_t timeoutMs = 2000);

    /**
     * @brief Check if task is running.
     * @return True if running
     */
    bool isRunning() const noexcept;

    /**
     * @brief Notify task from ISR to run one cycle.
     * @param higherPrioTaskWoken Output for context switch
     */
    void notifyFromISR(BaseType_t* higherPrioTaskWoken);

    /**
     * @brief Notify task from normal context.
     */
    void notify();

    /**
     * @brief Update configuration (takes effect on next start).
     * @param cfg New configuration
     */
    void setConfig(const ControlTaskConfig& cfg);

    /**
     * @brief Get current configuration.
     * @return Current configuration
     */
    ControlTaskConfig getConfig() const noexcept;

    /**
     * @brief Set update rate.
     * @param hz Update frequency in Hz
     */
    void setUpdateHz(uint32_t hz);

    /**
     * @brief Get update rate.
     * @return Update frequency in Hz
     */
    uint32_t getUpdateHz() const noexcept;

    /**
     * @brief Configure core affinity and priority.
     * @param coreId Core index or tskNO_AFFINITY
     * @param priority Task priority
     */
    void configureTask(int coreId, UBaseType_t priority);

    /**
     * @brief Get configured core ID.
     * @return Core ID or tskNO_AFFINITY
     */
    int getCoreId() const noexcept;

    /**
     * @brief Get configured priority.
     * @return Task priority
     */
    UBaseType_t getPriority() const noexcept;

    /**
     * @brief Enable notify-driven updates.
     * @param enable True for notify mode, false for polling
     * @param maxBlockTicks Max ticks to block waiting
     */
    void enableNotifyDrivenUpdates(bool enable, TickType_t maxBlockTicks = portMAX_DELAY);

    /**
     * @brief Check if notify-driven mode is enabled.
     * @return True if notify-driven
     */
    bool isNotifyDriven() const noexcept;

    /**
     * @brief Get task handle (for diagnostics).
     * @return Task handle or nullptr
     */
    TaskHandle_t getTaskHandle() const noexcept;

  private:
    /**
     * @brief Static task entry point.
     * @param param ControlTask instance
     */
    static void taskFunction(void* param);

    /**
     * @brief Task main loop.
     */
    void runLoop();

    ControlTaskConfig config;                      ///< Configuration
    UpdateCallback updateCallback;                 ///< User callback
    TaskHandle_t taskHandle = nullptr;             ///< FreeRTOS task handle
    SemaphoreHandle_t exitSemaphore = nullptr;     ///< Exit signaling
    std::atomic<bool> stopRequested{false};        ///< Stop flag
    std::atomic<bool> running{false};              ///< Running state
};

}  // namespace DC_Motor_Controller_Firmware::Control
