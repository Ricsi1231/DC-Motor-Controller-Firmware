/**
 * @file MotorControl.hpp
 * @brief High-level closed-loop controller for positioning a DC motor.
 *
 * Combines encoder feedback, a DRV8876 motor driver, and a PID controller to
 * move a DC motor to a target position with configurable motion parameters.
 */

#pragma once

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "FuzzyPIDController.hpp"
#include "MotionDetector.hpp"
#include "MotionProfile.hpp"
#include "MotorControlConfig.hpp"
#include "PID.hpp"
#include "SoftLimits.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <atomic>
#include <cstdint>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @class MotorControl
 * @brief Controls a DC motor using encoder feedback and PID regulation.
 *
 * Provides high-level closed-loop control by coordinating encoder feedback,
 * PID + feed-forward, motion profile shaping, and motor driver commands.
 */
class MotorControl {
  public:
    /**
     * @brief Constructor for the motor controller.
     * @param enc Reference to Encoder instance
     * @param drv Reference to DRV8876 motor driver
     * @param pid Reference to PID controller
     * @param cfg Optional control parameters (default config used if omitted)
     */
    MotorControl(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pid,
                 const MotorControlConfig& cfg = MotorControlConfig());

    /**
     * @brief Constructor with fuzzy PID controller.
     * @param enc Reference to Encoder instance
     * @param drv Reference to DRV8876 motor driver
     * @param pid Reference to PID controller
     * @param fuzzy Reference to Fuzzy PID controller
     * @param cfg Optional control parameters
     */
    MotorControl(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pid,
                 PID::FuzzyPIDController& fuzzy, const MotorControlConfig& cfg = MotorControlConfig());

    /**
     * @brief Destructor. Stops task and motor if running.
     */
    ~MotorControl();

    MotorControl(MotorControl&& other) noexcept;
    MotorControl(const MotorControl&) = delete;
    MotorControl& operator=(const MotorControl&) = delete;
    MotorControl& operator=(MotorControl&&) = delete;

    /**
     * @brief Starts the FreeRTOS task that handles periodic PID-based motor control.
     */
    void startTask();

    /**
     * @brief Stop the background control task if running.
     */
    void stopTask();

    /**
     * @brief Immediately stop the motor and mark motion as done.
     */
    void stop();

    /**
     * @brief Set the desired target position in degrees.
     * @param degrees New target angle (deg)
     */
    void setTargetDegrees(float degrees);

    /**
     * @brief Set the desired target position in encoder ticks.
     * @param ticks Target position in raw encoder ticks
     */
    void setTargetTicks(int32_t ticks);

    /**
     * @brief Backward-compatible alias for degree-based target setter.
     * @param degrees New target angle (deg)
     */
    void setTarget(float degrees);

    /**
     * @brief Set PID gain parameters.
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPID(float kp, float ki, float kd);

    /**
     * @brief Get the current PID parameters.
     * @param kp Output: proportional gain
     * @param ki Output: integral gain
     * @param kd Output: derivative gain
     */
    void getPID(float& kp, float& ki, float& kd);

    /**
     * @brief Update the motor controller configuration at runtime.
     * @param cfg New configuration parameters to apply
     */
    void setConfig(const MotorControlConfig& cfg);

    /**
     * @brief Get the current motor controller configuration.
     * @return Current MotorControlConfig structure
     */
    MotorControlConfig getConfig() const;

    /**
     * @brief Run the control logic once. Should be called periodically if not using a task.
     */
    void update();

    /**
     * @brief Check if the motor has reached its target and settled.
     * @return true if motion is complete, false otherwise
     */
    bool isMotionDone() const;

    /**
     * @brief Configure the update loop frequency for the control task.
     * @param hz Desired update rate in Hertz (must be > 0)
     */
    void setUpdateHz(uint32_t hz);

    /**
     * @brief Get a snapshot of the current motor controller state.
     * @return MotorStatus containing target, position, error, velocity,
     *         last PID output, stuck counter, and motion completion flag.
     */
    MotorStatus getStatus() const;

    /**
     * @brief Log the motor status information using ESP-IDF logging at INFO level.
     * @param status The MotorStatus struct to log
     */
    void logMotorStatus(const MotorStatus& status);

    /**
     * @brief Set angular soft limits with optional enforcement.
     * @param minDeg Minimum allowed mechanical angle (deg)
     * @param maxDeg Maximum allowed mechanical angle (deg)
     * @param enforce Enable enforcement immediately if true (default: true)
     */
    void setSoftLimits(float minDeg, float maxDeg, bool enforce = true);

    /**
     * @brief Set per-command motion timeout in milliseconds.
     * @param ms Timeout in milliseconds; 0 disables the guard.
     */
    void setMotionTimeoutMs(uint32_t ms);

    /**
     * @brief Get the configured per-command motion timeout in milliseconds.
     * @return Timeout in milliseconds; 0 if disabled.
     */
    uint32_t getMotionTimeoutMs() const;

    /**
     * @brief Enable or disable motion profile shaping at runtime.
     * @param enable True to enable, false to disable
     */
    void enableMotionProfile(bool enable);

    /**
     * @brief Configure motion profile parameters at runtime.
     * @param type Profile type (trapezoid or S-curve).
     * @param accelPctPerSec Acceleration limit in %/s.
     * @param jerkPctPerSec2 Jerk limit in %/s^2 (used for S-curve; ignored for trapezoid).
     */
    void configureMotionProfile(MotionProfileType type, float accelPctPerSec, float jerkPctPerSec2);

    /**
     * @brief Set feed-forward gains used as: u_ff = Kff_pos*ref + Kff_vel*refDot.
     * @param kpos Position feed-forward gain (percent per degree).
     * @param kvel Velocity feed-forward gain (percent per percent-of-speed, signed).
     */
    void setFeedForward(float kpos, float kvel);

    /**
     * @brief Get the current feed-forward gains.
     * @param kpos Out: position feed-forward gain.
     * @param kvel Out: velocity feed-forward gain.
     */
    void getFeedForward(float& kpos, float& kvel) const;

    /**
     * @brief Set the number of PID warmup cycles to skip settle/stuck decisions.
     * @param cycles Count of update cycles; must be >= 0.
     */
    void setPidWarmupCycles(int cycles);

    /**
     * @brief Get the configured number of PID warmup cycles.
     * @return Warmup cycle count (>= 0).
     */
    int getPidWarmupCycles() const;

    /**
     * @brief Configure the control task's core affinity and priority used by startTask().
     * @param coreId CPU core index (e.g., 0 or 1) or tskNO_AFFINITY (-1) for no pinning.
     * @param priority FreeRTOS task priority (>= 1).
     */
    void configureControlTask(int coreId, UBaseType_t priority);

    /**
     * @brief Get the configured core affinity for the control task.
     * @return Core index or tskNO_AFFINITY (-1).
     */
    int getControlTaskCore() const;

    /**
     * @brief Get the configured FreeRTOS priority for the control task.
     * @return Task priority (>= 1).
     */
    UBaseType_t getControlTaskPriority() const;

    /**
     * @brief Enable ISR/timer-driven updates so the control task blocks on notifications.
     * @param enable True to enable notify-driven updates, false to use tick polling.
     * @param maxBlockTicks Maximum ticks to block waiting for a notify.
     */
    void enableNotifyDrivenUpdates(bool enable, TickType_t maxBlockTicks = portMAX_DELAY);

    /**
     * @brief Notify the control task from an ISR to run one update cycle.
     * @param higherPrioTaskWoken Optional pointer for ISR context switch request.
     */
    void notifyControlTaskFromISR(BaseType_t* higherPrioTaskWoken);

    /**
     * @brief User callback signature for controller events.
     */
    using MotionEventCallback = void (*)(const MotorStatus& status, void* user);

    /**
     * @brief Register callback fired once when motion is declared done.
     * @param cb Function pointer or nullptr to clear.
     * @param user Opaque pointer passed back on callback.
     */
    void setOnMotionDone(MotionEventCallback cb, void* user = nullptr);

    /**
     * @brief Register callback fired when stall is detected.
     * @param cb Function pointer or nullptr to clear.
     * @param user Opaque pointer passed back on callback.
     */
    void setOnStall(MotionEventCallback cb, void* user = nullptr);

    /**
     * @brief Register callback fired when a soft limit is hit.
     * @param cb Function pointer or nullptr to clear.
     * @param user Opaque pointer passed back on callback.
     */
    void setOnLimitHit(MotionEventCallback cb, void* user = nullptr);

  private:
    Encoder::Encoder& encoder;      ///< Encoder feedback interface
    DRV8876::DRV8876& motor;        ///< Motor driver interface
    PID::PIDController& pid;        ///< PID regulator instance
    PID::FuzzyPIDController fuzzy;  ///< Fuzzy-PID scheduler instance
    MotorControlConfig config;      ///< User-defined motion parameters

    MotionProfile profile;    ///< Motion profile generator component
    MotionDetector detector;  ///< Motion settle/stall detector component
    SoftLimits limits;        ///< Soft position limits component

    bool useFuzzy = false;  ///< True if using fuzzy PID controller

    float target = 0;                         ///< Target position in degrees
    std::atomic<bool> motionDone{true};       ///< True if motion is complete
    float lastPos = 0;                        ///< Last encoder position (deg)
    SemaphoreHandle_t targetMutex = nullptr;  ///< Mutex protecting target
    SemaphoreHandle_t configMutex = nullptr;  ///< Mutex protecting config

    TaskHandle_t taskHandle = nullptr;              ///< Optional background control task
    uint32_t updateHz = 100;                        ///< Background control update rate (Hz)
    volatile bool taskStopRequested = false;        ///< Flag to signal task to exit gracefully
    SemaphoreHandle_t taskExitSemaphore = nullptr;  ///< Semaphore signaled when task exits

    float lastPidOut = 0.0f;        ///< Last computed PID output (for status)
    float lastVelDegPerSec = 0.0f;  ///< Last estimated angular velocity (deg/s)

    uint64_t motionStartUs = 0;  ///< Motion start timestamp (us) for timeout guard

    BaseType_t controlTaskCoreId = tskNO_AFFINITY;  ///< Core affinity for xTaskCreatePinnedToCore()
    UBaseType_t controlTaskPriority = 10;           ///< FreeRTOS priority for the control task

    bool notifyDriven = false;                    ///< If true, task waits on ulTaskNotifyTake()
    TickType_t notifyBlockTicks = portMAX_DELAY;  ///< Max ticks to block waiting for notify

    MotionEventCallback onMotionDoneCb = nullptr;  ///< Motion done callback function
    void* onMotionDoneUser = nullptr;              ///< User data for motion done callback
    MotionEventCallback onStallCb = nullptr;       ///< Stall detection callback function
    void* onStallUser = nullptr;                   ///< User data for stall callback
    MotionEventCallback onLimitHitCb = nullptr;    ///< Limit hit callback function
    void* onLimitHitUser = nullptr;                ///< User data for limit hit callback

    uint64_t lastPidLogUs = 0;     ///< Timestamp (us) of last PID log output
    uint64_t lastStateLogUs = 0;   ///< Timestamp (us) of last state log output
    uint64_t lastCmdLogUs = 0;     ///< Timestamp (us) of last command log output
    uint64_t lastStatusLogUs = 0;  ///< Timestamp (us) of last status log output

    float slewOutPct = 0.0f;   ///< Current slew-limited output (%)
    uint64_t lastSlewUs = 0;   ///< Timestamp (us) of last slew rate update

    mutable float clampPrevValue = NAN;  ///< Previous clamped value for log deduplication

    static constexpr const char* TAG = "MotorControl";  ///< Log tag

    /**
     * @brief Synchronize component configurations from main config.
     */
    void syncComponentsFromConfig();

    /**
     * @brief Clamp a PID output signal to [-100, 100].
     * @param signal Raw PID output signal (percent semantics)
     * @return Clamped output in [-100, 100]
     */
    float clampToPercentRange(float signal) const;

    /**
     * @brief Handle idle state when motion is done.
     */
    void handleIdleState(float currentTarget, float currentPos, uint64_t nowUs);

    /**
     * @brief Check and handle motion timeout.
     * @return True if timeout occurred and motion was stopped.
     */
    bool checkTimeout(const MotorControlConfig& cfg, uint64_t nowUs);

    /**
     * @brief Compute control output from PID and feed-forward.
     * @return Combined control signal in percent.
     */
    float computeControl(const MotorControlConfig& cfg, float currentTarget, float currentPos, uint64_t nowUs);

    /**
     * @brief Check and handle soft limit violations.
     * @return True if at limit and motion was stopped.
     */
    bool checkLimits(float combined, float currentPos);

    /**
     * @brief Apply min/max speed limits and profile shaping.
     * @return Profiled speed command.
     */
    float applySpeedLimits(const MotorControlConfig& cfg, float combined, float currentTarget,
                           float currentPos, uint64_t nowUs);

    /**
     * @brief Check if PID controller reports settled.
     * @return True if PID is settled.
     */
    bool checkPidSettled(const MotorControlConfig& cfg);

    /**
     * @brief Handle motion settled state.
     */
    void handleSettled();

    /**
     * @brief Handle stall condition.
     */
    void handleStall(const MotorControlConfig& cfg, float currentTarget, float currentPos);

    /**
     * @brief Apply slew-limited motor command.
     */
    void applyMotorCommand(const MotorControlConfig& cfg, float profiled, uint64_t nowUs);

    /**
     * @brief Static task entry point for FreeRTOS task.
     * @param param Pointer to MotorControl instance
     */
    static void taskFunc(void* param);
};

}  // namespace DC_Motor_Controller_Firmware::Control
