/**
 * @file MotorController.hpp
 * @brief High-level closed-loop controller for positioning a DC motor.
 *
 * Combines encoder feedback, a DRV8876 motor driver, and a PID controller to
 * move a DC motor to a target position with configurable motion parameters.
 * Delegates motion profiling, settle detection, stall detection, soft limits,
 * and motion guarding to dedicated sub-components.
 */

#pragma once

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "MotionGuard.hpp"
#include "MotionProfiler.hpp"
#include "PID.hpp"
#include "SettleDetector.hpp"
#include "SoftLimiter.hpp"
#include "StallDetector.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <atomic>
#include <cmath>
#include <cstdint>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @struct MotorControllerConfig
 * @brief Configuration structure for motion constraints and sub-component parameters.
 */
struct MotorControllerConfig {
    float minSpeed = 2.0f;           ///< Minimum allowed motor speed (%).
    float maxSpeed = 100.0f;         ///< Maximum allowed motor speed (%).
    float minErrorToMove = 0.2f;     ///< Minimum |error| to trigger motion (deg).
    int countsPerRevolution = 1024;  ///< Encoder counts per full revolution (ticks/turn).

    float Kff_pos = 0.0f;  ///< Position feed-forward gain; maps reference position (deg) to percent command.
    float Kff_vel = 0.0f;  ///< Velocity feed-forward gain; maps reference velocity (signed %, profiled) to percent command.

    MotionProfilerConfig profiler;  ///< Motion profile shaping configuration.
    SettleDetectorConfig settle;    ///< Settle detection configuration.
    StallDetectorConfig stall;      ///< Stall/stuck detection configuration.
    MotionGuardConfig guard;        ///< Motion timeout and drift wake-up configuration.
};

/**
 * @struct MotorStatus
 * @brief Snapshot of controller state for monitoring and debugging.
 *
 * Units:
 * - target / position / error: degrees
 * - velocity: degrees per second (estimated)
 * - pidOutput: raw PID output (same scale as used by motor command mapping)
 * - stuckCount: update-cycle count accumulated by stuck detection
 * - motionDone: true if controller considers motion settled/complete
 */
struct MotorStatus {
    float target;     ///< Current target position (deg).
    float position;   ///< Current measured position (deg).
    float error;      ///< target - position (deg).
    float velocity;   ///< Estimated angular velocity (deg/s).
    float pidOutput;  ///< Last computed PID output value.
    int stuckCount;   ///< Current stuck counter value.
    bool motionDone;  ///< True if motion is complete.
};

/**
 * @class MotorController
 * @brief Controls a DC motor using encoder feedback and PID regulation.
 *
 * Provides high-level closed-loop control by coordinating encoder feedback,
 * PID + feed-forward, motion profile shaping, and motor driver commands.
 * Delegates sub-tasks to MotionProfiler, SettleDetector, StallDetector,
 * SoftLimiter, and MotionGuard sub-components.
 */
class MotorController {
  public:
    /**
     * @brief Constructor for the motor controller.
     *
     * @param enc Reference to Encoder instance for position feedback.
     * @param drv Reference to DRV8876 motor driver for motor commands.
     * @param pid Reference to PID controller for error-driven regulation.
     * @param cfg Optional control parameters (default config used if omitted).
     */
    MotorController(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pid, const MotorControllerConfig& cfg = MotorControllerConfig());

    /**
     * @brief Destructor.
     *
     * If the control task is running, it will be deleted automatically.
     * Stops the motor and releases mutex resources.
     */
    ~MotorController();

    /**
     * @brief Move constructor.
     *
     * Transfers ownership of FreeRTOS resources (mutexes, task handle)
     * from the source. The source is left in a safe but unusable state.
     *
     * @param other Source controller to move from.
     */
    MotorController(MotorController&& other) noexcept;

    /** @brief Copy constructor (deleted). */
    MotorController(const MotorController&) = delete;

    /** @brief Copy assignment operator (deleted). */
    MotorController& operator=(const MotorController&) = delete;

    /** @brief Move assignment operator (deleted). */
    MotorController& operator=(MotorController&&) = delete;

    /**
     * @brief Starts the FreeRTOS task that handles periodic PID-based motor control.
     *
     * Creates a background task using xTaskCreatePinnedToCore() that continuously
     * calls update(). If the task is already running, this method does nothing.
     */
    void startTask();

    /**
     * @brief Stop the background control task if running.
     *
     * Deletes the FreeRTOS task created by startTask(). If no task is
     * running, this method does nothing.
     */
    void stopTask();

    /**
     * @brief Immediately stop the motor and mark motion as done.
     *
     * Stops the motor driver, resets all sub-component state,
     * and sets motionDone to true.
     */
    void stop();

    /**
     * @brief Set the desired target position in degrees.
     *
     * Clamps to soft limits if enforced, resets PID and sub-component state,
     * and begins a new motion command.
     *
     * @param degrees New target angle (deg).
     */
    void setTargetDegrees(float degrees);

    /**
     * @brief Set the desired target position in encoder ticks.
     *
     * Converts ticks to degrees using countsPerRevolution, then delegates
     * to setTargetDegrees().
     *
     * @param ticks Target position in raw encoder ticks.
     */
    void setTargetTicks(int32_t ticks);

    /**
     * @brief Backward-compatible alias for degree-based target setter.
     * @param degrees New target angle (deg).
     */
    void setTarget(float degrees);

    /**
     * @brief Set PID gain parameters.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    void setPID(float kp, float ki, float kd);

    /**
     * @brief Get the current PID parameters.
     * @param kp Output: proportional gain.
     * @param ki Output: integral gain.
     * @param kd Output: derivative gain.
     */
    void getPID(float& kp, float& ki, float& kd);

    /**
     * @brief Update the motor controller configuration at runtime.
     *
     * Thread-safe via configMutex.
     *
     * @param cfg New configuration parameters to apply.
     */
    void setConfig(const MotorControllerConfig& cfg);

    /**
     * @brief Get the current motor controller configuration.
     *
     * Thread-safe via configMutex.
     *
     * @return Current MotorControllerConfig structure.
     */
    MotorControllerConfig getConfig() const;

    /**
     * @brief Run the control logic once.
     *
     * Executes the full control pipeline: read encoder, PID compute,
     * feed-forward, soft limit check, speed mapping, profile shaping,
     * settle/stall detection, slew-rate limiting, and motor command.
     * Should be called periodically if not using a background task.
     */
    void update();

    /**
     * @brief Check if the motor has reached its target and settled.
     * @return true if motion is complete, false otherwise.
     */
    bool isMotionDone() const;

    /**
     * @brief Configure the update loop frequency for the control task.
     * @param hz Desired update rate in Hertz (must be > 0).
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
     * @param status The MotorStatus struct to log.
     */
    void logMotorStatus(const MotorStatus& status);

    /**
     * @brief Set angular soft limits with optional enforcement.
     *
     * Delegates to SoftLimiter. If enforced, the current target is clamped
     * and setTargetDegrees() is called if it changed.
     *
     * @param minDeg Minimum allowed mechanical angle (deg).
     * @param maxDeg Maximum allowed mechanical angle (deg).
     * @param enforce Enable enforcement immediately if true (default: true).
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
     * @param enable True to enable, false to disable.
     */
    void enableMotionProfile(bool enable);

    /**
     * @brief Configure motion profile parameters at runtime.
     *
     * @param type Profile type (trapezoid or S-curve).
     * @param accelPctPerSec Acceleration limit in %/s.
     * @param jerkPctPerSec2 Jerk limit in %/s^2 (used for S-curve; ignored for trapezoid).
     */
    void configureMotionProfile(MotionProfileType type, float accelPctPerSec, float jerkPctPerSec2);

    /**
     * @brief Set feed-forward gains used as: u_ff = Kff_pos*ref + Kff_vel*refDot.
     *
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
     *
     * When enabled, the task calls ulTaskNotifyTake() and runs update() after each notify.
     *
     * @param enable True to enable notify-driven updates, false to use tick polling.
     * @param maxBlockTicks Maximum ticks to block waiting for a notify (use portMAX_DELAY for indefinite).
     */
    void enableNotifyDrivenUpdates(bool enable, TickType_t maxBlockTicks = portMAX_DELAY);

    /**
     * @brief Notify the control task from an ISR to run one update cycle.
     * @param higherPrioTaskWoken Optional pointer for ISR context switch request (pass from ISR).
     */
    void notifyControlTaskFromISR(BaseType_t* higherPrioTaskWoken);

    /**
     * @brief User callback signature for controller events.
     * @param status Current snapshot of controller state.
     * @param user Opaque user pointer passed at registration.
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
    /** @brief Encoder feedback interface for position measurement. */
    Encoder::Encoder& encoder;

    /** @brief Motor driver interface for direction and speed commands. */
    DRV8876::DRV8876& motor;

    /** @brief PID regulator instance for error-driven control output. */
    PID::PIDController& pid;

    /** @brief User-defined motion parameters and sub-component configurations. */
    MotorControllerConfig config;

    /** @brief Sub-component for trapezoid/S-curve speed shaping and slew-rate limiting. */
    MotionProfiler profiler;

    /** @brief Sub-component for detecting position and velocity settlement. */
    SettleDetector settleDetector;

    /** @brief Sub-component for stuck/stall detection with PID warmup gating. */
    StallDetector stallDetector;

    /** @brief Sub-component for soft mechanical angle limit enforcement. */
    SoftLimiter softLimiter;

    /** @brief Sub-component for motion timeout and idle drift wake-up. */
    MotionGuard motionGuard;

    /** @brief Target position in degrees, protected by targetMutex. */
    float target = 0;

    /** @brief True if motion is complete; atomic for lock-free reads from any context. */
    std::atomic<bool> motionDone{true};

    /** @brief Last encoder position (deg) used for velocity estimation and stuck detection. */
    float lastPos = 0;

    /** @brief Last computed PID output value cached for status reporting. */
    float lastPidOut = 0.0f;

    /** @brief Last estimated angular velocity (deg/s) cached for status reporting. */
    float lastVelDegPerSec = 0.0f;

    /** @brief Mutex protecting the target variable for thread-safe access. */
    SemaphoreHandle_t targetMutex = nullptr;

    /** @brief Mutex protecting the config structure for thread-safe access. */
    SemaphoreHandle_t configMutex = nullptr;

    /** @brief FreeRTOS task handle for the optional background control task. */
    TaskHandle_t taskHandle = nullptr;

    /** @brief Background control update rate in Hertz. */
    uint32_t updateHz = 100;

    /** @brief Core affinity for xTaskCreatePinnedToCore(); tskNO_AFFINITY for no pinning. */
    BaseType_t controlTaskCoreId = tskNO_AFFINITY;

    /** @brief FreeRTOS priority for the control task. */
    UBaseType_t controlTaskPriority = 10;

    /** @brief If true, the control task waits on ulTaskNotifyTake() instead of polling. */
    bool notifyDriven = false;

    /** @brief Max ticks to block waiting for a task notification. */
    TickType_t notifyBlockTicks = portMAX_DELAY;

    /** @brief Callback function pointer invoked when motion settles (done). */
    MotionEventCallback onMotionDoneCb = nullptr;

    /** @brief Opaque user pointer passed to the onMotionDone callback. */
    void* onMotionDoneUser = nullptr;

    /** @brief Callback function pointer invoked when a stall is detected. */
    MotionEventCallback onStallCb = nullptr;

    /** @brief Opaque user pointer passed to the onStall callback. */
    void* onStallUser = nullptr;

    /** @brief Callback function pointer invoked when a soft limit is hit. */
    MotionEventCallback onLimitHitCb = nullptr;

    /** @brief Opaque user pointer passed to the onLimitHit callback. */
    void* onLimitHitUser = nullptr;

    /** @brief Previous clamped value for log suppression in clampToPercentRange(). */
    mutable float clampPreviousValue = NAN;

    /** @brief Timestamp (us) for periodic PID logging throttle. */
    uint64_t lastPidLogUs = 0;

    /** @brief Timestamp (us) for periodic state logging throttle. */
    uint64_t lastStateLogUs = 0;

    /** @brief Timestamp (us) for periodic command logging throttle. */
    uint64_t lastCmdLogUs = 0;

    /** @brief Timestamp (us) for periodic status logging throttle. */
    uint64_t lastStatusLogUs = 0;

    /** @brief Log tag for ESP-IDF logging. */
    static constexpr const char* TAG = "MotorController";

    /**
     * @brief Clamp a PID output signal expressed in percent to [-100, 100].
     *
     * Logs a warning when clamping occurs (suppressed for repeated values).
     *
     * @param signal Raw PID output signal (percent semantics).
     * @return Clamped output in [-100, 100].
     */
    float clampToPercentRange(float signal) const;

    /**
     * @brief Reset all sub-component state for a new motion command or stop.
     *
     * Calls reset() on profiler, settleDetector, stallDetector, and motionGuard.
     *
     * @param nowUs Current timestamp in microseconds (passed to profiler.reset()).
     */
    void resetMotionState(uint64_t nowUs);

    /**
     * @brief Static task entry point for FreeRTOS task created by startTask().
     *
     * Runs an infinite loop calling update() either periodically (poll mode)
     * or on task notification (notify mode).
     *
     * @param param Pointer to the MotorController instance.
     */
    static void taskFunc(void* param);
};

}  // namespace DC_Motor_Controller_Firmware::Control
