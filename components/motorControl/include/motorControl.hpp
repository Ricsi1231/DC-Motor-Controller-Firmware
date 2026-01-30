/**
 * @file MotorController.hpp
 * @brief High-level closed-loop controller for positioning a DC motor.
 *
 * Combines encoder feedback, a DRV8876 motor driver, and a PID controller to
 * move a DC motor to a target position with configurable motion parameters.
 */

#pragma once

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "PID.hpp"
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
 * @enum MotionProfileType
 * @brief Selects the motion profile shaping strategy.
 */
enum class MotionProfileType {
    TRAPEZOID,  ///< Acceleration-limited (rate-limited speed), no explicit jerk limit.
    S_CURVE     ///< Acceleration with jerk limit for smoother ramps.
};

/**
 * @struct MotorControllerConfig
 * @brief Configuration structure for motion constraints and drift/stuck detection.
 */
struct MotorControllerConfig {
    float minSpeed = 2.0f;        ///< Minimum allowed motor speed (%).
    float maxSpeed = 100.0f;      ///< Maximum allowed motor speed (%).
    float minErrorToMove = 0.2f;  ///< Minimum |error| to trigger motion (deg).

    float driftThreshold = 1.0f;   ///< Legacy drift threshold (deg); not used for wake-up; see driftDeadband/driftHysteresis.
    float driftDeadband = 0.5f;    ///< Allowed steady-state drift band (deg); ignore drift within this when settled.
    float driftHysteresis = 0.5f;  ///< Extra margin (deg) added to deadband for wake-up: |drift| > deadband + hysteresis.

    float stuckPositionEpsilon = 0.05f;  ///< Movement epsilon for stuck detection (deg); count stuck only if |error| > minErrorToMove AND |Δpos| < epsilon.
    int stuckCountLimit = 50;            ///< Update cycles before declaring “stuck”.
    int pidWarmupLimit = 10;             ///< Cycles allowed for PID to stabilize before drift/stuck checks.
    int countsPerRevolution = 1024;      ///< Encoder counts per full revolution (ticks/turn).

    int motionTimeoutMs = 0;  ///< Max allowed motion time per command (ms); 0 = disabled.

    bool motionProfileEnabled = false;                                   ///< Enable motion profile shaping (trapezoid or S-curve) on speed command.
    MotionProfileType motionProfileType = MotionProfileType::TRAPEZOID;  ///< Profile type selection.

    float accelLimitPctPerSec = 300.0f;   ///< Max acceleration of commanded speed (% per second).
    float jerkLimitPctPerSec2 = 6000.0f;  ///< Max jerk for S-curve (% per second^2); ignored for trapezoid.

    float Kff_pos = 0.0f;  ///< Position feed-forward gain; maps reference position (deg) to percent command.
    float Kff_vel = 0.0f;  ///< Velocity feed-forward gain; maps reference velocity (signed %, profiled) to percent command.

    float settlePosTolDeg = 0.3f;        ///< Settle position tolerance (deg).
    float settleVelTolDegPerSec = 1.0f;  ///< Settle velocity tolerance (deg/s).
    int settleCountLimit = 5;            ///< Consecutive cycles required to declare settled.
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
    float target;     ///< Current target position (deg)
    float position;   ///< Current measured position (deg)
    float error;      ///< target - position (deg)
    float velocity;   ///< Estimated angular velocity (deg/s)
    float pidOutput;  ///< Last computed PID output value
    int stuckCount;   ///< Current stuck counter value
    bool motionDone;  ///< True if motion is complete
};

/**
 * @class MotorController
 * @brief Controls a DC motor using encoder feedback and PID regulation.
 *
 * Provides high-level closed-loop control by coordinating encoder feedback,
 * PID + feed-forward, motion profile shaping, and motor driver commands.
 */
class MotorController {
  public:
    /**
     * @brief Constructor for the motor controller.
     *
     * @param enc Reference to Encoder instance
     * @param drv Reference to DRV8876 motor driver
     * @param pid Reference to PID controller
     * @param cfg Optional control parameters (default config used if omitted)
     */
    MotorController(Encoder::Encoder& enc, DRV8876::DRV8876& drv, PID::PIDController& pid, const MotorControllerConfig& cfg = MotorControllerConfig());

    /**
     * @brief Destructor.
     *
     * If the control task is running, it will be deleted automatically.
     */
    ~MotorController();

    MotorController(MotorController&& other) noexcept;
    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;
    MotorController& operator=(MotorController&&) = delete;

    /**
     * @brief Starts the FreeRTOS task that handles periodic PID-based motor control.
     *
     * Creates a background task using `xTaskCreate()` that continuously calls
     * `update()`. If the task is already running, this method does nothing.
     */
    void startTask();

    /**
     * @brief Stop the background control task if running.
     *
     * Deletes the FreeRTOS task created by `startTask()`. If no task is
     * running, this method does nothing.
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
    void setConfig(const MotorControllerConfig& cfg);

    /**
     * @brief Get the current motor controller configuration.
     * @return Current MotorControllerConfig structure
     */
    MotorControllerConfig getConfig() const;

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
     * @return @ref MotorStatus containing target, position, error, velocity,
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
     * @brief Configure the control task’s core affinity and priority used by startTask().
     * @param coreId   CPU core index (e.g., 0 or 1) or tskNO_AFFINITY (-1) for no pinning.
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
     *        When enabled, the task calls ulTaskNotifyTake() and runs update() after each notify.
     * @param enable           True to enable notify-driven updates, false to use tick polling.
     * @param maxBlockTicks    Maximum ticks to block waiting for a notify (use portMAX_DELAY for indefinite).
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
     * @param user   Opaque user pointer passed at registration.
     */
    using MotionEventCallback = void (*)(const MotorStatus& status, void* user);

    /**
     * @brief Register callback fired once when motion is declared done.
     * @param cb   Function pointer or nullptr to clear.
     * @param user Opaque pointer passed back on callback.
     */
    void setOnMotionDone(MotionEventCallback cb, void* user = nullptr);

    /**
     * @brief Register callback fired when stall is detected.
     * @param cb   Function pointer or nullptr to clear.
     * @param user Opaque pointer passed back on callback.
     */
    void setOnStall(MotionEventCallback cb, void* user = nullptr);

    /**
     * @brief Register callback fired when a soft limit is hit.
     * @param cb   Function pointer or nullptr to clear.
     * @param user Opaque pointer passed back on callback.
     */
    void setOnLimitHit(MotionEventCallback cb, void* user = nullptr);

  private:
    Encoder::Encoder& encoder;      ///< Encoder feedback interface
    DRV8876::DRV8876& motor;        ///< Motor driver interface
    PID::PIDController& pid;       ///< PID regulator instance
    MotorControllerConfig config;  ///< User-defined motion parameters

    float target = 0;                         ///< Target position in degrees
    std::atomic<bool> motionDone{true};       ///< True if motion is complete
    float lastPos = 0;                        ///< Last encoder position (deg)
    int stuckCounter = 0;                     ///< Count of how long position hasn't changed
    int pidWarmupCounter = 0;                 ///< Warm-up period before drift/stuck detection
    SemaphoreHandle_t targetMutex = nullptr;  ///< Mutex protecting @ref target
    SemaphoreHandle_t configMutex = nullptr;  ///< Mutex protecting @ref config

    TaskHandle_t taskHandle = nullptr;  ///< Optional background control task
    uint32_t updateHz = 100;            ///< Background control update rate (Hz)

    float lastPidOut = 0.0f;        ///< Last computed PID output (for status)
    float lastVelDegPerSec = 0.0f;  ///< Last estimated angular velocity (deg/s) for status

    bool softLimitsEnforced = false;  ///< Enforce [softMinDeg, softMaxDeg]
    float softMinDeg = 0.0f;          ///< Minimum angle (deg) when enforcement is on
    float softMaxDeg = 0.0f;          ///< Maximum angle (deg) when enforcement is on

    uint64_t motionStartUs = 0;  ///< Motion start timestamp (us) for timeout guard

    float profSpeedPercent = 0.0f;    ///< Current profiled speed command (%), signed
    float profAccelPctPerSec = 0.0f;  ///< Current acceleration state for S-curve (%/s)
    uint64_t lastProfileUs = 0;       ///< Timestamp of last profile update (us)

    int settleCounter = 0;  ///< Consecutive settle-cycle counter.

    BaseType_t controlTaskCoreId = tskNO_AFFINITY;  ///< Core affinity for xTaskCreatePinnedToCore().
    UBaseType_t controlTaskPriority = 10;           ///< FreeRTOS priority for the control task.

    bool notifyDriven = false;                    ///< If true, task waits on ulTaskNotifyTake() for updates.
    TickType_t notifyBlockTicks = portMAX_DELAY;  ///< Max ticks to block waiting for notify.

    MotionEventCallback onMotionDoneCb = nullptr;
    void* onMotionDoneUser = nullptr;

    MotionEventCallback onStallCb = nullptr;
    void* onStallUser = nullptr;

    MotionEventCallback onLimitHitCb = nullptr;
    void* onLimitHitUser = nullptr;

    mutable float clampPreviousValue = NAN;  ///< Previous clamped value for log suppression

    uint64_t lastPidLogUs = 0;     ///< Timestamp for periodic PID logging (us)
    uint64_t lastStateLogUs = 0;   ///< Timestamp for periodic state logging (us)
    uint64_t lastCmdLogUs = 0;     ///< Timestamp for periodic command logging (us)
    uint64_t lastStatusLogUs = 0;  ///< Timestamp for periodic status logging (us)

    float slewLastOutPct = 0.0f;  ///< Previous slew-rate-limited output (%)
    uint64_t slewLastUs = 0;      ///< Timestamp of last slew-rate update (us)

    static constexpr const char* TAG = "MotorController";  ///< Log tag

    /**
     * @brief Clamp a PID output signal expressed in percent to [-100, 100].
     * @param signal Raw PID output signal (percent semantics)
     * @return Clamped output in [-100, 100]
     */
    float clampToPercentRange(float signal) const;

    /**
     * @brief Apply trapezoid/S-curve shaping to the desired speed command.
     *
     * @param desiredSigned Target speed command in percent (signed).
     * @param nowUs Current timestamp in microseconds.
     * @return Profiled speed command in percent (signed).
     */
    float shapeSpeedWithProfile(float desiredSigned, uint64_t nowUs);

    /**
     * @brief Static task entry point for FreeRTOS task created by startTask().
     * @param param Pointer to MotorController instance
     */
    static void taskFunc(void* param);
};

}  // namespace DC_Motor_Controller_Firmware::Control
