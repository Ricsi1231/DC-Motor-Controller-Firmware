/**
 * @file IMotorController.hpp
 * @brief Hardware-agnostic interface for closed-loop motor controllers.
 *
 * Defines a generic motor controller API that combines encoder feedback,
 * a motor driver, and a PID controller to move a DC motor to a target
 * position. Concrete implementations inherit from this interface and
 * provide algorithm-specific functionality.
 */

#pragma once

#include <cstdint>

namespace DC_Motor_Controller_Firmware {

/**
 * @struct MotorStatus
 * @brief Snapshot of the motor controller state at a point in time.
 */
struct MotorStatus {
    float target;     ///< Target position in degrees
    float position;   ///< Current position in degrees
    float error;      ///< Position error (target - position) in degrees
    float velocity;   ///< Estimated angular velocity in deg/s
    float pidOutput;  ///< Last PID controller output value
    int stuckCount;   ///< Stuck/stall detection counter
    bool motionDone;  ///< True if motion is complete and settled
};

/**
 * @class IMotorController
 * @brief Abstract interface for a closed-loop motor position controller.
 */
class IMotorController {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IMotorController() = default;

    /**
     * @brief Start the background control task.
     */
    virtual void startTask() = 0;

    /**
     * @brief Stop the background control task.
     */
    virtual void stopTask() = 0;

    /**
     * @brief Immediately stop the motor and mark motion as done.
     */
    virtual void stop() = 0;

    /**
     * @brief Set the desired target position in degrees.
     * @param degrees New target angle (deg).
     */
    virtual void setTargetDegrees(float degrees) = 0;

    /**
     * @brief Set the desired target position in encoder ticks.
     * @param ticks Target position in raw encoder ticks.
     */
    virtual void setTargetTicks(int32_t ticks) = 0;

    /**
     * @brief Backward-compatible alias for degree-based target setter.
     * @param degrees New target angle (deg).
     */
    virtual void setTarget(float degrees) = 0;

    /**
     * @brief Set PID gain parameters.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    virtual void setPID(float kp, float ki, float kd) = 0;

    /**
     * @brief Get the current PID parameters.
     * @param kp Output: proportional gain.
     * @param ki Output: integral gain.
     * @param kd Output: derivative gain.
     */
    virtual void getPID(float& kp, float& ki, float& kd) = 0;

    /**
     * @brief Run the control logic once (full pipeline: encoder, PID, motor command).
     */
    virtual void update() = 0;

    /**
     * @brief Check if the motor has reached its target and settled.
     * @return true if motion is complete, false otherwise.
     */
    virtual bool isMotionDone() const = 0;

    /**
     * @brief Configure the update loop frequency for the control task.
     * @param hz Desired update rate in Hertz (must be > 0).
     */
    virtual void setUpdateHz(uint32_t hz) = 0;

    /**
     * @brief Get a snapshot of the current motor controller state.
     * @return MotorStatus containing target, position, error, velocity, PID output, stuck counter, and motion flag.
     */
    virtual MotorStatus getStatus() const = 0;

    /**
     * @brief Set angular soft limits with optional enforcement.
     * @param minDeg Minimum allowed mechanical angle (deg).
     * @param maxDeg Maximum allowed mechanical angle (deg).
     * @param enforce Enable enforcement immediately if true (default: true).
     */
    virtual void setSoftLimits(float minDeg, float maxDeg, bool enforce = true) = 0;

    /**
     * @brief Set per-command motion timeout in milliseconds.
     * @param ms Timeout in milliseconds; 0 disables the guard.
     */
    virtual void setMotionTimeoutMs(uint32_t ms) = 0;

    /**
     * @brief Get the configured per-command motion timeout in milliseconds.
     * @return Timeout in milliseconds; 0 if disabled.
     */
    virtual uint32_t getMotionTimeoutMs() const = 0;

    /**
     * @brief Set feed-forward gains used as: u_ff = Kff_pos*ref + Kff_vel*refDot.
     * @param kpos Position feed-forward gain (percent per degree).
     * @param kvel Velocity feed-forward gain (percent per percent-of-speed, signed).
     */
    virtual void setFeedForward(float kpos, float kvel) = 0;

    /**
     * @brief Get the current feed-forward gains.
     * @param kpos Out: position feed-forward gain.
     * @param kvel Out: velocity feed-forward gain.
     */
    virtual void getFeedForward(float& kpos, float& kvel) const = 0;

    /**
     * @brief Callback type for motion events (done, stall, limit hit).
     * @param status Current motor status at the time of the event.
     * @param user Opaque user-defined pointer passed during registration.
     */
    using MotionEventCallback = void (*)(const MotorStatus& status, void* user);

    /**
     * @brief Register callback fired once when motion is declared done.
     * @param cb Function pointer or nullptr to clear.
     * @param user Opaque pointer passed back on callback.
     */
    virtual void setOnMotionDone(MotionEventCallback cb, void* user = nullptr) = 0;

    /**
     * @brief Register callback fired when stall is detected.
     * @param cb Function pointer or nullptr to clear.
     * @param user Opaque pointer passed back on callback.
     */
    virtual void setOnStall(MotionEventCallback cb, void* user = nullptr) = 0;

    /**
     * @brief Register callback fired when a soft limit is hit.
     * @param cb Function pointer or nullptr to clear.
     * @param user Opaque pointer passed back on callback.
     */
    virtual void setOnLimitHit(MotionEventCallback cb, void* user = nullptr) = 0;
};

}  // namespace DC_Motor_Controller_Firmware
