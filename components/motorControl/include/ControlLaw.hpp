/**
 * @file ControlLaw.hpp
 * @brief Pure control signal computation without hardware or threading concerns.
 */

#pragma once

#include "FuzzyPIDController.hpp"
#include "MotionProfile.hpp"
#include "MotorControlConfig.hpp"
#include "PID.hpp"
#include <cmath>
#include <cstdint>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @struct ControlLawConfig
 * @brief Configuration for control law computation.
 */
struct ControlLawConfig {
    float minSpeed = 2.0f;              ///< Minimum output magnitude (%)
    float maxSpeed = 100.0f;            ///< Maximum output magnitude (%)
    float minErrorToMove = 0.2f;        ///< Deadband threshold (deg)
    float Kff_pos = 0.0f;               ///< Position feed-forward gain
    float Kff_vel = 0.0f;               ///< Velocity feed-forward gain
    bool motionProfileEnabled = false;  ///< Enable profile shaping
    MotionProfileType profileType = MotionProfileType::TRAPEZOID;  ///< Profile type
    float accelLimitPctPerSec = 300.0f; ///< Acceleration limit (%/s)
    float jerkLimitPctPerSec2 = 6000.0f;///< Jerk limit (%/s^2, S-curve only)
};

/**
 * @struct ControlOutput
 * @brief Result of control law computation.
 */
struct ControlOutput {
    float commandPct = 0.0f;   ///< Signed command percentage [-100, 100]
    bool settled = false;      ///< True if PID declares settled
    float rawPidOutput = 0.0f; ///< Pre-profile PID output for diagnostics
};

/**
 * @class ControlLaw
 * @brief Computes motor control signal from error and feedback.
 */
class ControlLaw {
  public:
    /**
     * @brief Construct with standard PID controller.
     * @param pid Reference to PID controller
     * @param cfg Control law configuration
     */
    ControlLaw(PID::PIDController& pid, const ControlLawConfig& cfg = ControlLawConfig());

    /**
     * @brief Construct with fuzzy PID controller.
     * @param pid Reference to base PID controller
     * @param fuzzy Reference to fuzzy PID controller
     * @param cfg Control law configuration
     */
    ControlLaw(PID::PIDController& pid, PID::FuzzyPIDController& fuzzy,
               const ControlLawConfig& cfg = ControlLawConfig());

    /**
     * @brief Compute control output for given state.
     * @param target Target position (deg)
     * @param position Current position (deg)
     * @param velocity Current velocity (deg/s)
     * @param nowUs Current timestamp (microseconds)
     * @return ControlOutput with command percentage and flags
     */
    ControlOutput compute(float target, float position, float velocity, uint64_t nowUs);

    /**
     * @brief Reset internal state (PID integral, profile state).
     */
    void reset();

    /**
     * @brief Update configuration at runtime.
     * @param cfg New configuration
     */
    void setConfig(const ControlLawConfig& cfg);

    /**
     * @brief Get current configuration.
     * @return Current ControlLawConfig
     */
    ControlLawConfig getConfig() const noexcept;

    /**
     * @brief Set PID gains.
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPID(float kp, float ki, float kd);

    /**
     * @brief Get current PID gains.
     * @param kp Output: proportional gain
     * @param ki Output: integral gain
     * @param kd Output: derivative gain
     */
    void getPID(float& kp, float& ki, float& kd) const;

    /**
     * @brief Set feed-forward gains.
     * @param kpos Position FF gain
     * @param kvel Velocity FF gain
     */
    void setFeedForward(float kpos, float kvel);

    /**
     * @brief Get feed-forward gains.
     * @param kpos Output: position FF gain
     * @param kvel Output: velocity FF gain
     */
    void getFeedForward(float& kpos, float& kvel) const noexcept;

    /**
     * @brief Enable/disable motion profile.
     * @param enable True to enable
     */
    void enableMotionProfile(bool enable);

    /**
     * @brief Configure motion profile parameters.
     * @param type Profile type
     * @param accel Acceleration limit (%/s)
     * @param jerk Jerk limit (%/s^2)
     */
    void configureMotionProfile(MotionProfileType type, float accel, float jerk);

    /**
     * @brief Set maximum speed for profile.
     * @param maxSpeed Maximum speed (%)
     */
    void setMaxSpeed(float maxSpeed);

    /**
     * @brief Check if using fuzzy PID.
     * @return True if fuzzy PID is active
     */
    bool isUsingFuzzy() const noexcept;

  private:
    /**
     * @brief Clamp value to [-100, 100] range.
     * @param signal Input signal
     * @return Clamped signal
     */
    float clampToPercentRange(float signal) const;

    /**
     * @brief Apply min/max speed constraints.
     * @param combined Combined control signal
     * @param error Position error
     * @return Speed-limited signal
     */
    float applySpeedLimits(float combined, float error) const;

    PID::PIDController& pid;                ///< Base PID controller
    PID::FuzzyPIDController* fuzzyPtr;      ///< Optional fuzzy controller (nullptr if not used)
    bool useFuzzy;                          ///< True if using fuzzy PID
    ControlLawConfig config;                ///< Configuration
    MotionProfile profile;                  ///< Motion profile shaper
    mutable float clampPrevValue = NAN;     ///< Previous clamped value for log deduplication
};

}  // namespace DC_Motor_Controller_Firmware::Control
