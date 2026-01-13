/**
 * @file MotionProfile.hpp
 * @brief Motion profile generator for trapezoid and S-curve speed shaping.
 */

#pragma once

#include "MotorControlConfig.hpp"
#include <cstdint>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @class MotionProfile
 * @brief Generates smooth speed profiles using trapezoid or S-curve shaping.
 */
class MotionProfile {
  public:
    /**
     * @brief Configure profile parameters.
     * @param type Profile type (TRAPEZOID or S_CURVE).
     * @param accelLimit Maximum acceleration in %/s.
     * @param jerkLimit Maximum jerk in %/s^2 (S-curve only).
     */
    void configure(MotionProfileType type, float accelLimit, float jerkLimit);

    /**
     * @brief Enable or disable profile shaping.
     * @param enabled True to enable, false to bypass.
     */
    void setEnabled(bool enabled);

    /**
     * @brief Check if profile is enabled.
     * @return True if enabled.
     */
    bool isEnabled() const;

    /**
     * @brief Set maximum speed limit.
     * @param maxSpeed Maximum speed in %.
     */
    void setMaxSpeed(float maxSpeed);

    /**
     * @brief Shape desired speed through the profile.
     * @param desiredSpeed Target speed in % (signed).
     * @param nowUs Current timestamp in microseconds.
     * @return Profiled speed in % (signed).
     */
    float shape(float desiredSpeed, uint64_t nowUs);

    /**
     * @brief Reset profile state for new motion.
     */
    void reset();

  private:
    bool enabled = false;                                  ///< True if profile shaping is active
    MotionProfileType type = MotionProfileType::TRAPEZOID; ///< Current profile type
    float accelLimitPctPerSec = 300.0f;                    ///< Acceleration limit (%/s)
    float jerkLimitPctPerSec2 = 6000.0f;                   ///< Jerk limit (%/s^2) for S-curve
    float maxSpeedPct = 100.0f;                            ///< Maximum speed limit (%)

    float currentSpeed = 0.0f;   ///< Current profiled speed (%)
    float currentAccel = 0.0f;   ///< Current acceleration state (%/s) for S-curve
    uint64_t lastUpdateUs = 0;   ///< Timestamp of last update (us)
};

}  // namespace DC_Motor_Controller_Firmware::Control
