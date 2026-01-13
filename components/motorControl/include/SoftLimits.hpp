/**
 * @file SoftLimits.hpp
 * @brief Soft limit boundary enforcement for motor positioning.
 */

#pragma once

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @class SoftLimits
 * @brief Manages soft position limits and boundary checking.
 */
class SoftLimits {
  public:
    /**
     * @brief Set soft limit boundaries.
     * @param minDeg Minimum angle in degrees.
     * @param maxDeg Maximum angle in degrees.
     * @param enforce True to enable enforcement.
     */
    void set(float minDeg, float maxDeg, bool enforce);

    /**
     * @brief Clamp a target position to within limits.
     * @param target Desired target position in degrees.
     * @return Clamped target position.
     */
    float clampTarget(float target) const;

    /**
     * @brief Check if position is at a limit given command direction.
     * @param position Current position in degrees.
     * @param command Signed command (negative = left, positive = right).
     * @return True if at limit and command would exceed it.
     */
    bool isAtLimit(float position, float command) const;

    /**
     * @brief Check if enforcement is enabled.
     * @return True if limits are enforced.
     */
    bool isEnforced() const;

    /**
     * @brief Get minimum limit.
     * @return Minimum angle in degrees.
     */
    float getMin() const;

    /**
     * @brief Get maximum limit.
     * @return Maximum angle in degrees.
     */
    float getMax() const;

  private:
    bool enforced = false;   ///< True if limits are being enforced
    float minDeg = 0.0f;     ///< Minimum angle limit (deg)
    float maxDeg = 0.0f;     ///< Maximum angle limit (deg)
};

}  // namespace DC_Motor_Controller_Firmware::Control
