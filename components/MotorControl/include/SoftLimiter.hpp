/**
 * @file SoftLimiter.hpp
 * @brief Soft mechanical limit enforcement for motor positioning.
 *
 * Clamps target positions and checks runtime commands against
 * configurable angle boundaries.
 */

#pragma once

#include "esp_log.h"
#include <cmath>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @class SoftLimiter
 * @brief Enforces soft mechanical angle limits on target and runtime commands.
 *
 * Provides two modes of enforcement:
 * - Target clamping: constrains the requested target position to [min, max].
 * - Runtime blocking: prevents motor commands that would drive beyond limits.
 */
class SoftLimiter {
  public:
    /** @brief Default constructor. */
    SoftLimiter() = default;

    /** @brief Default destructor. */
    ~SoftLimiter() = default;

    /** @brief Default copy constructor. */
    SoftLimiter(const SoftLimiter&) = default;

    /** @brief Default copy assignment operator. */
    SoftLimiter& operator=(const SoftLimiter&) = default;

    /** @brief Default move constructor. */
    SoftLimiter(SoftLimiter&&) = default;

    /** @brief Default move assignment operator. */
    SoftLimiter& operator=(SoftLimiter&&) = default;

    /**
     * @brief Set the soft limits and enforcement state.
     *
     * If minDeg > maxDeg, the values are automatically swapped.
     *
     * @param minDeg Minimum allowed angle (deg).
     * @param maxDeg Maximum allowed angle (deg).
     * @param enforce Enable enforcement immediately if true (default: true).
     */
    void setLimits(float minDeg, float maxDeg, bool enforce = true);

    /**
     * @brief Check if enforcement is currently active.
     * @return true if soft limits are being enforced.
     */
    bool isEnforced() const;

    /**
     * @brief Clamp a target degree value to [min, max] if enforced.
     *
     * If enforcement is disabled, returns the input unchanged.
     *
     * @param degrees Input target angle (deg).
     * @return Possibly clamped target angle (deg).
     */
    float clampTarget(float degrees) const;

    /**
     * @brief Check if a signed command would violate limits given current position.
     *
     * Returns true if:
     * - Enforcement is active AND
     * - signedCommand < 0 and currentPosDeg <= minDeg, OR
     * - signedCommand > 0 and currentPosDeg >= maxDeg.
     *
     * @param signedCommand Signed speed command (negative = towards min, positive = towards max).
     * @param currentPosDeg Current motor position in degrees.
     * @return true if the command is blocked (limit hit).
     */
    bool isBlocked(float signedCommand, float currentPosDeg) const;

    /**
     * @brief Get the configured minimum angle limit.
     * @return Minimum angle (deg).
     */
    float getMinDeg() const;

    /**
     * @brief Get the configured maximum angle limit.
     * @return Maximum angle (deg).
     */
    float getMaxDeg() const;

  private:
    /** @brief Whether soft limit enforcement is active. */
    bool enforced = false;

    /** @brief Minimum allowed mechanical angle (deg). */
    float minDeg = 0.0f;

    /** @brief Maximum allowed mechanical angle (deg). */
    float maxDeg = 0.0f;

    /** @brief Log tag for ESP-IDF logging. */
    static constexpr const char* TAG = "SoftLimiter";
};

}  // namespace DC_Motor_Controller_Firmware::Control
