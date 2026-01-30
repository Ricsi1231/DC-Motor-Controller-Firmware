/**
 * @file MotionProfiler.hpp
 * @brief Motion profile shaping and slew-rate limiting for motor control.
 *
 * Provides trapezoid and S-curve speed shaping, plus final slew-rate
 * limiting of the motor output command.
 */

#pragma once

#include "esp_log.h"
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
 * @struct MotionProfilerConfig
 * @brief Configuration for motion profile shaping.
 */
struct MotionProfilerConfig {
    bool enabled = false;                                           ///< Enable motion profile shaping.
    MotionProfileType type = MotionProfileType::TRAPEZOID;          ///< Profile type selection.
    float accelLimitPctPerSec = 300.0f;                             ///< Max acceleration of commanded speed (% per second).
    float jerkLimitPctPerSec2 = 6000.0f;                            ///< Max jerk for S-curve (% per second^2); ignored for trapezoid.
    float maxSpeed = 100.0f;                                        ///< Maximum speed clamp (%).
};

/**
 * @class MotionProfiler
 * @brief Applies trapezoid/S-curve shaping and slew-rate limiting to speed commands.
 */
class MotionProfiler {
  public:
    /** @brief Default constructor. */
    MotionProfiler() = default;

    /** @brief Default destructor. */
    ~MotionProfiler() = default;

    /** @brief Default copy constructor. */
    MotionProfiler(const MotionProfiler&) = default;

    /** @brief Default copy assignment operator. */
    MotionProfiler& operator=(const MotionProfiler&) = default;

    /** @brief Default move constructor. */
    MotionProfiler(MotionProfiler&&) = default;

    /** @brief Default move assignment operator. */
    MotionProfiler& operator=(MotionProfiler&&) = default;

    /**
     * @brief Apply profile shaping to a desired speed command.
     *
     * Uses trapezoid or S-curve algorithm to ramp the speed towards the
     * desired value, respecting acceleration and jerk limits.
     *
     * @param desiredSigned Target speed command in percent (signed).
     * @param nowUs Current timestamp in microseconds.
     * @param cfg Profile configuration.
     * @return Profiled speed command in percent (signed).
     */
    float shapeSpeed(float desiredSigned, uint64_t nowUs, const MotionProfilerConfig& cfg);

    /**
     * @brief Apply slew-rate limiting to a profiled speed command.
     *
     * Limits the rate of change of the final motor output to prevent
     * instantaneous speed jumps.
     *
     * @param profiled Profiled speed command (%).
     * @param nowUs Current timestamp in microseconds.
     * @param accelLimitPctPerSec Acceleration limit for slew rate (%/s).
     * @return Slew-limited output (%).
     */
    float applySlew(float profiled, uint64_t nowUs, float accelLimitPctPerSec);

    /**
     * @brief Reset all internal state.
     *
     * Clears profiled speed, acceleration, and slew-rate state.
     * Should be called when starting a new motion command or stopping.
     *
     * @param nowUs Optional initial timestamp (us); defaults to 0.
     */
    void reset(uint64_t nowUs = 0);

    /**
     * @brief Get the current profiled speed.
     *
     * Used as the reference velocity for feed-forward compensation.
     *
     * @return Current profiled speed percent (signed).
     */
    float getProfiledSpeed() const;

  private:
    /** @brief Current profiled speed command (%), signed. */
    float profSpeedPercent = 0.0f;

    /** @brief Current acceleration state for S-curve profile (%/s). */
    float profAccelPctPerSec = 0.0f;

    /** @brief Timestamp of last profile update (us). */
    uint64_t lastProfileUs = 0;

    /** @brief Previous slew-rate-limited output (%). */
    float slewLastOutPct = 0.0f;

    /** @brief Timestamp of last slew-rate update (us). */
    uint64_t slewLastUs = 0;

    /** @brief Log tag for ESP-IDF logging. */
    static constexpr const char* TAG = "MotionProfiler";
};

}  // namespace DC_Motor_Controller_Firmware::Control
