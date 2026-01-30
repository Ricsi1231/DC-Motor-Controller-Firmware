/**
 * @file StallDetector.hpp
 * @brief Stall and stuck detection for motor control.
 *
 * Tracks whether the motor is stuck (position not changing despite error),
 * manages PID warmup counter, and evaluates PID-settled stall condition.
 */

#pragma once

#include "esp_log.h"
#include <cmath>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @struct StallDetectorConfig
 * @brief Configuration for stall/stuck detection.
 */
struct StallDetectorConfig {
    float stuckPositionEpsilon = 0.05f;  ///< Movement epsilon for stuck detection (deg).
    int stuckCountLimit = 50;            ///< Update cycles before declaring "stuck".
    int pidWarmupLimit = 10;             ///< Cycles allowed for PID to stabilize before checks.
    float minErrorToMove = 0.2f;         ///< Minimum |error| to consider motor should be moving (deg).
};

/**
 * @class StallDetector
 * @brief Detects motor stall conditions including stuck position and PID saturation.
 *
 * Manages a warmup phase after each new motion command, during which
 * stall/settle checks are suppressed. After warmup, tracks whether
 * the motor position is changing; if not, increments a stuck counter.
 * Also evaluates whether the PID controller has settled while error
 * persists, indicating the motor cannot reach its target.
 */
class StallDetector {
  public:
    /**
     * @enum Result
     * @brief Possible outcomes of one update cycle.
     */
    enum class Result {
        WARMING_UP,  ///< Still in warmup period, no decisions made.
        OK,          ///< Normal operation, no stall detected.
        STALLED      ///< Stall detected (stuck counter exceeded or PID settled with error).
    };

    /** @brief Default constructor. */
    StallDetector() = default;

    /** @brief Default destructor. */
    ~StallDetector() = default;

    /** @brief Default copy constructor. */
    StallDetector(const StallDetector&) = default;

    /** @brief Default copy assignment operator. */
    StallDetector& operator=(const StallDetector&) = default;

    /** @brief Default move constructor. */
    StallDetector(StallDetector&&) = default;

    /** @brief Default move assignment operator. */
    StallDetector& operator=(StallDetector&&) = default;

    /**
     * @brief Feed one cycle of position delta, error, and PID settled state.
     *
     * During warmup (first @ref StallDetectorConfig::pidWarmupLimit cycles),
     * returns WARMING_UP and suppresses stuck detection. After warmup,
     * checks if position is not changing while error persists, and evaluates
     * PID settled condition.
     *
     * @param positionDelta Position change since last cycle (deg).
     * @param errorAbs Absolute position error (deg).
     * @param pidIsSettled Whether the PID controller reports settled.
     * @param cfg Stall detection configuration.
     * @return Result of the stall evaluation.
     */
    Result update(float positionDelta, float errorAbs, bool pidIsSettled, const StallDetectorConfig& cfg);

    /**
     * @brief Reset warmup and stuck counters to zero.
     *
     * Should be called when starting a new motion command.
     */
    void reset();

    /**
     * @brief Get the current stuck counter value.
     * @return Number of consecutive cycles where position did not change despite error.
     */
    int getStuckCount() const;

    /**
     * @brief Check if still in warmup phase.
     * @return true if the warmup counter has not yet exceeded the configured limit.
     */
    bool isWarming() const;

  private:
    /** @brief Count of consecutive cycles where position did not change despite error. */
    int stuckCounter = 0;

    /** @brief Number of update cycles elapsed since last motion command reset. */
    int pidWarmupCounter = 0;

    /** @brief Cached warmup limit for isWarming() queries between update() calls. */
    int lastWarmupLimit = 0;

    /** @brief Log tag for ESP-IDF logging. */
    static constexpr const char* TAG = "StallDetector";
};

}  // namespace DC_Motor_Controller_Firmware::Control
