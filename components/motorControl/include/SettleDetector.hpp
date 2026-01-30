/**
 * @file SettleDetector.hpp
 * @brief Settle detection for motor positioning.
 *
 * Determines whether position error and velocity are within tolerance
 * for enough consecutive cycles to declare motion "settled" (done).
 */

#pragma once

#include "esp_log.h"
#include <cmath>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @struct SettleDetectorConfig
 * @brief Configuration for settle detection thresholds.
 */
struct SettleDetectorConfig {
    float posTolDeg = 0.3f;         ///< Settle position tolerance (deg).
    float velTolDegPerSec = 1.0f;   ///< Settle velocity tolerance (deg/s).
    int countLimit = 5;             ///< Consecutive cycles required to declare settled.
};

/**
 * @class SettleDetector
 * @brief Tracks position and velocity tolerance to detect motion settlement.
 *
 * Each update cycle, checks whether the absolute position error is below
 * the position tolerance and the absolute velocity is below the velocity
 * tolerance. If both conditions hold for @ref SettleDetectorConfig::countLimit
 * consecutive cycles, the detector reports settled.
 */
class SettleDetector {
  public:
    /** @brief Default constructor. */
    SettleDetector() = default;

    /** @brief Default destructor. */
    ~SettleDetector() = default;

    /** @brief Default copy constructor. */
    SettleDetector(const SettleDetector&) = default;

    /** @brief Default copy assignment operator. */
    SettleDetector& operator=(const SettleDetector&) = default;

    /** @brief Default move constructor. */
    SettleDetector(SettleDetector&&) = default;

    /** @brief Default move assignment operator. */
    SettleDetector& operator=(SettleDetector&&) = default;

    /**
     * @brief Feed one cycle of error and velocity data.
     *
     * Increments the settle counter if both position and velocity are
     * within tolerance; resets the counter otherwise.
     *
     * @param errorAbs Absolute position error (deg).
     * @param velocityDegPerSec Angular velocity (deg/s).
     * @param cfg Settle configuration.
     * @return true if settle count limit has been reached.
     */
    bool update(float errorAbs, float velocityDegPerSec, const SettleDetectorConfig& cfg);

    /**
     * @brief Reset the internal settle counter to zero.
     */
    void reset();

    /**
     * @brief Get the current settle counter value.
     * @return Number of consecutive cycles where both tolerances were met.
     */
    int getCount() const;

  private:
    /** @brief Count of consecutive cycles where position and velocity are within tolerance. */
    int settleCounter = 0;

    /** @brief Log tag for ESP-IDF logging. */
    static constexpr const char* TAG = "SettleDetector";
};

}  // namespace DC_Motor_Controller_Firmware::Control
