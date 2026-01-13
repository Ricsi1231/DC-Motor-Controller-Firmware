/**
 * @file MotionDetector.hpp
 * @brief Motion state detection for settle, stall, and stuck conditions.
 */

#pragma once

#include <cmath>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @class MotionDetector
 * @brief Detects motion completion, stall, and stuck conditions.
 */
class MotionDetector {
  public:
    /**
     * @struct Config
     * @brief Configuration for motion detection thresholds.
     */
    struct Config {
        float minErrorToMove = 0.2f;         ///< Minimum error to consider motion needed (deg).
        float stuckPositionEpsilon = 0.05f;  ///< Movement threshold for stuck detection (deg).
        int stuckCountLimit = 50;            ///< Cycles before declaring stuck.
        int pidWarmupLimit = 10;             ///< Warmup cycles before detection starts.
        float settlePosTolDeg = 0.3f;        ///< Position tolerance for settle (deg).
        float settleVelTolDegPerSec = 1.0f;  ///< Velocity tolerance for settle (deg/s).
        int settleCountLimit = 5;            ///< Consecutive cycles for settle.
    };

    /**
     * @brief Set detection configuration.
     * @param cfg Configuration parameters.
     */
    void setConfig(const Config& cfg);

    /**
     * @brief Update detection state with new measurements.
     * @param error Current position error in degrees.
     * @param velocity Current velocity in deg/s.
     * @param deltaPos Position change since last update in degrees.
     */
    void update(float error, float velocity, float deltaPos);

    /**
     * @brief Check if motion has settled.
     * @return True if settled.
     */
    bool isSettled() const;

    /**
     * @brief Check if motor is stuck.
     * @return True if stuck.
     */
    bool isStuck() const;

    /**
     * @brief Get current stuck counter value.
     * @return Stuck count.
     */
    int getStuckCount() const;

    /**
     * @brief Get current settle counter value.
     * @return Settle count.
     */
    int getSettleCount() const;

    /**
     * @brief Reset detection state for new motion.
     */
    void reset();

    /**
     * @brief Start motion detection cycle.
     */
    void startMotion();

  private:
    Config config;            ///< Detection configuration parameters
    int stuckCounter = 0;     ///< Consecutive stuck cycles counter
    int settleCounter = 0;    ///< Consecutive settle cycles counter
    int warmupCounter = 0;    ///< Warmup cycles counter
    bool inMotion = false;    ///< True if motion detection is active
};

}  // namespace DC_Motor_Controller_Firmware::Control
