/**
 * @file MotionGuard.hpp
 * @brief Per-command motion timeout and idle-state drift wake-up detection.
 *
 * Tracks motion command duration for timeout enforcement and monitors
 * drift in idle state to trigger wake-up when the motor drifts too far.
 */

#pragma once

#include "esp_log.h"
#include <cmath>
#include <cstdint>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @struct MotionGuardConfig
 * @brief Configuration for motion timeout and drift wake-up.
 */
struct MotionGuardConfig {
    int motionTimeoutMs = 0;       ///< Max allowed motion time per command (ms); 0 = disabled.
    float driftDeadband = 0.5f;    ///< Allowed steady-state drift band (deg).
    float driftHysteresis = 0.5f;  ///< Extra margin (deg) added to deadband for wake-up.
};

/**
 * @class MotionGuard
 * @brief Monitors motion timeout and idle drift for wake-up decisions.
 *
 * Provides two guard functions:
 * - Timeout: enforces a maximum motion duration per command.
 * - Drift wake-up: re-activates the control loop when the motor
 *   drifts beyond the deadband + hysteresis threshold while idle.
 */
class MotionGuard {
  public:
    /** @brief Default constructor. */
    MotionGuard() = default;

    /** @brief Default destructor. */
    ~MotionGuard() = default;

    /** @brief Default copy constructor. */
    MotionGuard(const MotionGuard&) = default;

    /** @brief Default copy assignment operator. */
    MotionGuard& operator=(const MotionGuard&) = default;

    /** @brief Default move constructor. */
    MotionGuard(MotionGuard&&) = default;

    /** @brief Default move assignment operator. */
    MotionGuard& operator=(MotionGuard&&) = default;

    /**
     * @brief Start timing a new motion command.
     *
     * Records the current timestamp as the motion start time for timeout tracking.
     *
     * @param nowUs Current timestamp in microseconds.
     */
    void startMotion(uint64_t nowUs);

    /**
     * @brief Reset the guard state (motion ended or stopped).
     *
     * Clears the motion start timestamp.
     */
    void reset();

    /**
     * @brief Check if the motion timeout has expired.
     *
     * Returns false if the timeout is disabled (motionTimeoutMs <= 0)
     * or if no motion has been started (motionStartUs == 0).
     *
     * @param nowUs Current timestamp in microseconds.
     * @param cfg Guard configuration.
     * @return true if the elapsed time exceeds the configured timeout.
     */
    bool isTimedOut(uint64_t nowUs, const MotionGuardConfig& cfg) const;

    /**
     * @brief Check if a settled motor should wake due to drift.
     *
     * Computes the wake threshold as driftDeadband + driftHysteresis
     * and returns true if |drift| exceeds that threshold.
     *
     * @param drift Current drift (target - position) in degrees.
     * @param cfg Guard configuration.
     * @return true if the motor should re-activate to correct drift.
     */
    bool shouldWake(float drift, const MotionGuardConfig& cfg) const;

    /**
     * @brief Get the configured drift deadband value.
     *
     * Static utility for evaluating final drift after stall detection.
     *
     * @param cfg Guard configuration.
     * @return Drift deadband value (deg).
     */
    static float getDriftDeadband(const MotionGuardConfig& cfg);

    /**
     * @brief Get the motion start timestamp.
     * @return Timestamp in microseconds, or 0 if no motion is active.
     */
    uint64_t getMotionStartUs() const;

  private:
    /** @brief Timestamp (us) when the current motion command was initiated; 0 if inactive. */
    uint64_t motionStartUs = 0;

    /** @brief Log tag for ESP-IDF logging. */
    static constexpr const char* TAG = "MotionGuard";
};

}  // namespace DC_Motor_Controller_Firmware::Control
