/**
 * @file MotionMonitor.hpp
 * @brief Observes motion behavior and emits events for state transitions.
 */

#pragma once

#include "MotionDetector.hpp"
#include "SoftLimits.hpp"
#include <cstdint>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @enum MotionEvent
 * @brief Events emitted by MotionMonitor indicating state transitions.
 */
enum class MotionEvent {
    NONE,       ///< No event - continue normal operation
    SETTLED,    ///< Motion has settled at target
    STALLED,    ///< Motor is stuck (not moving despite command)
    TIMEOUT,    ///< Motion timeout exceeded
    LIMIT_HIT   ///< Soft limit reached
};

/**
 * @struct MotionMonitorConfig
 * @brief Configuration for motion monitoring thresholds.
 */
struct MotionMonitorConfig {
    float minErrorToMove = 0.2f;         ///< Minimum error to consider motion (deg)
    float stuckPositionEpsilon = 0.05f;  ///< Movement threshold for stuck detection (deg)
    int stuckCountLimit = 50;            ///< Cycles before declaring stuck
    int pidWarmupLimit = 10;             ///< Warmup cycles before detection starts
    float settlePosTolDeg = 0.3f;        ///< Position tolerance for settle (deg)
    float settleVelTolDegPerSec = 1.0f;  ///< Velocity tolerance for settle (deg/s)
    int settleCountLimit = 5;            ///< Consecutive cycles for settle

    // Timeout configuration
    int motionTimeoutMs = 0;             ///< Motion timeout (ms); 0 = disabled

    // Drift detection
    float driftDeadband = 0.5f;          ///< Allowed drift in idle state (deg)
    float driftHysteresis = 0.5f;        ///< Extra margin for wake-up (deg)

    // Soft limits
    float softLimitMinDeg = 0.0f;        ///< Minimum angle limit (deg)
    float softLimitMaxDeg = 0.0f;        ///< Maximum angle limit (deg)
    bool softLimitsEnforced = false;     ///< Enable soft limit enforcement
};

/**
 * @struct MotionObservation
 * @brief Input state snapshot for motion evaluation.
 */
struct MotionObservation {
    float error;           ///< Position error (deg)
    float velocity;        ///< Angular velocity (deg/s)
    float deltaPosition;   ///< Position change since last update (deg)
    float position;        ///< Current position (deg)
    float commandPct;      ///< Current commanded output (%)
    uint64_t nowUs;        ///< Current timestamp (us)
    bool pidSettled;       ///< PID controller reports settled
};

/**
 * @class MotionMonitor
 * @brief Observes motion and decides when to signal state transitions.
 */
class MotionMonitor {
  public:
    /**
     * @brief Default constructor.
     */
    MotionMonitor() = default;

    /**
     * @brief Construct with configuration.
     * @param cfg Monitor configuration
     */
    explicit MotionMonitor(const MotionMonitorConfig& cfg);

    /**
     * @brief Evaluate motion state and return any event.
     * @param obs Current motion observation
     * @return MotionEvent indicating state transition (or NONE)
     */
    MotionEvent evaluate(const MotionObservation& obs);

    /**
     * @brief Start monitoring a new motion command.
     * @param nowUs Start timestamp (us)
     */
    void startMotion(uint64_t nowUs);

    /**
     * @brief Reset all monitoring state.
     */
    void reset();

    /**
     * @brief Update configuration.
     * @param cfg New configuration
     */
    void setConfig(const MotionMonitorConfig& cfg);

    /**
     * @brief Get current configuration.
     * @return Current configuration
     */
    MotionMonitorConfig getConfig() const noexcept;

    /**
     * @brief Set soft limits.
     * @param minDeg Minimum angle (deg)
     * @param maxDeg Maximum angle (deg)
     * @param enforce Enable enforcement
     */
    void setSoftLimits(float minDeg, float maxDeg, bool enforce);

    /**
     * @brief Clamp target to soft limits.
     * @param target Desired target
     * @return Clamped target
     */
    float clampTarget(float target) const;

    /**
     * @brief Check if soft limits are enforced.
     * @return True if enforced
     */
    bool areSoftLimitsEnforced() const noexcept;

    /**
     * @brief Get soft limit minimum.
     * @return Minimum angle (deg)
     */
    float getSoftLimitMin() const noexcept;

    /**
     * @brief Get soft limit maximum.
     * @return Maximum angle (deg)
     */
    float getSoftLimitMax() const noexcept;

    /**
     * @brief Get stuck counter value (for diagnostics).
     * @return Current stuck count
     */
    int getStuckCount() const noexcept;

    /**
     * @brief Get settle counter value (for diagnostics).
     * @return Current settle count
     */
    int getSettleCount() const noexcept;

    /**
     * @brief Set motion timeout.
     * @param ms Timeout in milliseconds (0 = disabled)
     */
    void setMotionTimeoutMs(uint32_t ms);

    /**
     * @brief Get motion timeout.
     * @return Timeout in milliseconds
     */
    uint32_t getMotionTimeoutMs() const noexcept;

    /**
     * @brief Set warmup cycles before detection activates.
     * @param cycles Number of warmup cycles
     */
    void setPidWarmupCycles(int cycles);

    /**
     * @brief Get warmup cycle count.
     * @return Warmup cycles
     */
    int getPidWarmupCycles() const noexcept;

    /**
     * @brief Check if drift exceeds wake threshold.
     * @param target Current target position (deg)
     * @param position Current position (deg)
     * @return True if should wake from idle
     */
    bool shouldWakeFromIdle(float target, float position) const;

    /**
     * @brief Check if final error is within acceptable deadband.
     * @param error Position error (deg)
     * @return True if within deadband
     */
    bool isWithinDriftDeadband(float error) const;

  private:
    /**
     * @brief Check for timeout condition.
     * @param nowUs Current time
     * @return True if timeout exceeded
     */
    bool checkTimeout(uint64_t nowUs) const;

    /**
     * @brief Check for limit hit condition.
     * @param position Current position
     * @param commandPct Commanded output
     * @return True if at limit
     */
    bool checkLimitHit(float position, float commandPct) const;

    MotionMonitorConfig config;      ///< Configuration
    MotionDetector detector;         ///< Settle/stall detection
    SoftLimits limits;               ///< Soft limit management
    uint64_t motionStartUs = 0;      ///< Motion start timestamp
    bool motionActive = false;       ///< True if monitoring active
};

}  // namespace DC_Motor_Controller_Firmware::Control
