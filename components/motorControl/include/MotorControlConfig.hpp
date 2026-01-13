/**
 * @file MotorControlConfig.hpp
 * @brief Configuration structures for motor control system.
 */

#pragma once

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
 * @struct MotorControlConfig
 * @brief Configuration structure for motion constraints and drift/stuck detection.
 */
struct MotorControlConfig {
    float minSpeed = 2.0f;        ///< Minimum allowed motor speed (%).
    float maxSpeed = 100.0f;      ///< Maximum allowed motor speed (%).
    float minErrorToMove = 0.2f;  ///< Minimum |error| to trigger motion (deg).

    float driftThreshold = 1.0f;   ///< Legacy drift threshold (deg); not used for wake-up; see driftDeadband/driftHysteresis.
    float driftDeadband = 0.5f;    ///< Allowed steady-state drift band (deg); ignore drift within this when settled.
    float driftHysteresis = 0.5f;  ///< Extra margin (deg) added to deadband for wake-up: |drift| > deadband + hysteresis.

    float stuckPositionEpsilon = 0.05f;  ///< Movement epsilon for stuck detection (deg).
    int stuckCountLimit = 50;            ///< Update cycles before declaring "stuck".
    int pidWarmupLimit = 10;             ///< Cycles allowed for PID to stabilize before drift/stuck checks.
    int countsPerRevolution = 1024;      ///< Encoder counts per full revolution (ticks/turn).

    int motionTimeoutMs = 0;  ///< Max allowed motion time per command (ms); 0 = disabled.

    bool motionProfileEnabled = false;                                   ///< Enable motion profile shaping.
    MotionProfileType motionProfileType = MotionProfileType::TRAPEZOID;  ///< Profile type selection.

    float accelLimitPctPerSec = 300.0f;   ///< Max acceleration of commanded speed (% per second).
    float jerkLimitPctPerSec2 = 6000.0f;  ///< Max jerk for S-curve (% per second^2).

    float Kff_pos = 0.0f;  ///< Position feed-forward gain.
    float Kff_vel = 0.0f;  ///< Velocity feed-forward gain.

    float settlePosTolDeg = 0.3f;        ///< Settle position tolerance (deg).
    float settleVelTolDegPerSec = 1.0f;  ///< Settle velocity tolerance (deg/s).
    int settleCountLimit = 5;            ///< Consecutive cycles required to declare settled.
};

/**
 * @struct MotorStatus
 * @brief Snapshot of controller state for monitoring and debugging.
 */
struct MotorStatus {
    float target;     ///< Current target position (deg)
    float position;   ///< Current measured position (deg)
    float error;      ///< target - position (deg)
    float velocity;   ///< Estimated angular velocity (deg/s)
    float pidOutput;  ///< Last computed PID output value
    int stuckCount;   ///< Current stuck counter value
    bool motionDone;  ///< True if motion is complete
};

}  // namespace DC_Motor_Controller_Firmware::Control
