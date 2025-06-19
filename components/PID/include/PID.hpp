/**
 * @file PIDController.hpp
 * @brief Generic PID controller class with anti-windup, derivative filtering,
 * and auto-settle detection.
 *
 * This class computes PID output from a target setpoint and actual feedback
 * input. It supports advanced features like integral clamping, derivative
 * filtering, and stability detection.
 */

#pragma once

#include <cstdint>

namespace DC_Motor_Controller_Firmware {
namespace PID {

/**
 * @struct PidConfig
 * @brief Configuration parameters for the PID controller.
 */
struct PidConfig {
  float kp;                 ///< Proportional gain
  float ki;                 ///< Integral gain
  float kd;                 ///< Derivative gain
  float maxOutput = 100.0f; ///< Output clamping (absolute)
  float maxIntegral =
      1000.0f; ///< Maximum allowed integral accumulation (anti-windup)
  float errorEpsilon = 2.0f; ///< Threshold to consider error "small"
  float speedEpsilon = 7.0f; ///< Threshold to consider motor speed "slow"
  float errorTimeoutSec =
      0.6f; ///< Time (sec) to consider "settled" if error stays small
  float stuckTimeoutSec = 0.5f; ///< Time (sec) to detect if system is stuck
  float derivativeAlpha =
      1.0f; ///< Derivative smoothing factor (low-pass filter)
};

/**
 * @class PIDController
 * @brief Implements a discrete PID controller with smoothing, clamping, and
 * settle detection.
 */
class PIDController {
public:
  /**
   * @brief Constructor to initialize the PID controller with config values.
   *
   * @param cfg Constant reference to PidConfig struct
   */
  explicit PIDController(const PidConfig &cfg);

  /**
   * @brief Destructor.
   */
  ~PIDController();

  /**
   * @brief Reset all internal state (integral, error, derivative).
   */
  void reset();

  /**
   * @brief Compute the PID output based on current setpoint and measurement.
   *
   * @param setpoint Desired target value
   * @param actual Actual measured value
   * @return float PID controller output (clamped)
   */
  float compute(float setpoint, float actual);

  /**
   * @brief Set new PID parameters (kp, ki, kd).
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void setParameters(float kp, float ki, float kd);

  /**
   * @brief Get current PID parameters.
   *
   * @param kp Proportional gain (output)
   * @param ki Integral gain (output)
   * @param kd Derivative gain (output)
   */
  void getParameters(float &kp, float &ki, float &kd);

  /**
   * @brief Check if the controller has settled (within tolerance for time).
   *
   * @return true if settled
   * @return false if still adjusting
   */
  bool isSettled() const;

  /**
   * @brief Get the last computed error value.
   *
   * @return float Last error
   */
  float getLastError() const;

  /**
   * @brief Get the last computed derivative value.
   *
   * @return float Last derivative
   */
  float getLastDerivative() const;

  /**
   * @brief Get the last output value from compute().
   *
   * @return float Last PID output
   */
  float getOutput() const;

private:
  PidConfig config; ///< Configuration struct

  float integral = 0.0f;       ///< Integral term accumulator
  float lastError = 0.0f;      ///< Last cycle error
  float lastOutput = 0.0f;     ///< Output from last compute
  float lastDerivative = 0.0f; ///< Last calculated derivative
  float prevMeasured = 0.0f;   ///< Previous measured value for derivative

  uint64_t lastTimeUs = 0; ///< Last compute time (in microseconds)
  bool settled = true;     ///< Is the controller settled?

  int64_t stuckStartTime = 0;      ///< Time when stuck condition started
  int64_t smallErrorStartTime = 0; ///< Time when error became small
};

} // namespace PID
} // namespace DC_Motor_Controller_Firmware
