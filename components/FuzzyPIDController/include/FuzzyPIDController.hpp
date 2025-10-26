/**
 * @file FuzzyPIDController.hpp
 * @brief Fuzzy-PID controller that adaptively adjusts PID gains using fuzzy logic
 *
 * This controller uses fuzzy logic to tune Kp, Ki, and Kd parameters in real-time
 * based on error magnitude and error rate of change. This provides better performance
 * across different operating conditions compared to fixed-gain PID.
 *
 * Theory: The fuzzy system evaluates two inputs (error, error_rate) and produces
 * three outputs (Kp_scale, Ki_scale, Kd_scale) that adjust the PID gains dynamically.
 */

#pragma once

#include "PID.hpp"
#include "Fuzzy.h"
#include "FuzzyInput.h"
#include "FuzzyOutput.h"
#include "FuzzyRule.h"
#include "FuzzySet.h"
#include "FuzzyRuleAntecedent.h"
#include "FuzzyRuleConsequent.h"
#include <cmath>

namespace DC_Motor_Controller_Firmware {
namespace PID {

/**
 * @struct FuzzyPidConfig
 * @brief Configuration for Fuzzy-PID controller
 */
struct FuzzyPidConfig {
    // Base PID configuration
    PidConfig basePidConfig;

    // Fuzzy tuning ranges for PID gains
    float kpMin = 0.5f;   ///< Minimum Kp value
    float kpMax = 10.0f;  ///< Maximum Kp value
    float kiMin = 0.0f;   ///< Minimum Ki value
    float kiMax = 5.0f;   ///< Maximum Ki value
    float kdMin = 0.0f;   ///< Minimum Kd value
    float kdMax = 2.0f;   ///< Maximum Kd value

    // Fuzzy input ranges (normalized universe of discourse)
    float errorRange = 100.0f;     ///< Maximum error magnitude for fuzzy input
    float errorRateRange = 50.0f;  ///< Maximum error rate for fuzzy input

    // Enable/disable fuzzy adaptation
    bool enableFuzzyAdaptation = true;  ///< If false, uses base PID gains only
};

/**
 * @class FuzzyPIDController
 * @brief PID controller with fuzzy logic gain scheduling
 *
 * Fuzzy Rule Base Strategy:
 * - Large error + increasing error rate -> High Kp (fast response), Low Ki, Medium Kd
 * - Large error + decreasing error rate -> Medium Kp, Low Ki, High Kd (prevent overshoot)
 * - Small error + stable -> Low Kp, High Ki (eliminate steady-state error), Low Kd
 * - Small error + oscillating -> Medium Kp, Medium Ki, High Kd (damping)
 */
class FuzzyPIDController {
  public:
    /**
     * @brief Constructor
     * @param cfg Fuzzy-PID configuration
     */
    explicit FuzzyPIDController(const FuzzyPidConfig& cfg);

    /**
     * @brief Destructor - cleans up fuzzy system components
     */
    ~FuzzyPIDController();

    /**
     * @brief Compute control output with fuzzy gain adaptation
     * @param setpoint Desired target value
     * @param actual Current measured value
     * @return Control output (clamped to maxOutput)
     */
    float compute(float setpoint, float actual);

    /**
     * @brief Reset controller state (clears integral, error history)
     */
    void reset();

    /**
     * @brief Check if controller has settled
     * @return true if settled, false otherwise
     */
    bool isSettled() const;

    /**
     * @brief Get current PID gains (after fuzzy adaptation)
     * @param kp Proportional gain output
     * @param ki Integral gain output
     * @param kd Derivative gain output
     */
    void getCurrentGains(float& kp, float& ki, float& kd) const;

    /**
     * @brief Get last error value
     * @return Last error (setpoint - actual)
     */
    float getLastError() const;

    /**
     * @brief Get last controller output
     * @return Last PID output
     */
    float getOutput() const;

    /**
     * @brief Enable or disable fuzzy adaptation
     * @param enable True to enable fuzzy tuning, false for fixed gains
     */
    void setFuzzyAdaptation(bool enable);

  private:
    /**
     * @brief Initialize fuzzy logic system with membership functions and rules
     */
    void initializeFuzzySystem();

    /**
     * @brief Update PID gains using fuzzy inference
     * @param error Current error
     * @param errorRate Rate of error change
     */
    void updateGains(float error, float errorRate);

    /**
     * @brief Cleanup fuzzy system memory
     */
    void cleanupFuzzySystem();

    float clamp(float value, float lowLimit, float highLimit);
    float wrapDeg(float angle);
    float angDiffDeg(float target, float actual);

    // Configuration
    FuzzyPidConfig config;

    // PID controller instance
    PIDController pidController;

    // Fuzzy logic system
    Fuzzy* fuzzy;

    // Fuzzy inputs
    FuzzyInput* errorInput;
    FuzzyInput* errorRateInput;

    // Fuzzy outputs
    FuzzyOutput* kpOutput;
    FuzzyOutput* kiOutput;
    FuzzyOutput* kdOutput;

    // Fuzzy sets for error (5 membership functions)
    FuzzySet* errorNB;
    FuzzySet* errorNS;
    FuzzySet* errorZE;
    FuzzySet* errorPS;
    FuzzySet* errorPB;

    // Fuzzy sets for error rate (5 membership functions)
    FuzzySet* errorRateNB;
    FuzzySet* errorRateNS;
    FuzzySet* errorRateZE;
    FuzzySet* errorRatePS;
    FuzzySet* errorRatePB;

    // Fuzzy sets for Kp output
    FuzzySet* kpVS;
    FuzzySet* kpS;
    FuzzySet* kpM;
    FuzzySet* kpB;
    FuzzySet* kpVB;

    // Fuzzy sets for Ki output
    FuzzySet* kiVS;
    FuzzySet* kiS;
    FuzzySet* kiM;
    FuzzySet* kiB;
    FuzzySet* kiVB;

    // Fuzzy sets for Kd output
    FuzzySet* kdVS;
    FuzzySet* kdS;
    FuzzySet* kdM;
    FuzzySet* kdB;
    FuzzySet* kdVB;

    // State variables
    float lastError;
    float currentKp;
    float currentKi;
    float currentKd;
    uint64_t lastTimeUs;
    bool fuzzyInitialized;
};

}  // namespace PID
}  // namespace DC_Motor_Controller_Firmware
