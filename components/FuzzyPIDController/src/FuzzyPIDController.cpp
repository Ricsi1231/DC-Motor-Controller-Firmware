#include "FuzzyPIDController.hpp"
#include "esp_timer.h"
#include "esp_log.h"

namespace DC_Motor_Controller_Firmware {
namespace PID {

FuzzyPIDController::FuzzyPIDController(const FuzzyPidConfig& cfg)
    : config(cfg),
      pidController(cfg.basePidConfig),
      fuzzy(nullptr),
      errorInput(nullptr),
      errorRateInput(nullptr),
      kpOutput(nullptr),
      kiOutput(nullptr),
      kdOutput(nullptr),
      errorNB(nullptr),
      errorNS(nullptr),
      errorZE(nullptr),
      errorPS(nullptr),
      errorPB(nullptr),
      errorRateNB(nullptr),
      errorRateNS(nullptr),
      errorRateZE(nullptr),
      errorRatePS(nullptr),
      errorRatePB(nullptr),
      kpVS(nullptr),
      kpS(nullptr),
      kpM(nullptr),
      kpB(nullptr),
      kpVB(nullptr),
      kiVS(nullptr),
      kiS(nullptr),
      kiM(nullptr),
      kiB(nullptr),
      kiVB(nullptr),
      kdVS(nullptr),
      kdS(nullptr),
      kdM(nullptr),
      kdB(nullptr),
      kdVB(nullptr),
      lastError(0.0f),
      currentKp(cfg.basePidConfig.kp),
      currentKi(cfg.basePidConfig.ki),
      currentKd(cfg.basePidConfig.kd),
      lastTimeUs(0),
      fuzzyInitialized(false) {
    if (config.enableFuzzyAdaptation) {
        initializeFuzzySystem();
    }
}

FuzzyPIDController::~FuzzyPIDController() { cleanupFuzzySystem(); }

void FuzzyPIDController::cleanupFuzzySystem() {
    if (!fuzzyInitialized) {
        return;
    }

    delete errorNB;
    delete errorNS;
    delete errorZE;
    delete errorPS;
    delete errorPB;

    delete errorRateNB;
    delete errorRateNS;
    delete errorRateZE;
    delete errorRatePS;
    delete errorRatePB;

    delete kpVS;
    delete kpS;
    delete kpM;
    delete kpB;
    delete kpVB;
    delete kiVS;
    delete kiS;
    delete kiM;
    delete kiB;
    delete kiVB;
    delete kdVS;
    delete kdS;
    delete kdM;
    delete kdB;
    delete kdVB;

    delete errorInput;
    delete errorRateInput;
    delete kpOutput;
    delete kiOutput;
    delete kdOutput;

    delete fuzzy;

    fuzzyInitialized = false;
}

float FuzzyPIDController::clamp(float value, float lowLimit, float highLimit) {
    if (value < lowLimit) return lowLimit;
    if (value > highLimit) return highLimit;
    return value;
}

float FuzzyPIDController::wrapDeg(float angle) {
    float wrapped = fmodf(angle, 360.0f);
    if (wrapped < 0.0f) {
        wrapped += 360.0f;
    }
    return wrapped;
}

float FuzzyPIDController::angDiffDeg(float target, float actual) {
    float wrappedTarget = wrapDeg(target);
    float wrappedActual = wrapDeg(actual);

    float diff = wrappedTarget - wrappedActual;

    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }

    return diff;
}

void FuzzyPIDController::initializeFuzzySystem() {
    fuzzy = new Fuzzy();

    // ========== CREATE FUZZY INPUTS ==========
    // Input 1: Error (e = setpoint - actual)
    // Range: -errorRange to +errorRange
    errorInput = new FuzzyInput(1);
    float eRange = config.errorRange;

    // Trapezoidal and triangular membership functions
    errorNB = new FuzzySet(-eRange, -eRange, -0.85f * eRange, -0.50f * eRange);
    errorNS = new FuzzySet(-eRange * 0.5f, -eRange * 0.25f, -eRange * 0.25f, 0.0f);
    errorZE = new FuzzySet(-eRange * 0.25f, 0.0f, 0.0f, eRange * 0.25f);
    errorPS = new FuzzySet(0.0f, eRange * 0.25f, eRange * 0.25f, eRange * 0.5f);
    errorPB = new FuzzySet(0.50f * eRange, 0.85f * eRange, eRange, eRange);

    errorInput->addFuzzySet(errorNB);
    errorInput->addFuzzySet(errorNS);
    errorInput->addFuzzySet(errorZE);
    errorInput->addFuzzySet(errorPS);
    errorInput->addFuzzySet(errorPB);

    fuzzy->addFuzzyInput(errorInput);

    // Input 2: Error Rate (de/dt)
    // Range: -errorRateRange to +errorRateRange
    errorRateInput = new FuzzyInput(2);
    float erRange = config.errorRateRange;

    errorRateNB = new FuzzySet(-erRange, -erRange, -0.85f * erRange, -0.50f * erRange);
    errorRateNS = new FuzzySet(-erRange * 0.5f, -erRange * 0.25f, -erRange * 0.25f, 0.0f);
    errorRateZE = new FuzzySet(-erRange * 0.25f, 0.0f, 0.0f, erRange * 0.25f);
    errorRatePS = new FuzzySet(0.0f, erRange * 0.25f, erRange * 0.25f, erRange * 0.5f);
    errorRatePB = new FuzzySet(0.50f * erRange, 0.85f * erRange, erRange, erRange);

    errorRateInput->addFuzzySet(errorRateNB);
    errorRateInput->addFuzzySet(errorRateNS);
    errorRateInput->addFuzzySet(errorRateZE);
    errorRateInput->addFuzzySet(errorRatePS);
    errorRateInput->addFuzzySet(errorRatePB);

    fuzzy->addFuzzyInput(errorRateInput);

    // ========== CREATE FUZZY OUTPUTS ==========
    // Output 1: Kp scaling (0-100, will be mapped to kpMin-kpMax)
    kpOutput = new FuzzyOutput(1);
    kpVS = new FuzzySet(0.0f, 0.0f, 10.0f, 25.0f);
    kpS = new FuzzySet(10.0f, 25.0f, 25.0f, 50.0f);
    kpM = new FuzzySet(25.0f, 50.0f, 50.0f, 75.0f);
    kpB = new FuzzySet(50.0f, 75.0f, 75.0f, 90.0f);
    kpVB = new FuzzySet(75.0f, 90.0f, 100.0f, 100.0f);

    kpOutput->addFuzzySet(kpVS);
    kpOutput->addFuzzySet(kpS);
    kpOutput->addFuzzySet(kpM);
    kpOutput->addFuzzySet(kpB);
    kpOutput->addFuzzySet(kpVB);

    fuzzy->addFuzzyOutput(kpOutput);

    // Output 2: Ki scaling (0-100, will be mapped to kiMin-kiMax)
    kiOutput = new FuzzyOutput(2);
    kiVS = new FuzzySet(0.0f, 0.0f, 10.0f, 25.0f);
    kiS = new FuzzySet(10.0f, 25.0f, 25.0f, 50.0f);
    kiM = new FuzzySet(25.0f, 50.0f, 50.0f, 75.0f);
    kiB = new FuzzySet(50.0f, 75.0f, 75.0f, 90.0f);
    kiVB = new FuzzySet(75.0f, 90.0f, 100.0f, 100.0f);

    kiOutput->addFuzzySet(kiVS);
    kiOutput->addFuzzySet(kiS);
    kiOutput->addFuzzySet(kiM);
    kiOutput->addFuzzySet(kiB);
    kiOutput->addFuzzySet(kiVB);

    fuzzy->addFuzzyOutput(kiOutput);

    // Output 3: Kd scaling (0-100, will be mapped to kdMin-kdMax)
    kdOutput = new FuzzyOutput(3);
    kdVS = new FuzzySet(0.0f, 0.0f, 10.0f, 25.0f);
    kdS = new FuzzySet(10.0f, 25.0f, 25.0f, 50.0f);
    kdM = new FuzzySet(25.0f, 50.0f, 50.0f, 75.0f);
    kdB = new FuzzySet(50.0f, 75.0f, 75.0f, 90.0f);
    kdVB = new FuzzySet(75.0f, 90.0f, 100.0f, 100.0f);

    kdOutput->addFuzzySet(kdVS);
    kdOutput->addFuzzySet(kdS);
    kdOutput->addFuzzySet(kdM);
    kdOutput->addFuzzySet(kdB);
    kdOutput->addFuzzySet(kdVB);

    fuzzy->addFuzzyOutput(kdOutput);

    // ========== CREATE FUZZY RULES ==========
    // Rule 1: Large Positive Error Growing Fast - need aggressive correction
    FuzzyRuleAntecedent* largePositiveErrorGrowingAnt = new FuzzyRuleAntecedent();
    largePositiveErrorGrowingAnt->joinWithAND(errorPB, errorRatePB);
    FuzzyRuleConsequent* aggressiveCorrectionCon = new FuzzyRuleConsequent();
    aggressiveCorrectionCon->addOutput(kpVB);
    aggressiveCorrectionCon->addOutput(kiVS);
    aggressiveCorrectionCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(1, largePositiveErrorGrowingAnt, aggressiveCorrectionCon));

    // Rule 2: Large Positive Error Stable - strong correction with damping
    FuzzyRuleAntecedent* largePositiveErrorStableAnt = new FuzzyRuleAntecedent();
    largePositiveErrorStableAnt->joinWithAND(errorPB, errorRateZE);
    FuzzyRuleConsequent* strongCorrectionWithDampingCon = new FuzzyRuleConsequent();
    strongCorrectionWithDampingCon->addOutput(kpVB);
    strongCorrectionWithDampingCon->addOutput(kiS);
    strongCorrectionWithDampingCon->addOutput(kdB);
    fuzzy->addFuzzyRule(new FuzzyRule(2, largePositiveErrorStableAnt, strongCorrectionWithDampingCon));

    // Rule 3: Large Positive Error Decreasing - prevent overshoot
    FuzzyRuleAntecedent* largePositiveErrorDecreasingAnt = new FuzzyRuleAntecedent();
    largePositiveErrorDecreasingAnt->joinWithAND(errorPB, errorRateNB);
    FuzzyRuleConsequent* overshootPreventionCon = new FuzzyRuleConsequent();
    overshootPreventionCon->addOutput(kpB);
    overshootPreventionCon->addOutput(kiM);
    overshootPreventionCon->addOutput(kdVB);
    fuzzy->addFuzzyRule(new FuzzyRule(3, largePositiveErrorDecreasingAnt, overshootPreventionCon));

    // Rule 4: Small Positive Error Growing - moderate correction
    FuzzyRuleAntecedent* smallPositiveErrorGrowingAnt = new FuzzyRuleAntecedent();
    smallPositiveErrorGrowingAnt->joinWithAND(errorPS, errorRatePS);
    FuzzyRuleConsequent* moderateCorrectionCon = new FuzzyRuleConsequent();
    moderateCorrectionCon->addOutput(kpB);
    moderateCorrectionCon->addOutput(kiS);
    moderateCorrectionCon->addOutput(kdS);
    fuzzy->addFuzzyRule(new FuzzyRule(4, smallPositiveErrorGrowingAnt, moderateCorrectionCon));

    // Rule 5: Small Positive Error Stable - balanced response
    FuzzyRuleAntecedent* smallPositiveErrorStableAnt = new FuzzyRuleAntecedent();
    smallPositiveErrorStableAnt->joinWithAND(errorPS, errorRateZE);
    FuzzyRuleConsequent* balancedResponseCon = new FuzzyRuleConsequent();
    balancedResponseCon->addOutput(kpM);
    balancedResponseCon->addOutput(kiM);
    balancedResponseCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(5, smallPositiveErrorStableAnt, balancedResponseCon));

    // Rule 6: Small Positive Error Decreasing - emphasize integral
    FuzzyRuleAntecedent* smallPositiveErrorDecreasingAnt = new FuzzyRuleAntecedent();
    smallPositiveErrorDecreasingAnt->joinWithAND(errorPS, errorRateNS);
    FuzzyRuleConsequent* emphasizeIntegralCon = new FuzzyRuleConsequent();
    emphasizeIntegralCon->addOutput(kpS);
    emphasizeIntegralCon->addOutput(kiB);
    emphasizeIntegralCon->addOutput(kdS);
    fuzzy->addFuzzyRule(new FuzzyRule(6, smallPositiveErrorDecreasingAnt, emphasizeIntegralCon));

    // Rule 7: At Setpoint Stable - eliminate steady state error
    FuzzyRuleAntecedent* atSetpointStableAnt = new FuzzyRuleAntecedent();
    atSetpointStableAnt->joinWithAND(errorZE, errorRateZE);
    FuzzyRuleConsequent* steadyStateEliminationCon = new FuzzyRuleConsequent();
    steadyStateEliminationCon->addOutput(kpS);
    steadyStateEliminationCon->addOutput(kiVB);
    steadyStateEliminationCon->addOutput(kdVS);
    fuzzy->addFuzzyRule(new FuzzyRule(7, atSetpointStableAnt, steadyStateEliminationCon));

    // Rule 8: At Setpoint Drifting Positive - gentle correction
    FuzzyRuleAntecedent* atSetpointDriftingPositiveAnt = new FuzzyRuleAntecedent();
    atSetpointDriftingPositiveAnt->joinWithAND(errorZE, errorRatePS);
    FuzzyRuleConsequent* gentleCorrectionPositiveCon = new FuzzyRuleConsequent();
    gentleCorrectionPositiveCon->addOutput(kpM);
    gentleCorrectionPositiveCon->addOutput(kiM);
    gentleCorrectionPositiveCon->addOutput(kdS);
    fuzzy->addFuzzyRule(new FuzzyRule(8, atSetpointDriftingPositiveAnt, gentleCorrectionPositiveCon));

    // Rule 9: At Setpoint Drifting Negative - gentle correction
    FuzzyRuleAntecedent* atSetpointDriftingNegativeAnt = new FuzzyRuleAntecedent();
    atSetpointDriftingNegativeAnt->joinWithAND(errorZE, errorRateNS);
    FuzzyRuleConsequent* gentleCorrectionNegativeCon = new FuzzyRuleConsequent();
    gentleCorrectionNegativeCon->addOutput(kpM);
    gentleCorrectionNegativeCon->addOutput(kiM);
    gentleCorrectionNegativeCon->addOutput(kdS);
    fuzzy->addFuzzyRule(new FuzzyRule(9, atSetpointDriftingNegativeAnt, gentleCorrectionNegativeCon));

    // Rule 10: Large Negative Error Growing - aggressive negative correction
    FuzzyRuleAntecedent* largeNegativeErrorGrowingAnt = new FuzzyRuleAntecedent();
    largeNegativeErrorGrowingAnt->joinWithAND(errorNB, errorRateNB);
    FuzzyRuleConsequent* aggressiveNegativeCorrectionCon = new FuzzyRuleConsequent();
    aggressiveNegativeCorrectionCon->addOutput(kpVB);
    aggressiveNegativeCorrectionCon->addOutput(kiVS);
    aggressiveNegativeCorrectionCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(10, largeNegativeErrorGrowingAnt, aggressiveNegativeCorrectionCon));

    // Rule 11: Large Negative Error Stable - strong negative correction with damping
    FuzzyRuleAntecedent* largeNegativeErrorStableAnt = new FuzzyRuleAntecedent();
    largeNegativeErrorStableAnt->joinWithAND(errorNB, errorRateZE);
    FuzzyRuleConsequent* strongNegativeCorrectionCon = new FuzzyRuleConsequent();
    strongNegativeCorrectionCon->addOutput(kpVB);
    strongNegativeCorrectionCon->addOutput(kiS);
    strongNegativeCorrectionCon->addOutput(kdB);
    fuzzy->addFuzzyRule(new FuzzyRule(11, largeNegativeErrorStableAnt, strongNegativeCorrectionCon));

    // Rule 12: Large Negative Error Decreasing - prevent negative overshoot
    FuzzyRuleAntecedent* largeNegativeErrorDecreasingAnt = new FuzzyRuleAntecedent();
    largeNegativeErrorDecreasingAnt->joinWithAND(errorNB, errorRatePB);
    FuzzyRuleConsequent* negativeOvershootPreventionCon = new FuzzyRuleConsequent();
    negativeOvershootPreventionCon->addOutput(kpB);
    negativeOvershootPreventionCon->addOutput(kiM);
    negativeOvershootPreventionCon->addOutput(kdVB);
    fuzzy->addFuzzyRule(new FuzzyRule(12, largeNegativeErrorDecreasingAnt, negativeOvershootPreventionCon));

    // Rule 13: Small Negative Error Growing - moderate negative correction
    FuzzyRuleAntecedent* smallNegativeErrorGrowingAnt = new FuzzyRuleAntecedent();
    smallNegativeErrorGrowingAnt->joinWithAND(errorNS, errorRateNS);
    FuzzyRuleConsequent* moderateNegativeCorrectionCon = new FuzzyRuleConsequent();
    moderateNegativeCorrectionCon->addOutput(kpB);
    moderateNegativeCorrectionCon->addOutput(kiS);
    moderateNegativeCorrectionCon->addOutput(kdS);
    fuzzy->addFuzzyRule(new FuzzyRule(13, smallNegativeErrorGrowingAnt, moderateNegativeCorrectionCon));

    // Rule 14: Small Negative Error Stable - balanced negative response
    FuzzyRuleAntecedent* smallNegativeErrorStableAnt = new FuzzyRuleAntecedent();
    smallNegativeErrorStableAnt->joinWithAND(errorNS, errorRateZE);
    FuzzyRuleConsequent* balancedNegativeResponseCon = new FuzzyRuleConsequent();
    balancedNegativeResponseCon->addOutput(kpM);
    balancedNegativeResponseCon->addOutput(kiM);
    balancedNegativeResponseCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(14, smallNegativeErrorStableAnt, balancedNegativeResponseCon));

    // Rule 15: Small Negative Error Decreasing - emphasize integral action
    FuzzyRuleAntecedent* smallNegativeErrorDecreasingAnt = new FuzzyRuleAntecedent();
    smallNegativeErrorDecreasingAnt->joinWithAND(errorNS, errorRatePS);
    FuzzyRuleConsequent* emphasizeIntegralNegativeCon = new FuzzyRuleConsequent();
    emphasizeIntegralNegativeCon->addOutput(kpS);
    emphasizeIntegralNegativeCon->addOutput(kiB);
    emphasizeIntegralNegativeCon->addOutput(kdS);
    fuzzy->addFuzzyRule(new FuzzyRule(15, smallNegativeErrorDecreasingAnt, emphasizeIntegralNegativeCon));

    // Additional edge case rules for robust performance
    // Rule 16: Large Positive Error Slightly Decreasing - maintain strong correction
    FuzzyRuleAntecedent* largePositiveErrorSlowDecreaseAnt = new FuzzyRuleAntecedent();
    largePositiveErrorSlowDecreaseAnt->joinWithAND(errorPB, errorRateNS);
    FuzzyRuleConsequent* maintainStrongCorrectionCon = new FuzzyRuleConsequent();
    maintainStrongCorrectionCon->addOutput(kpVB);
    maintainStrongCorrectionCon->addOutput(kiVS);
    maintainStrongCorrectionCon->addOutput(kdB);
    fuzzy->addFuzzyRule(new FuzzyRule(16, largePositiveErrorSlowDecreaseAnt, maintainStrongCorrectionCon));

    // Rule 17: Large Negative Error Slightly Decreasing - maintain strong negative correction
    FuzzyRuleAntecedent* largeNegativeErrorSlowDecreaseAnt = new FuzzyRuleAntecedent();
    largeNegativeErrorSlowDecreaseAnt->joinWithAND(errorNB, errorRatePS);
    FuzzyRuleConsequent* maintainStrongNegativeCorrectionCon = new FuzzyRuleConsequent();
    maintainStrongNegativeCorrectionCon->addOutput(kpVB);
    maintainStrongNegativeCorrectionCon->addOutput(kiVS);
    maintainStrongNegativeCorrectionCon->addOutput(kdB);
    fuzzy->addFuzzyRule(new FuzzyRule(17, largeNegativeErrorSlowDecreaseAnt, maintainStrongNegativeCorrectionCon));

    // Rule 18: Small Positive Error Rapidly Growing - prevent escalation
    FuzzyRuleAntecedent* smallPositiveErrorRapidGrowthAnt = new FuzzyRuleAntecedent();
    smallPositiveErrorRapidGrowthAnt->joinWithAND(errorPS, errorRatePB);
    FuzzyRuleConsequent* preventEscalationCon = new FuzzyRuleConsequent();
    preventEscalationCon->addOutput(kpVB);
    preventEscalationCon->addOutput(kiVS);
    preventEscalationCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(18, smallPositiveErrorRapidGrowthAnt, preventEscalationCon));

    // Rule 19: Small Negative Error Rapidly Growing - prevent negative escalation
    FuzzyRuleAntecedent* smallNegativeErrorRapidGrowthAnt = new FuzzyRuleAntecedent();
    smallNegativeErrorRapidGrowthAnt->joinWithAND(errorNS, errorRateNB);
    FuzzyRuleConsequent* preventNegativeEscalationCon = new FuzzyRuleConsequent();
    preventNegativeEscalationCon->addOutput(kpVB);
    preventNegativeEscalationCon->addOutput(kiVS);
    preventNegativeEscalationCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(19, smallNegativeErrorRapidGrowthAnt, preventNegativeEscalationCon));

    // Rule 20: At Setpoint Rapidly Drifting Positive - quick intervention
    FuzzyRuleAntecedent* atSetpointRapidPositiveDriftAnt = new FuzzyRuleAntecedent();
    atSetpointRapidPositiveDriftAnt->joinWithAND(errorZE, errorRatePB);
    FuzzyRuleConsequent* quickPositiveInterventionCon = new FuzzyRuleConsequent();
    quickPositiveInterventionCon->addOutput(kpB);
    quickPositiveInterventionCon->addOutput(kiS);
    quickPositiveInterventionCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(20, atSetpointRapidPositiveDriftAnt, quickPositiveInterventionCon));

    // Rule 21: At Setpoint Rapidly Drifting Negative - quick negative intervention
    FuzzyRuleAntecedent* atSetpointRapidNegativeDriftAnt = new FuzzyRuleAntecedent();
    atSetpointRapidNegativeDriftAnt->joinWithAND(errorZE, errorRateNB);
    FuzzyRuleConsequent* quickNegativeInterventionCon = new FuzzyRuleConsequent();
    quickNegativeInterventionCon->addOutput(kpB);
    quickNegativeInterventionCon->addOutput(kiS);
    quickNegativeInterventionCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(21, atSetpointRapidNegativeDriftAnt, quickNegativeInterventionCon));

    // Rule 22: Small Positive Error Rapidly Decreasing - smooth approach to setpoint
    FuzzyRuleAntecedent* smallPositiveErrorRapidDecreaseAnt = new FuzzyRuleAntecedent();
    smallPositiveErrorRapidDecreaseAnt->joinWithAND(errorPS, errorRateNB);
    FuzzyRuleConsequent* smoothApproachPositiveCon = new FuzzyRuleConsequent();
    smoothApproachPositiveCon->addOutput(kpM);
    smoothApproachPositiveCon->addOutput(kiB);
    smoothApproachPositiveCon->addOutput(kdVB);
    fuzzy->addFuzzyRule(new FuzzyRule(22, smallPositiveErrorRapidDecreaseAnt, smoothApproachPositiveCon));

    // Rule 23: Small Negative Error Rapidly Decreasing - smooth negative approach
    FuzzyRuleAntecedent* smallNegativeErrorRapidDecreaseAnt = new FuzzyRuleAntecedent();
    smallNegativeErrorRapidDecreaseAnt->joinWithAND(errorNS, errorRatePB);
    FuzzyRuleConsequent* smoothApproachNegativeCon = new FuzzyRuleConsequent();
    smoothApproachNegativeCon->addOutput(kpM);
    smoothApproachNegativeCon->addOutput(kiB);
    smoothApproachNegativeCon->addOutput(kdVB);
    fuzzy->addFuzzyRule(new FuzzyRule(23, smallNegativeErrorRapidDecreaseAnt, smoothApproachNegativeCon));

    // Rule 24: Large Positive Error Slightly Growing - strong sustained correction
    FuzzyRuleAntecedent* largePositiveErrorSlowGrowthAnt = new FuzzyRuleAntecedent();
    largePositiveErrorSlowGrowthAnt->joinWithAND(errorPB, errorRatePS);
    FuzzyRuleConsequent* strongSustainedCorrectionCon = new FuzzyRuleConsequent();
    strongSustainedCorrectionCon->addOutput(kpVB);
    strongSustainedCorrectionCon->addOutput(kiS);
    strongSustainedCorrectionCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(24, largePositiveErrorSlowGrowthAnt, strongSustainedCorrectionCon));

    // Rule 25: Large Negative Error Slightly Growing - strong sustained negative correction
    FuzzyRuleAntecedent* largeNegativeErrorSlowGrowthAnt = new FuzzyRuleAntecedent();
    largeNegativeErrorSlowGrowthAnt->joinWithAND(errorNB, errorRateNS);
    FuzzyRuleConsequent* strongSustainedNegativeCorrectionCon = new FuzzyRuleConsequent();
    strongSustainedNegativeCorrectionCon->addOutput(kpVB);
    strongSustainedNegativeCorrectionCon->addOutput(kiS);
    strongSustainedNegativeCorrectionCon->addOutput(kdM);
    fuzzy->addFuzzyRule(new FuzzyRule(25, largeNegativeErrorSlowGrowthAnt, strongSustainedNegativeCorrectionCon));

    fuzzyInitialized = true;
}

void FuzzyPIDController::updateGains(float error, float errorRate) {
    if (!config.enableFuzzyAdaptation || !fuzzyInitialized) {
        return;
    }

    const float e = clamp(error, -config.errorRange, config.errorRange);
    const float de = clamp(errorRate, -config.errorRateRange, config.errorRateRange);

    fuzzy->setInput(1, e);
    fuzzy->setInput(2, de);
    fuzzy->fuzzify();

    float kpScale = fuzzy->defuzzify(1);
    float kiScale = fuzzy->defuzzify(2);
    float kdScale = fuzzy->defuzzify(3);

    const bool noActivation = (kpScale == 0.0f && kiScale == 0.0f && kdScale == 0.0f);
    if (noActivation) {
        pidController.setParameters(currentKp, currentKi, currentKd);
        return;
    }

    currentKp = config.kpMin + (kpScale / 100.0f) * (config.kpMax - config.kpMin);
    currentKi = config.kiMin + (kiScale / 100.0f) * (config.kiMax - config.kiMin);
    currentKd = config.kdMin + (kdScale / 100.0f) * (config.kdMax - config.kdMin);

    pidController.setParameters(currentKp, currentKi, currentKd);
}

float FuzzyPIDController::compute(float setpoint, float actual) {
    const uint64_t nowUs = esp_timer_get_time();
    const float error = angDiffDeg(setpoint, actual);

    float errorRate = 0.0f;
    if (lastTimeUs != 0) {
        const float dt = (nowUs - lastTimeUs) * 1e-6f;
        if (dt >= 1e-4f && dt <= 0.2f) {
            const float prevErr = lastError;
            const float dErr = error - prevErr;
            errorRate = dErr / dt;
        }
    }

    lastTimeUs = nowUs;
    lastError = error;

    updateGains(error, errorRate);
    return pidController.compute(setpoint, actual);
}

void FuzzyPIDController::reset() {
    pidController.reset();
    lastError = 0.0f;
    lastTimeUs = 0;

    currentKp = config.basePidConfig.kp;
    currentKi = config.basePidConfig.ki;
    currentKd = config.basePidConfig.kd;
    pidController.setParameters(currentKp, currentKi, currentKd);
}

bool FuzzyPIDController::isSettled() const { return pidController.isSettled(); }

void FuzzyPIDController::getCurrentGains(float& kp, float& ki, float& kd) const {
    kp = currentKp;
    ki = currentKi;
    kd = currentKd;
}

float FuzzyPIDController::getLastError() const { return pidController.getLastError(); }

float FuzzyPIDController::getOutput() const { return pidController.getOutput(); }

void FuzzyPIDController::setFuzzyAdaptation(bool enable) {
    config.enableFuzzyAdaptation = enable;

    if (enable && !fuzzyInitialized) {
        initializeFuzzySystem();
    }

    if (!enable) {
        currentKp = config.basePidConfig.kp;
        currentKi = config.basePidConfig.ki;
        currentKd = config.basePidConfig.kd;
        pidController.setParameters(currentKp, currentKi, currentKd);
    }
}
}  // namespace PID
}  // namespace DC_Motor_Controller_Firmware
