#pragma once

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "MotorControlConfig.hpp"
#include "PID.hpp"

using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;
using namespace DC_Motor_Controller_Firmware::PID;
using namespace DC_Motor_Controller_Firmware::Control;

constexpr gpio_num_t PH_PIN = GPIO_NUM_4;
constexpr gpio_num_t EN_PIN = GPIO_NUM_5;
constexpr gpio_num_t FAULT_PIN = GPIO_NUM_6;
constexpr gpio_num_t SLEEP_PIN = GPIO_NUM_7;
constexpr gpio_num_t ENCODER_A = GPIO_NUM_1;
constexpr gpio_num_t ENCODER_B = GPIO_NUM_2;

constexpr DRV8876Config motorDriverConfig = {
    .phPin = PH_PIN,
    .enPin = EN_PIN,
    .nFault = FAULT_PIN,
    .nSleep = SLEEP_PIN,
    .pwmChannel = LEDC_CHANNEL_0,
    .resolution = LEDC_TIMER_10_BIT,
    .frequency = 20000,
    .minFrequency = 100,
    .maxFrequency = 100000,
    .rampStepPercent = 5,
    .rampStepDelayMs = 10,
    .minEffectivePwmPercent = 5,
};

constexpr pcnt_unit_config_t pcntUnitConfig = {
    .low_limit = -32767,
    .high_limit = 32767,
    .intr_priority = 0,
    .flags = {.accum_count = true},
};

constexpr SpeedFilterConfig speedFilterConfig = {
    .filterType = SpeedFilterType::EMA,
    .emaAlpha = 0.3f,
    .iirCutoffHz = 2.0f,
    .sampleRateHz = 100,
};

constexpr DirectionConfig directionConfig = {
    .hysteresisThreshold = 8,
    .debounceTimeMs = 100,
    .enableHysteresis = true,
};

constexpr EncoderConfig encoderConfig = {
    .pinA = ENCODER_A,
    .pinB = ENCODER_B,
    .unitConfig = pcntUnitConfig,
    .pulsesPerRevolution = 1024,
    .filterThresholdNs = 1000,
    .rpmCalcPeriodUs = 100000,
    .maxRpm = 5000,
    .enableWatchPoint = true,
    .watchLowLimit = 0,
    .watchHighLimit = 0,
    .openCollectorInputs = false,
    .rpmBlendThreshold = 10,
    .rpmBlendBand = 3,
    .speedFilter = speedFilterConfig,
    .direction = directionConfig,
};

PidConfig defaultConfig = {
    .kp = 1.0f,
    .ki = 0.04f,
    .kd = 0.02f,
    .maxOutput = 100.0f,
    .maxIntegral = 300.0f,
    .errorEpsilon = 0.1f,
    .speedEpsilon = 0.2f,
    .errorTimeoutSec = 0.5f,
    .stuckTimeoutSec = 0.5f,
};

MotorControlConfig motorCfg = {
    .minSpeed = 2.0f,
    .maxSpeed = 100.0f,
    .minErrorToMove = 0.2f,
    .driftThreshold = 1.0f,
    .stuckPositionEpsilon = 0.05f,
    .stuckCountLimit = 50,
    .pidWarmupLimit = 10,
};
