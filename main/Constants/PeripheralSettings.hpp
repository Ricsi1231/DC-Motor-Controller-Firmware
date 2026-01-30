#pragma once

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "PID.hpp"
#include "motorControl.hpp"

using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;
using namespace DC_Motor_Controller_Firmware::PID;
using namespace DC_Motor_Controller_Firmware::Control;

DRV8876Config motorConfig = {
    .phPin = GPIO_NUM_4,
    .enPin = GPIO_NUM_5,
    .nFault = GPIO_NUM_6,
    .nSleep = GPIO_NUM_7,
    .pwmChannel = LEDC_CHANNEL_0,
    .resolution = LEDC_TIMER_10_BIT,
    .frequency = 20000,
    .minFrequency = 100,
    .maxFrequency = 100000,
    .rampStepPercent = 5,
    .rampStepDelayMs = 10,
    .minEffectivePwmPercent = 0,
};

EncoderConfig encoderConfig = {
    .pinA = GPIO_NUM_2,
    .pinB = GPIO_NUM_1,
    .unitConfig = {.low_limit = -1000,
                   .high_limit = 1000,
                   .flags = {
                       .accum_count = true,
                   }},
    .pulsesPerRevolution = 1024,
    .filterThresholdNs = 0,
    .rpmCalcPeriodUs = 10000,
    .maxRpm = 0,
    .enableWatchPoint = true,
    .watchLowLimit = 0,
    .watchHighLimit = 0,
    .openCollectorInputs = false,
    .rpmBlendThreshold = 60,
    .rpmBlendBand = 20,
    .speedFilter = {.filterType = SpeedFilterType::EMA, .emaAlpha = 0.3f, .iirCutoffHz = 0.0f, .sampleRateHz = 0},
    .direction = {.hysteresisThreshold = 2, .debounceTimeMs = 50, .enableHysteresis = true},
};

PidConfig defaultConfig = {
    .kp = 1.0f,
    .ki = 0.1f,
    .kd = 0.08f,
    .maxOutput = 100.0f,
    .maxIntegral = 300.0f,
    .errorEpsilon = 0.1f,
    .speedEpsilon = 0.2f,
    .errorTimeoutSec = 0.5f,
    .stuckTimeoutSec = 0.5f,
};

MotorControllerConfig motorCfg = {
    .minSpeed = 2.0f,
    .maxSpeed = 100.0f,
    .minErrorToMove = 0.2f,
    .countsPerRevolution = 1024,
    .stall = {.stuckPositionEpsilon = 0.05f,
              .stuckCountLimit = 50,
              .pidWarmupLimit = 10,
              .minErrorToMove = 0.2f},
};
