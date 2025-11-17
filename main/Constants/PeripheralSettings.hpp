#include "MotorControl.hpp"
#include "PID.hpp"

#pragma once

using namespace DC_Motor_Controller_Firmware::PID;
using namespace DC_Motor_Controller_Firmware::Control;

ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

uint16_t ppr = 1024;
pcnt_unit_config_t pcntUnit = {.low_limit = -1000,
                               .high_limit = 1000,
                               .flags = {
                                   .accum_count = true,
                               }};

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

MotorControllerConfig motorCfg = {
    .minSpeed = 2.0f,
    .maxSpeed = 100.0f,
    .minErrorToMove = 0.2f,
    .driftThreshold = 1.0f,
    .stuckPositionEpsilon = 0.05f,
    .stuckCountLimit = 50,
    .pidWarmupLimit = 10,
};