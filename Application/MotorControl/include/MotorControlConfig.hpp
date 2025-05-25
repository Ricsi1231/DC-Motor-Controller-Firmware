#pragma once

namespace DC_Motor_Controller_Firmware {
namespace motorControl {
namespace config {

constexpr float PID_KP              = 1.0f;
constexpr float PID_KI              = 0.5f;
constexpr float PID_KD              = 0.1f;
constexpr float PID_KF              = 0.0f;

constexpr float PID_OUTPUT_MIN      = -100.0f;
constexpr float PID_OUTPUT_MAX      = 100.0f;

constexpr float PID_MAX_I_OUTPUT    = 50.0f;
constexpr bool  PID_REVERSED        = false;
constexpr float PID_INITIAL_SETPOINT= 0.0f;
constexpr float PID_RAMP_RATE       = 10.0f;
constexpr float PID_SETPOINT_RANGE  = 5.0f;
constexpr float PID_FILTER_STRENGTH = 0.1f;

} // namespace config
} // namespace motorControl
} // namespace DC_Motor_Controller_Firmware
