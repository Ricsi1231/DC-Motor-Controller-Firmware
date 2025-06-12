#pragma once

#include "USB.hpp"
namespace DC_Motor_Controller_Firmware {
namespace Communication {
class MotorCommHandler {
public:
  explicit MotorCommHandler(USB::USB &usbRef);

  void process();

  void sendMotorState(float degrees);
  void sendPIDParams(float kp, float ki, float kd);
  void notifyMotorPositionReached();

  void clearTarget();

  float getTargetDegrees();
  void getPIDParams(float &kp, float &ki, float &kd);

  bool isNewTargetReceived() const;
  bool isNewPIDReceived() const;
  bool wasPIDRequested() const;
  bool isStopRequested() const;
  bool isMotorEnabled() const;

  void clearStopFlag(bool stopRequested = false);

private:
  USB::USB &usb;

  float targetDegrees = 0.0f;
  bool newTarget = false;

  float pidKp = 0.0f;
  float pidKi = 0.0f;
  float pidKd = 0.0f;
  bool newPID = false;

  bool pidRequested = false;
  bool stopRequested = false;
  bool motorEnabled = true;

  void parseMessage(const char *msg);

  static constexpr const char *MSG_SET_DEG = "SET_DEG:";
  static constexpr const char *MSG_SET_PID = "SET_PID:";
  static constexpr const char *MSG_GET_PID = "GET_PID";
  static constexpr const char *MSG_PID_REPLY = "PID:";
  static constexpr const char *MSG_MOTOR_POS = "MOTOR_POS:";
  static constexpr const char *MSG_REACHED = "MOTOR_REACHED";
  static constexpr const char *MSG_STOP = "STOP";
  static constexpr const char *MSG_ENABLE = "ENABLE";
  static constexpr const char *MSG_DISABLE = "DISABLE";
  static constexpr const char *MSG_RESET = "RESET_ALL";
  static constexpr const char *MSG_GET_STATE = "GET_STATE";
};
} // namespace Communication
} // namespace DC_Motor_Controller_Firmware
