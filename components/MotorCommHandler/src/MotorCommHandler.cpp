#include "MotorCommHandler.hpp"
#include <cstdlib>
#include <cstring>

#include "esp_log.h"

namespace DC_Motor_Controller_Firmware {
namespace Communication {
MotorCommHandler::MotorCommHandler(USB::USB &usbRef) : usb(usbRef) {}

void MotorCommHandler::process() {
  uint8_t buffer[128];
  size_t len = 0;

  if (usb.receiveData(buffer, &len) == ESP_OK && len > 0) {
    buffer[len] = '\0';
    parseMessage(reinterpret_cast<const char *>(buffer));
  }
}

void MotorCommHandler::parseMessage(const char *msg) {
  // ESP_LOGI("PARSE", "Received raw msg: '%s'", msg);

  if (strncmp(msg, MSG_SET_DEG, strlen(MSG_SET_DEG)) == 0) {
    targetDegrees = strtof(msg + strlen(MSG_SET_DEG), nullptr);
    newTarget = true;
  } else if (strncmp(msg, MSG_SET_PID, strlen(MSG_SET_PID)) == 0) {
    int ret =
        sscanf(msg + strlen(MSG_SET_PID), "%f,%f,%f", &pidKp, &pidKi, &pidKd);

    if (ret == 3) {
      newPID = true;
      usb.sendString("ACK\n");
    } else {
      usb.sendString("ERR\n");
    }
  } else if (strcmp(msg, MSG_GET_PID) == 0) {
    pidRequested = true;
  } else if (strncmp(msg, MSG_STOP, strlen(MSG_STOP)) == 0) {
    stopRequested = true;
  } else if (strncmp(msg, MSG_ENABLE, strlen(MSG_ENABLE)) == 0) {
    motorEnabled = true;
  } else if (strncmp(msg, MSG_DISABLE, strlen(MSG_DISABLE)) == 0) {
    motorEnabled = false;
  } else if (strncmp(msg, MSG_RESET, strlen(MSG_RESET)) == 0) {
    targetDegrees = 0;

    pidKp = 0;
    pidKi = 0;
    pidKd = 0;

    newTarget = false;
    newPID = false;
    pidRequested = false;
    stopRequested = false;
  } else if (strncmp(msg, MSG_GET_STATE, strlen(MSG_GET_STATE)) == 0) {
    char msg[128];

    snprintf(msg, sizeof(msg),
             "STATE: DEG=%.2f PID=%.2f,%.2f,%.2f ENABLED=%d STOP=%d\n",
             targetDegrees, pidKp, pidKi, pidKd, motorEnabled, stopRequested);
    usb.sendString(msg);
  } else {
    ESP_LOGI("USB", "Wrong message from HOST device");
    usb.sendString("ERR:UNKNOWN_COMMAND\n");
  }
}

void MotorCommHandler::sendMotorState(float degrees) {
  char msg[64];

  snprintf(msg, sizeof(msg), "%s%.2f\n", MSG_MOTOR_POS, degrees);
  usb.sendString(msg);
}

void MotorCommHandler::sendPIDParams(float kp, float ki, float kd) {
  char msg[64];
  esp_err_t usbState = ESP_OK;

  snprintf(msg, sizeof(msg), "%s%.2f,%.2f,%.2f\n", MSG_PID_REPLY, kp, ki, kd);
  usbState = usb.sendString(msg);

  if (usbState == ESP_OK) {
    pidRequested = false;
  }

  // usb.flushRxBuffer();
}

void MotorCommHandler::clearTarget() {
  targetDegrees = 0;
  newTarget = false;
}

void MotorCommHandler::notifyMotorPositionReached() {
  usb.sendString(MSG_REACHED);
}

float MotorCommHandler::getTargetDegrees() {
  newTarget = false;
  // usb.flushRxBuffer();

  return targetDegrees;
}

void MotorCommHandler::getPIDParams(float &kp, float &ki, float &kd) {
  kp = pidKp;
  ki = pidKi;
  kd = pidKd;
  // usb.flushRxBuffer();

  newPID = false;
}

void MotorCommHandler::clearStopFlag(bool stopRequested) {
  this->stopRequested = stopRequested;
}

bool MotorCommHandler::isNewTargetReceived() const { return newTarget; }
bool MotorCommHandler::isNewPIDReceived() const { return newPID; }
bool MotorCommHandler::wasPIDRequested() const { return pidRequested; }
bool MotorCommHandler::isStopRequested() const { return stopRequested; }
bool MotorCommHandler::isMotorEnabled() const { return motorEnabled; }
} // namespace Communication
} // namespace DC_Motor_Controller_Firmware
