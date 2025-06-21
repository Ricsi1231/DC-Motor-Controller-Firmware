#include "MotorCommHandler.hpp"
#include <cstdlib>
#include <cstring>

#include "esp_log.h"

namespace DC_Motor_Controller_Firmware {
namespace Communication {
MotorCommHandler::MotorCommHandler(USB::USB &usbRef) : usb(usbRef) {}

MotorCommHandler::~MotorCommHandler() {}

void MotorCommHandler::process() {
  uint8_t buffer[128];
  size_t len = 0;

  if (usb.receiveData(buffer, &len) == ESP_OK && len > 0) {
    buffer[len] = '\0';
    parseMessage(reinterpret_cast<const char *>(buffer));
  }
}

void MotorCommHandler::startTask() {
  if (taskHandle == nullptr) {
    xTaskCreate(commTaskWrapper, "MotorCommTask", 4096, this, 10, &taskHandle);
  }
}

void MotorCommHandler::commTaskWrapper(void *param) {
  auto *self = static_cast<MotorCommHandler *>(param);
  while (true) {
    self->process();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void MotorCommHandler::parseMessage(const char *msg) {
  portENTER_CRITICAL(&spinlock);
  if (strncmp(msg, MSG_SET_DEG, strlen(MSG_SET_DEG)) == 0) {
    targetDegrees = strtof(msg + strlen(MSG_SET_DEG), nullptr);
    newTarget = true;
  } else if (strncmp(msg, MSG_SET_PID, strlen(MSG_SET_PID)) == 0) {
    int ret =
        sscanf(msg + strlen(MSG_SET_PID), "%f,%f,%f", &pidKp, &pidKi, &pidKd);
    if (ret == 3) {
      newPID = true;
      portEXIT_CRITICAL(&spinlock);
      usb.sendString("ACK\n");
      return;
    } else {
      portEXIT_CRITICAL(&spinlock);
      usb.sendString("ERR\n");
      return;
    }
  } else if (strcmp(msg, MSG_GET_PID) == 0) {
    pidRequested = true;
  } else if (strcmp(msg, MSG_STOP) == 0) {
    stopRequested = true;
  } else if (strcmp(msg, MSG_ENABLE) == 0) {
    motorEnabled = true;
  } else if (strcmp(msg, MSG_DISABLE) == 0) {
    motorEnabled = false;
  } else if (strcmp(msg, MSG_RESET) == 0) {
    targetDegrees = 0;
    pidKp = pidKi = pidKd = 0;
    newTarget = newPID = pidRequested = stopRequested = false;
  }
  portEXIT_CRITICAL(&spinlock);
}

void MotorCommHandler::sendMotorState(float degrees) {
  char msg[64];

  snprintf(msg, sizeof(msg), "%s%.2f\n", MSG_MOTOR_POS, degrees);
  usb.sendString(msg);
}

esp_err_t MotorCommHandler::sendPIDParams(float kp, float ki, float kd) {
  char msg[64];
  esp_err_t usbState = ESP_OK;

  snprintf(msg, sizeof(msg), "%s%.2f,%.2f,%.2f\n", MSG_PID_REPLY, kp, ki, kd);
  ESP_LOGI("Comm", "sendPIDParams: %s", msg); // <- debug
  usbState = usb.sendString(msg);

  if (usbState == ESP_OK) {
    pidRequested = false;
  }

  return usbState;

  // usb.flushRxBuffer();
}

void MotorCommHandler::clearTarget() {
  portENTER_CRITICAL(&spinlock);
  targetDegrees = 0;
  newTarget = false;
  portEXIT_CRITICAL(&spinlock);
}

void MotorCommHandler::notifyMotorPositionReached() {
  usb.sendString(MSG_REACHED);
}

float MotorCommHandler::getTargetDegrees() {
  portENTER_CRITICAL(&spinlock);
  float value = targetDegrees;
  newTarget = false;
  portEXIT_CRITICAL(&spinlock);
  return value;
}

void MotorCommHandler::getPIDParams(float &kp, float &ki, float &kd) {
  portENTER_CRITICAL(&spinlock);
  kp = pidKp;
  ki = pidKi;
  kd = pidKd;
  newPID = false;
  portEXIT_CRITICAL(&spinlock);
}

void MotorCommHandler::clearStopFlag(bool stop) {
  portENTER_CRITICAL(&spinlock);
  stopRequested = stop;
  portEXIT_CRITICAL(&spinlock);
}

bool MotorCommHandler::isNewPIDReceived() const {
  bool value;
  taskENTER_CRITICAL(&spinlock);
  value = newPID;
  taskEXIT_CRITICAL(&spinlock);
  return value;
}

void MotorCommHandler::clearPIDRequest() {
  portENTER_CRITICAL(&spinlock);
  pidRequested = false;
  portEXIT_CRITICAL(&spinlock);
}

bool MotorCommHandler::isNewTargetReceived() const {
  bool value;
  taskENTER_CRITICAL(&spinlock);
  value = newTarget;
  taskEXIT_CRITICAL(&spinlock);
  return value;
}

bool MotorCommHandler::wasPIDRequested() const {
  bool value;
  portENTER_CRITICAL(&spinlock);
  value = pidRequested;
  portEXIT_CRITICAL(&spinlock);
  return value;
}

bool MotorCommHandler::isStopRequested() const {
  bool value;
  portENTER_CRITICAL(&spinlock);
  value = stopRequested;
  portEXIT_CRITICAL(&spinlock);
  return value;
}

bool MotorCommHandler::isMotorEnabled() const {
  bool value;
  portENTER_CRITICAL(&spinlock);
  value = motorEnabled;
  portEXIT_CRITICAL(&spinlock);
  return value;
}

bool MotorCommHandler::isUSBOpen() const {
  bool value;
  portENTER_CRITICAL(&spinlock);
  value = usb.usbIsConnected();
  portEXIT_CRITICAL(&spinlock);
  return value;
}

} // namespace Communication
} // namespace DC_Motor_Controller_Firmware
