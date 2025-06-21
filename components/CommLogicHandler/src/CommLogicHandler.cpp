#include "CommLogicHandler.hpp"
#include "esp_task_wdt.h"
#include <cmath>

using namespace DC_Motor_Controller_Firmware::Logic;

CommLogicHandler::CommLogicHandler(Communication::MotorCommHandler &comm,
                                   Control::MotorController &motor,
                                   Encoder::Encoder &enc)
    : motorComm(comm), motorControl(motor), encoder(enc) {}

void CommLogicHandler::startTask() {
  if (taskHandle == nullptr) {
    xTaskCreate(taskFunc, "CommLogicTask", 4096, this, 5, &taskHandle);
  }
}

void CommLogicHandler::taskFunc(void *param) {
  auto *self = static_cast<CommLogicHandler *>(param);

  if (self->getPIDValuesFirsTime) {
    self->motorControl.getPID(self->kp, self->ki, self->kd);
    self->getPIDValuesFirsTime = false;
  }

  while (true) {
    self->current = self->encoder.getPositionInDegrees();

    if (self->motorComm.isNewTargetReceived()) {
      self->offset = self->motorComm.getTargetDegrees();
      self->targetDegree = self->current + self->offset;

      self->motorControl.setTarget(self->targetDegree);
      self->motorComm.clearTarget();
      self->settled = false;
    }

    if (self->motorComm.isNewPIDReceived()) {
      self->motorComm.getPIDParams(self->kp, self->ki, self->kd);
      self->motorControl.setPID(self->kp, self->ki, self->kd);
    }

    if (self->motorComm.wasPIDRequested()) {
      if (self->motorComm.isUSBOpen()) {
        self->motorControl.getPID(self->kp, self->ki, self->kd);
        esp_err_t res =
            self->motorComm.sendPIDParams(self->kp, self->ki, self->kd);
        if (res != ESP_OK) {
          self->motorComm.clearPIDRequest();
        }
      } else {
        self->motorComm.clearPIDRequest();
      }
    }

    if ((fabsf(self->encoder.getPositionInDegrees() - self->targetDegree)) &&
        !self->settled) {
      if (self->motorComm.isUSBOpen()) {
        self->motorComm.notifyMotorPositionReached();
      }
      self->settled = true;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
