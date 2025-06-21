/**
 * @file CommLogicHandler.hpp
 * @brief High-level logic task connecting USB communication and motor control.
 *
 * This class runs a FreeRTOS task that bridges MotorCommHandler (USB protocol),
 * MotorController (motion logic), and Encoder (position feedback). It handles
 * position targets, PID updates, and motion state reporting in a closed loop.
 */

#pragma once

#include "Encoder.hpp"
#include "MotorCommHandler.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motorControl.hpp"

namespace DC_Motor_Controller_Firmware::Logic {

/**
 * @class CommLogicHandler
 * @brief High-level control task that connects USB commands with motor
 * behavior.
 *
 * This class launches a background FreeRTOS task to:
 * - Receive and apply new position targets
 * - Handle runtime PID tuning commands
 * - Monitor position feedback
 * - Send motion completion signals
 */
class CommLogicHandler {
public:
  /**
   * @brief Constructor.
   * @param comm Reference to MotorCommHandler instance for USB communication
   * @param motor Reference to MotorController for control execution
   * @param enc Reference to Encoder for feedback
   */
  CommLogicHandler(Communication::MotorCommHandler &comm,
                   Control::MotorController &motor, Encoder::Encoder &enc);

  /**
   * @brief Starts the FreeRTOS control logic task.
   *
   * The task continuously processes USB commands and controls the motor.
   * If already running, calling this does nothing.
   */
  void startTask();

private:
  Communication::MotorCommHandler &motorComm; ///< USB communication interface
  Control::MotorController &motorControl;     ///< Motor control interface
  Encoder::Encoder &encoder;                  ///< Encoder feedback provider

  TaskHandle_t taskHandle = nullptr; ///< Logic task handle

  float kp = 0.0f, ki = 0.0f, kd = 0.0f; ///< Current PID values
  float current = 0.0f;                  ///< Current motor position
  float offset = 0.0f;       ///< Requested offset from current position
  float targetDegree = 0.0f; ///< Final target position
  bool settled = false;      ///< Whether target has been reached

  bool getPIDValuesFirsTime = true; ///< Initialization flag for PID sync

  /**
   * @brief Static task function executed in FreeRTOS context.
   * @param param Pointer to CommLogicHandler instance
   *
   * Handles:
   * - Reading encoder position
   * - Applying incoming target/PID commands
   * - Notifying position reached
   * - Echoing back current PID values
   */
  static void taskFunc(void *param);
};

} // namespace DC_Motor_Controller_Firmware::Logic
