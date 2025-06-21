/**
 * @file MotorController.hpp
 * @brief High-level closed-loop controller for positioning a DC motor.
 *
 * Combines encoder feedback, a DRV8876 motor driver, and a PID controller to
 * move a DC motor to a target position with configurable motion parameters.
 */

#pragma once

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "PID.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @struct MotorControllerConfig
 * @brief Configuration structure for motion constraints and drift detection.
 */
struct MotorControllerConfig {
  float minSpeed = 2.0f;              ///< Minimum allowed motor speed (%)
  float maxSpeed = 100.0f;            ///< Maximum allowed motor speed (%)
  float minErrorToMove = 0.2f;        ///< Minimum error required to trigger motion (deg)
  float driftThreshold = 1.0f;        ///< Position drift threshold to detect overshoot (deg)
  float stuckPositionEpsilon = 0.05f; ///< Threshold to detect motor is not moving (deg)
  int stuckCountLimit = 50;           ///< Number of update cycles before declaring "stuck"
  int pidWarmupLimit = 10;            ///< Cycles to allow PID to stabilize before drift detection
};

/**
 * @class MotorController
 * @brief Controls a DC motor using encoder feedback and PID regulation.
 *
 * This class provides high-level closed-loop control by coordinating encoder feedback,
 * PID output, and motor driver commands to move a motor to a specified angular position.
 */
class MotorController {
public:
  /**
   * @brief Constructor for the motor controller.
   * 
   * @param enc Reference to Encoder instance
   * @param drv Reference to DRV8876 motor driver
   * @param pid Reference to PID controller
   * @param cfg Optional control parameters (default config used if omitted)
   */
  MotorController(Encoder::Encoder &enc,
                  DRV8876::DRV8876 &drv,
                  PID::PIDController &pid,
                  const MotorControllerConfig &cfg = MotorControllerConfig());

  /**
   * @brief Destructor.
   *
   * If the control task is running, it will be deleted automatically.
   */
  ~MotorController();

  /**
   * @brief Starts the FreeRTOS task that handles periodic PID-based motor control.
   *
   * Creates a background task using `xTaskCreate()` that continuously calls `update()`.
   * If the task is already running, this method does nothing.
   */
  void startTask();

  /**
   * @brief Set the desired target position in degrees.
   * @param degrees New target angle
   */
  void setTarget(float degrees);

  /**
   * @brief Set PID gain parameters.
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void setPID(float kp, float ki, float kd);

  /**
   * @brief Get the current PID parameters.
   * @param kp Output: proportional gain
   * @param ki Output: integral gain
   * @param kd Output: derivative gain
   */
  void getPID(float &kp, float &ki, float &kd);

  /**
   * @brief Run the control logic once. Should be called periodically if not using a task.
   */
  void update();

  /**
   * @brief Check if the motor has reached its target and settled.
   * @return true if motion is complete, false otherwise
   */
  bool isMotionDone() const;

private:
  Encoder::Encoder &encoder;          ///< Encoder feedback interface
  DRV8876::DRV8876 &motor;            ///< Motor driver interface
  PID::PIDController &pid;            ///< PID regulator instance
  MotorControllerConfig config;       ///< User-defined motion parameters

  float target = 0;                   ///< Target position in degrees
  bool motionDone = true;             ///< Flag: true if motion is complete
  float lastPos = 0;                  ///< Last encoder position
  int stuckCounter = 0;               ///< Count of how long position hasn't changed
  int pidWarmupCounter = 0;           ///< Warm-up period before drift/stuck detection

  TaskHandle_t taskHandle = nullptr;  ///< Optional background control task

  /**
   * @brief Static task entry point for FreeRTOS task created by startTask().
   * @param param Pointer to MotorController instance
   */
  static void taskFunc(void *param);
};

} // namespace DC_Motor_Controller_Firmware::Control
