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
  float minSpeed = 2.0f;       ///< Minimum allowed motor speed (%)
  float maxSpeed = 100.0f;     ///< Maximum allowed motor speed (%)
  float minErrorToMove = 0.2f; ///< Minimum error required to move motor (deg)
  float driftThreshold = 1.0f; ///< Threshold to detect position drift (deg)
  float stuckPositionEpsilon =
      0.05f;                ///< Position delta to detect motor is stuck (deg)
  int stuckCountLimit = 50; ///< Number of checks before declaring stuck
  int pidWarmupLimit =
      10; ///< Number of cycles before PID output becomes active
};

/**
 * @class MotorController
 * @brief Controls a DC motor using encoder feedback and PID regulation.
 */
class MotorController {
public:
  /**
   * @brief Constructor for the motor controller.
   *
   * @param enc Reference to encoder instance
   * @param drv Reference to motor driver (DRV8876)
   * @param pid Reference to PID controller
   * @param cfg Optional configuration (default values used if omitted)
   */
  MotorController(Encoder::Encoder &enc, DRV8876::DRV8876 &drv,
                  PID::PIDController &pid,
                  const MotorControllerConfig &cfg = MotorControllerConfig());

  /**
   * @brief Destructor.
   */
  ~MotorController();

  void startTask();

  /**
   * @brief Set target position in degrees.
   *
   * @param degrees Desired target position
   */
  void setTarget(float degrees);

  /**
   * @brief Set PID gain parameters.
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void setPID(float kp, float ki, float kd);

  /**
   * @brief Get current PID gain parameters.
   *
   * @param kp Output parameter for proportional gain
   * @param ki Output parameter for integral gain
   * @param kd Output parameter for derivative gain
   */
  void getPID(float &kp, float &ki, float &kd);

  /**
   * @brief Update motor control logic. Should be called periodically.
   */
  void update();

  /**
   * @brief Check if motor has reached its target position.
   *
   * @return true if motion is complete
   * @return false if still moving
   */
  bool isMotionDone() const;

private:
  Encoder::Encoder &encoder;    ///< Encoder for position feedback
  DRV8876::DRV8876 &motor;      ///< Motor driver for control
  PID::PIDController &pid;      ///< PID controller for motion logic
  MotorControllerConfig config; ///< Motion configuration

  TaskHandle_t taskHandle = nullptr;
  static void taskFunc(void *param);

  float target = 0;         ///< Current target position (deg)
  bool motionDone = true;   ///< Motion completion status
  float lastPos = 0;        ///< Last recorded position (deg)
  int stuckCounter = 0;     ///< Counter for stuck detection
  int pidWarmupCounter = 0; ///< Counter before PID is fully active
};

} // namespace DC_Motor_Controller_Firmware::Control
