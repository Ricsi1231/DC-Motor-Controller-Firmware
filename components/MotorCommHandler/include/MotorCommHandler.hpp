/**
 * @file MotorCommHandler.hpp
 * @brief Communication handler for motor control commands via USB.
 *
 * Handles incoming serial commands (target position, PID params,
 * enable/disable, etc.) and sends feedback (motor reached, current position,
 * etc.) using the USB interface.
 */

#pragma once

#include "USB.hpp"

namespace DC_Motor_Controller_Firmware {
namespace Communication {

/**
 * @class MotorCommHandler
 * @brief Parses motor-related commands and responses over USB.
 */
class MotorCommHandler {
public:
  /**
   * @brief Constructor.
   *
   * @param usbRef Reference to initialized USB communication interface
   */
  explicit MotorCommHandler(USB::USB &usbRef);

  /**
   * @brief Destructor.
   */
  ~MotorCommHandler();

  /**
   * @brief Main polling method to read and process USB messages.
   */
  void process();

  /**
   * @brief Send motor's current position in degrees.
   *
   * @param degrees Position in degrees
   */
  void sendMotorState(float degrees);

  /**
   * @brief Send current PID parameters (for response to GET_PID).
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void sendPIDParams(float kp, float ki, float kd);

  /**
   * @brief Notify the host that the motor has reached its target.
   */
  void notifyMotorPositionReached();

  /**
   * @brief Clear current target angle and flags.
   */
  void clearTarget();

  /**
   * @brief Get latest received target angle in degrees.
   *
   * @return float Target position
   */
  float getTargetDegrees();

  /**
   * @brief Get latest received PID parameters.
   *
   * @param kp Proportional gain (output)
   * @param ki Integral gain (output)
   * @param kd Derivative gain (output)
   */
  void getPIDParams(float &kp, float &ki, float &kd);

  /**
   * @brief Check if a new target angle has been received.
   *
   * @return true if new target is available
   * @return false otherwise
   */
  bool isNewTargetReceived() const;

  /**
   * @brief Check if new PID parameters have been received.
   *
   * @return true if updated PID available
   * @return false otherwise
   */
  bool isNewPIDReceived() const;

  /**
   * @brief Check if a PID value read (GET_PID) was requested by host.
   *
   * @return true if host requested PID
   * @return false otherwise
   */
  bool wasPIDRequested() const;

  /**
   * @brief Check if stop was requested by host.
   *
   * @return true if STOP command received
   * @return false otherwise
   */
  bool isStopRequested() const;

  /**
   * @brief Check if motor is currently enabled.
   *
   * @return true if enabled
   * @return false if DISABLE was received
   */
  bool isMotorEnabled() const;

  /**
   * @brief Clear stop flag after processing STOP request.
   *
   * @param stopRequested Set to true to keep stop active
   */
  void clearStopFlag(bool stopRequested = false);

private:
  USB::USB &usb; ///< USB interface reference

  float targetDegrees = 0.0f; ///< Target position in degrees
  bool newTarget = false;     ///< Flag: new target received

  float pidKp = 0.0f;  ///< PID - proportional
  float pidKi = 0.0f;  ///< PID - integral
  float pidKd = 0.0f;  ///< PID - derivative
  bool newPID = false; ///< Flag: new PID received

  bool pidRequested = false;  ///< Flag: GET_PID received
  bool stopRequested = false; ///< Flag: STOP received
  bool motorEnabled = true;   ///< Flag: ENABLE/DISABLE state

  /**
   * @brief Parse received command and update internal state accordingly.
   *
   * @param msg Null-terminated string message from USB
   */
  void parseMessage(const char *msg);

  // Recognized message headers
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
