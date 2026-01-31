/**
 * @file IMotorCommHandler.hpp
 * @brief Hardware-agnostic interface for motor communication handlers.
 *
 * Defines a generic communication handler API for motor control commands
 * and feedback. Works with any transport layer (USB, UART, Bluetooth, etc.)
 * via the IComm interface. Concrete implementations inherit from this
 * interface and provide protocol-specific functionality.
 */

#pragma once

#include "esp_err.h"

namespace DC_Motor_Controller_Firmware {

/**
 * @class IMotorCommHandler
 * @brief Abstract interface for handling motor-related communication commands.
 */
class IMotorCommHandler {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IMotorCommHandler() = default;

    /**
     * @brief Start the background task that continuously processes communication commands.
     */
    virtual void startTask() = 0;

    /**
     * @brief Poll for and process incoming messages.
     */
    virtual void process() = 0;

    /**
     * @brief Send the current motor position in degrees.
     * @param degrees Position in degrees.
     */
    virtual void sendMotorState(float degrees) = 0;

    /**
     * @brief Send current PID parameters in response to a request.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @return esp_err_t ESP_OK if successful.
     */
    virtual esp_err_t sendPIDParams(float kp, float ki, float kd) = 0;

    /**
     * @brief Send a notification that the motor has reached its target.
     */
    virtual void notifyMotorPositionReached() = 0;

    /**
     * @brief Clear the current position target and its flag.
     */
    virtual void clearTarget() = 0;

    /**
     * @brief Clear the PID request flag.
     */
    virtual void clearPIDRequest() = 0;

    /**
     * @brief Return the most recently received target angle.
     * @return Target angle in degrees.
     */
    virtual float getTargetDegrees() = 0;

    /**
     * @brief Retrieve the most recently received PID parameters.
     * @param kp Output: proportional gain.
     * @param ki Output: integral gain.
     * @param kd Output: derivative gain.
     */
    virtual void getPIDParams(float& kp, float& ki, float& kd) = 0;

    /**
     * @brief Indicate if a new target angle was received.
     * @return true if new target is available, false otherwise.
     */
    virtual bool isNewTargetReceived() const = 0;

    /**
     * @brief Indicate if new PID parameters were received.
     * @return true if updated PID is available, false otherwise.
     */
    virtual bool isNewPIDReceived() const = 0;

    /**
     * @brief Indicate if the host requested current PID parameters.
     * @return true if PID request was received, false otherwise.
     */
    virtual bool wasPIDRequested() const = 0;

    /**
     * @brief Indicate if the STOP command was received.
     * @return true if STOP requested, false otherwise.
     */
    virtual bool isStopRequested() const = 0;

    /**
     * @brief Indicate if the motor is currently enabled.
     * @return true if enabled, false if disabled.
     */
    virtual bool isMotorEnabled() const = 0;

    /**
     * @brief Indicate if the communication interface is open.
     * @return true if open, false otherwise.
     */
    virtual bool isUSBOpen() const = 0;

    /**
     * @brief Clear the stop flag.
     * @param stopRequested Optionally set to true to reassert stop state.
     */
    virtual void clearStopFlag(bool stopRequested = false) = 0;
};

}  // namespace DC_Motor_Controller_Firmware
