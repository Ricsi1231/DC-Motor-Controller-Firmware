/**
 * @file MotorCommHandler.hpp
 * @brief Communication handler for motor control commands via USB.
 *
 * Handles incoming serial commands (target position, PID params,
 * enable/disable, etc.) and sends feedback (motor reached, current position,
 * etc.) using the USB interface. Optionally integrates application-level
 * logic when a MotorController and Encoder are provided.
 */

#pragma once

#include "USB.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace DC_Motor_Controller_Firmware {
namespace Control {
class MotorController;
}
namespace Encoder {
class Encoder;
}

namespace Communication {

/**
 * @class MotorCommHandler
 * @brief Parses and manages motor-related USB communication commands.
 *
 * When constructed with optional MotorController and Encoder pointers,
 * also handles closed-loop application logic (target setting, PID tuning,
 * motion-done notification).
 */
class MotorCommHandler {
  public:
    /**
     * @brief Constructor.
     * @param usbRef Reference to the initialized USB communication interface.
     * @param motor Optional pointer to the motor controller for application logic.
     * @param enc Optional pointer to the encoder for position feedback.
     */
    explicit MotorCommHandler(USB::USB& usbRef, Control::MotorController* motorPtr = nullptr, Encoder::Encoder* encoderPtr = nullptr);

    /**
     * @brief Destructor.
     *
     * Deletes the FreeRTOS task created by startTask(), if running.
     */
    ~MotorCommHandler();

    /**
     * @brief Starts the FreeRTOS task that continuously processes USB commands.
     *
     * Creates a background task (`MotorCommTask`) using `xTaskCreate()` which
     * calls `process()`. Does nothing if the task is already running.
     */
    void startTask();

    /**
     * @brief Poll for and process incoming USB messages.
     */
    void process();

    /**
     * @brief Sends the current motor position in degrees over USB.
     * @param degrees Position in degrees.
     */
    void sendMotorState(float degrees);

    /**
     * @brief Sends current PID parameters in response to GET_PID.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @return esp_err_t ESP_OK if successful.
     */
    esp_err_t sendPIDParams(float kp, float ki, float kd);

    /**
     * @brief Sends a notification that the motor has reached its target.
     */
    void notifyMotorPositionReached();

    /**
     * @brief Clears the current position target and its flag.
     */
    void clearTarget();

    /**
     * @brief Clears the PID request flag.
     */
    void clearPIDRequest();

    /**
     * @brief Returns the most recently received target angle.
     * @return float Target angle in degrees.
     */
    float getTargetDegrees();

    /**
     * @brief Retrieves the most recently received PID parameters.
     * @param kp Output: proportional gain.
     * @param ki Output: integral gain.
     * @param kd Output: derivative gain.
     */
    void getPIDParams(float& kp, float& ki, float& kd);

    /**
     * @brief Indicates if a new target angle was received.
     * @return true if new target is available, false otherwise.
     */
    bool isNewTargetReceived() const;

    /**
     * @brief Indicates if new PID parameters were received.
     * @return true if updated PID is available, false otherwise.
     */
    bool isNewPIDReceived() const;

    /**
     * @brief Indicates if the host requested current PID parameters.
     * @return true if GET_PID was received, false otherwise.
     */
    bool wasPIDRequested() const;

    /**
     * @brief Indicates if the STOP command was received.
     * @return true if STOP requested, false otherwise.
     */
    bool isStopRequested() const;

    /**
     * @brief Indicates if the motor is currently enabled.
     * @return true if enabled, false if disabled.
     */
    bool isMotorEnabled() const;

    /**
     * @brief Indicates if the USB serial interface is open.
     * @return true if open, false otherwise.
     */
    bool isUSBOpen() const;

    /**
     * @brief Clears the stop flag.
     * @param stopRequested Optionally set to true to reassert stop state.
     */
    void clearStopFlag(bool stopRequested = false);

  private:
    USB::USB& usb;                      ///< USB interface reference
    TaskHandle_t taskHandle = nullptr;  ///< Handle for the FreeRTOS task
    mutable portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

    float targetDegrees = 0.0f;  ///< Last received target angle
    bool newTarget = false;      ///< Flag: new target was received

    float pidKp = 0.0f;   ///< Last received PID: kp
    float pidKi = 0.0f;   ///< Last received PID: ki
    float pidKd = 0.0f;   ///< Last received PID: kd
    bool newPID = false;  ///< Flag: new PID values received

    bool pidRequested = false;   ///< Flag: GET_PID command received
    bool stopRequested = false;  ///< Flag: STOP command received
    bool motorEnabled = true;    ///< Flag: motor is enabled or disabled

    Control::MotorController* motor = nullptr;  ///< Optional motor controller for application logic
    Encoder::Encoder* encoder = nullptr;        ///< Optional encoder for position feedback

    float logicKp = 0.0f;        ///< Cached PID kp for application logic
    float logicKi = 0.0f;        ///< Cached PID ki for application logic
    float logicKd = 0.0f;        ///< Cached PID kd for application logic
    float currentPos = 0.0f;     ///< Current encoder position in degrees
    float offset = 0.0f;         ///< Target offset received from USB
    float targetDegree = 0.0f;   ///< Computed absolute target in degrees
    bool settled = false;         ///< Flag: motor has settled at target
    bool getPIDValuesFirstTime = true;  ///< Flag: initial PID value read pending

    static constexpr float settledToleranceDeg = 1.0f;  ///< Position tolerance for settled detection

    /**
     * @brief Parses incoming USB message strings and updates internal state.
     * @param msg Null-terminated message string.
     */
    void parseMessage(const char* msg);

    /**
     * @brief Processes application logic (target, PID, settle detection).
     *
     * Only runs when motor and encoder are non-null.
     */
    void processApplicationLogic();

    /**
     * @brief Static FreeRTOS task wrapper for the communication loop.
     * @param param Pointer to MotorCommHandler instance.
     */
    static void commTaskWrapper(void* param);

    static constexpr const char* MSG_SET_DEG = "SET_DEG:";
    static constexpr const char* MSG_SET_PID = "SET_PID:";
    static constexpr const char* MSG_GET_PID = "GET_PID";
    static constexpr const char* MSG_PID_REPLY = "PID:";
    static constexpr const char* MSG_MOTOR_POS = "MOTOR_POS:";
    static constexpr const char* MSG_REACHED = "MOTOR_REACHED";
    static constexpr const char* MSG_STOP = "STOP";
    static constexpr const char* MSG_ENABLE = "ENABLE";
    static constexpr const char* MSG_DISABLE = "DISABLE";
    static constexpr const char* MSG_RESET = "RESET_ALL";
    static constexpr const char* MSG_GET_STATE = "GET_STATE";
};

}  // namespace Communication
}  // namespace DC_Motor_Controller_Firmware
