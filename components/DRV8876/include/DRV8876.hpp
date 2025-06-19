/**
 * @file DRV8876.hpp
 * @brief DRV8876 Motor Driver Control Class using PWM and GPIO.
 *
 * Provides high-level interface for controlling a DC motor using the DRV8876
 * H-bridge driver via ESP32 GPIO and LEDC PWM peripherals.
 */

#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"

namespace DC_Motor_Controller_Firmware {
namespace DRV8876 {

/**
 * @enum Direction
 * @brief Enumeration for motor rotation direction.
 */
enum class Direction : bool {
  LEFT = false, /**< Motor turns left / counterclockwise */
  RIGHT = true  /**< Motor turns right / clockwise */
};

/**
 * @class DRV8876
 * @brief Class to control a DRV8876 H-bridge DC motor driver via ESP32.
 */
class DRV8876 {
public:
  /**
   * @brief Constructor for DRV8876 class.
   *
   * @param phPin GPIO for PH (direction control)
   * @param enPin GPIO for EN (PWM enable)
   * @param nFault GPIO for nFAULT signal (active low fault flag)
   * @param pwmChannel LEDC channel used for PWM
   */
  DRV8876(gpio_num_t phPin, gpio_num_t enPin, gpio_num_t nFault,
          ledc_channel_t pwmChannel);

  /**
   * @brief Destructor.
   */
  ~DRV8876();

  /**
   * @brief Initialize motor driver: configure GPIO, PWM, and fault interrupt.
   *
   * @return esp_err_t ESP_OK if success, else error code.
   */
  esp_err_t init();

  /**
   * @brief Set motor rotation direction.
   *
   * @param direction Desired motor direction.
   */
  void setDirection(Direction direction);

  /**
   * @brief Reverse current motor direction.
   */
  void reverseDirection();

  /**
   * @brief Set motor speed using PWM.
   *
   * @param speed Speed (0-100% duty).
   * @return esp_err_t ESP_OK if success, else error.
   */
  esp_err_t setSpeed(uint8_t speed);

  /**
   * @brief Immediately stop motor.
   */
  void stop();

  /**
   * @brief Get last set motor speed.
   *
   * @return uint8_t Motor speed percentage (0-100).
   */
  uint8_t getMotorSpeed() const;

  /**
   * @brief Get current motor direction.
   *
   * @return Direction Current direction.
   */
  Direction getMotorDirection() const;

  /**
   * @brief Check if motor is currently running.
   *
   * @return true If speed > 0.
   * @return false If stopped.
   */
  bool motorIsRunning() const;

  /**
   * @brief Check if fault is triggered (nFAULT = LOW).
   *
   * @return true If fault occurred.
   * @return false No fault.
   */
  bool isFaultTriggered() const;

  /**
   * @brief Clear internal fault flag (after user handles fault).
   */
  void clearFaultFlag();

  /**
   * @brief Set PWM frequency and resolution.
   *
   * @param resolution PWM resolution (e.g., LEDC_TIMER_10_BIT)
   * @param frequency PWM frequency in Hz (default 20kHz).
   * @return esp_err_t ESP_OK if success.
   */
  esp_err_t setPwmValue(ledc_timer_bit_t resolution,
                        uint32_t frequency = 20000);

private:
  /**
   * @brief Set PWM duty cycle based on speed (0–100%).
   *
   * @param pwmChannel LEDC channel
   * @param duty Duty in percentage
   * @return esp_err_t ESP_OK if success.
   */
  esp_err_t setPwmDuty(ledc_channel_t pwmChannel, uint8_t duty);

  /**
   * @brief ISR for handling fault interrupt.
   *
   * @param arg Pointer to DRV8876 instance.
   */
  static IRAM_ATTR void faultISR(void *arg);

  gpio_num_t phPin;          ///< Direction pin
  gpio_num_t enPin;          ///< Enable (PWM) pin
  gpio_num_t nFault;         ///< Fault detect pin
  ledc_channel_t pwmChannel; ///< PWM channel used

  ledc_timer_bit_t resolution = LEDC_TIMER_10_BIT; ///< Default resolution
  uint32_t frequency = 20000;                      ///< Default frequency

  Direction motorDirection; ///< Current motor direction
  uint8_t motorSpeed;       ///< Current speed (0–100)

  volatile bool faultTriggered = false; ///< Fault flag

  const uint8_t minMotorSpeed = 0;   ///< Min motor speed
  const uint8_t maxMotorSpeed = 100; ///< Max motor speed

  const uint32_t MIN_PWM_FREQ = 1000;  ///< Minimum PWM frequency allowed
  const uint32_t MAX_PWM_FREQ = 30000; ///< Maximum PWM frequency allowed

  const uint8_t motorStop = 0; ///< Constant for stop signal

  bool isInitialized = false; ///< Tracks initialization status

  const char *TAG = "DRV8876"; ///< Logging tag
};

} // namespace DRV8876
} // namespace DC_Motor_Controller_Firmware
