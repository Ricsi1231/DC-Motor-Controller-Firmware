/**
 * @file Encoder.hpp
 * @brief Quadrature encoder handling class using PCNT and ESP timer on ESP32.
 * 
 * Provides position (ticks, degrees) and speed (RPM) feedback for DC motor control.
 */

#pragma once

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

namespace DC_Motor_Controller_Firmware {
namespace Encoder {

/**
 * @enum motorDirection
 * @brief Enum representing current direction of motor rotation.
 */
enum class motorDirection : bool {
  LEFT = true,  /**< Counter-clockwise direction */
  RIGHT = false /**< Clockwise direction */
};

/**
 * @class Encoder
 * @brief Handles rotary encoder reading using ESP32 PCNT + timer.
 */
class Encoder {
public:
  /**
   * @brief Constructor for Encoder class.
   * 
   * @param pinA GPIO pin connected to encoder channel A
   * @param pinB GPIO pin connected to encoder channel B
   * @param unitConfig PCNT unit configuration structure
   * @param ppr Pulses per revolution of the encoder
   */
  Encoder(gpio_num_t pinA, gpio_num_t pinB, pcnt_unit_config_t unitConfig, uint16_t ppr);

  /**
   * @brief Destructor.
   */
  ~Encoder();

  /**
   * @brief Initialize encoder: GPIO, PCNT, timer, and filters.
   * 
   * @return esp_err_t ESP_OK if successful
   */
  esp_err_t init();

  /**
   * @brief Reset encoder position to zero.
   * 
   * @return esp_err_t ESP_OK if successful
   */
  esp_err_t resetPositon();

  /**
   * @brief Get current position in encoder ticks.
   * 
   * @return int32_t Position in raw PCNT ticks
   */
  int32_t getPositionTicks() const;

  /**
   * @brief Get current position in degrees.
   * 
   * @return float Angle in degrees
   */
  float getPositionInDegrees() const;

  /**
   * @brief Get motor speed in RPM (revolutions per minute).
   * 
   * @return int32_t Speed in RPM
   */
  int32_t getPositionInRPM() const;

  /**
   * @brief Get current motor direction.
   * 
   * @return motorDirection LEFT or RIGHT
   */
  motorDirection getMotorDriection() const;

private:
  /**
   * @brief Initialize pulse counter unit.
   */
  esp_err_t initPcnt();

  /**
   * @brief Initialize PCNT glitch filter.
   */
  esp_err_t initPcntFilter();

  /**
   * @brief Initialize PCNT input GPIOs.
   */
  esp_err_t initPcntIo();

  /**
   * @brief Initialize ESP timer for RPM calculation.
   */
  esp_err_t initTimer();

  /**
   * @brief Setup PCNT watch point (overflow/underflow, etc.).
   */
  esp_err_t initWatchPoint();

  /**
   * @brief Enable PCNT unit after setup.
   */
  esp_err_t enablePcUnit();

  /**
   * @brief Timer callback to periodically calculate RPM.
   * 
   * @param arg Pointer to class instance
   */
  static void timerCallback(void *arg);

  gpio_num_t pinA;      ///< Encoder channel A pin
  gpio_num_t pinB;      ///< Encoder channel B pin
  uint16_t ppr;         ///< Pulses per revolution

  pcnt_unit_handle_t pcntUnit;             ///< PCNT unit handle
  pcnt_unit_config_t unitConfig;           ///< Initial unit configuration

  pcnt_channel_handle_t chanA;             ///< PCNT channel for A signal
  pcnt_channel_handle_t chanB;             ///< PCNT channel for B signal

  esp_timer_handle_t timer;                ///< Timer handle for RPM calc

  int lastCount;       ///< Last encoder tick count
  float rpm;           ///< Calculated RPM

  const char *TAG = "ENCODER"; ///< Logging tag
};

} // namespace Encoder
} // namespace DC_Motor_Controller_Firmware
