/**
 * @file Actuator.hpp
 * @brief Safe hardware interface for motor driver commands.
 */

#pragma once

#include "DRV8876.hpp"
#include <cstdint>

namespace DC_Motor_Controller_Firmware::Control {

/**
 * @struct ActuatorConfig
 * @brief Configuration for actuator behavior.
 */
struct ActuatorConfig {
    float slewRatePctPerSec = 300.0f;  ///< Slew rate limit (%/s)
    float maxSpeed = 100.0f;           ///< Maximum output speed (%)
};

/**
 * @class Actuator
 * @brief Translates signed percentage commands to motor driver operations.
 */
class Actuator {
  public:
    /**
     * @brief Construct actuator with motor driver reference.
     * @param driver Reference to DRV8876 motor driver
     * @param cfg Actuator configuration
     */
    explicit Actuator(DRV8876::DRV8876& driver, const ActuatorConfig& cfg = ActuatorConfig());

    /**
     * @brief Drive motor with signed percentage command.
     * @param commandPct Signed command [-100, 100]; positive = RIGHT, negative = LEFT
     * @param nowUs Current timestamp for slew rate calculation
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t drive(float commandPct, uint64_t nowUs);

    /**
     * @brief Immediately stop the motor (zero output).
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t stop();

    /**
     * @brief Emergency stop - bypasses slew limiting.
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t emergencyStop();

    /**
     * @brief Coast the motor (disable outputs).
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t coast();

    /**
     * @brief Brake the motor (short windings).
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t brake();

    /**
     * @brief Update actuator configuration.
     * @param cfg New configuration
     */
    void setConfig(const ActuatorConfig& cfg);

    /**
     * @brief Get current configuration.
     * @return Current configuration
     */
    ActuatorConfig getConfig() const noexcept;

    /**
     * @brief Get current output percentage (after slew limiting).
     * @return Current output percentage
     */
    float getCurrentOutput() const noexcept;

    /**
     * @brief Get current motor direction.
     * @return Current direction
     */
    DRV8876::Direction getDirection() const noexcept;

    /**
     * @brief Check if motor is running.
     * @return True if speed > 0
     */
    bool isRunning() const noexcept;

    /**
     * @brief Check for driver fault condition.
     * @return True if fault detected
     */
    bool isFaultDetected() const noexcept;

    /**
     * @brief Clear driver fault flag.
     */
    void clearFault();

    /**
     * @brief Reset slew state (e.g., on motion start).
     */
    void resetSlewState();

    /**
     * @brief Set slew rate limit.
     * @param pctPerSec Slew rate in %/s
     */
    void setSlewRate(float pctPerSec);

    /**
     * @brief Get slew rate limit.
     * @return Slew rate in %/s
     */
    float getSlewRate() const noexcept;

    /**
     * @brief Set maximum speed.
     * @param maxSpeed Maximum speed (%)
     */
    void setMaxSpeed(float maxSpeed);

  private:
    /**
     * @brief Apply slew rate limiting.
     * @param targetPct Target output percentage
     * @param nowUs Current timestamp
     * @return Slew-limited output percentage
     */
    float applySlewLimit(float targetPct, uint64_t nowUs);

    DRV8876::DRV8876& driver;        ///< Motor driver reference
    ActuatorConfig config;           ///< Configuration
    float currentOutputPct = 0.0f;   ///< Current slew-limited output
    uint64_t lastSlewUs = 0;         ///< Last slew update timestamp
};

}  // namespace DC_Motor_Controller_Firmware::Control
