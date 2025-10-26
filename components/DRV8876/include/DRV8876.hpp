/**
 * @file DRV8876.hpp
 * @brief DRV8876 Motor Driver Control Class using PWM and GPIO.
 *
 * Provides high-level interface for controlling a DC motor using the DRV8876
 * H-bridge driver via ESP32 GPIO and LEDC PWM peripherals.
 */

#pragma once

#include <inttypes.h>
#include <atomic>
#include <functional>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

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

struct DRV8876Config {
    gpio_num_t phPin;           ///< Direction pin
    gpio_num_t enPin;           ///< Enable (PWM) pin
    gpio_num_t nFault;          ///< Fault detect pin
    gpio_num_t nSleep;          ///< Driver enable pin
    ledc_channel_t pwmChannel;  ///< PWM channel used

    ledc_timer_bit_t resolution;     ///< Default PWM resolution
    uint32_t frequency;              ///< Default PWM frequency in Hz
    uint32_t minFrequency;           ///< Minimum allowed PWM frequency in Hz (default 100 Hz)
    uint32_t maxFrequency;           ///< Maximum allowed PWM frequency in Hz (default 100 kHz)
    uint8_t rampStepPercent;         ///< step size in % for safe direction changes
    uint32_t rampStepDelayMs;        ///< delay in ms between ramp steps
    uint8_t minEffectivePwmPercent;  ///< Minimum effective PWM duty (%) to overcome deadband
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
     * @param nSleep GPIO for driver sleep mode (need active high to enable the driver)
     * @param pwmChannel LEDC channel used for PWM
     */
    DRV8876(const DRV8876Config& config);

    /**
     * @brief Destructor.
     */
    ~DRV8876();

    /**
     * @brief Deleted copy constructor to prevent copying of DRV8876 instance.
     */
    DRV8876(const DRV8876&) = delete;

    /**
     * @brief Deleted copy assignment operator to prevent copying of DRV8876 instance.
     */
    DRV8876& operator=(const DRV8876&) = delete;

    /**
     * @brief Move constructor for transferring ownership of a DRV8876 instance.
     * @param other Source instance to move from.
     */
    DRV8876(DRV8876&& other) noexcept;

    /**
     * @brief Move assignment operator for transferring ownership of a DRV8876 instance.
     * @param other Source instance to move from.
     * @return Reference to the assigned DRV8876 instance.
     */
    DRV8876& operator=(DRV8876&& other) noexcept;

    /**
     * @brief Initialize motor driver: configure GPIO, PWM, and fault interrupt.
     *
     * @return esp_err_t ESP_OK if success, else error code.
     */
    esp_err_t init();

    /**
     * @brief Retrieve the current driver configuration.
     *
     * Provides a copy of the configuration structure that was
     * used to initialize this DRV8876 instance.
     *
     * @return DRV8876Config structure with pin assignments,
     *         timer/channel setup, and other parameters.
     */
    DRV8876Config getConfig() const;

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
     * @brief Safely set motor rotation direction.
     *
     * Ensures that direction changes are applied in a controlled manner,
     * preventing potential glitches or unsafe transitions on the PH pin.
     *
     * @param direction Desired motor direction.
     * @return ESP_OK on success, or an error code from lower-level drivers.
     */
    esp_err_t setDirectionSafe(Direction direction);

    /**
     * @brief Safely reverse the current motor rotation direction.
     *
     * Reads the current state and applies the opposite direction
     * using safe transition logic.
     *
     * @return ESP_OK on success, or an error code from lower-level drivers.
     */
    esp_err_t reverseDirectionSafe();

    /**
     * @brief Set motor speed using PWM.
     *
     * @param speed Speed (0-100% duty).
     * @return esp_err_t ESP_OK if success, else error.
     */
    esp_err_t setSpeed(uint8_t speed);

    /**
     * @brief Gradually change motor speed to a target value over a given time.
     *
     * This function ramps from the current speed to the target speed in small
     * increments so that the full transition takes approximately rampTimeMs.
     *
     * @param targetPercent Desired motor speed in percent [0–100].
     * @param rampTimeMs    Total time for the ramp (milliseconds).
     * @return esp_err_t ESP_OK if successful, or error code if PWM update fails.
     */
    esp_err_t setSpeed(uint8_t targetPercent, uint32_t rampTimeMs);

    /**
     * @brief Immediately stop motor.
     */
    esp_err_t stop();

    /**
     * @brief Coast the motor (disable outputs, motor spins freely).
     *
     * Unlike stop(), this does not actively drive the motor with 0% duty.
     * The motor is left floating, resulting in no torque and free run-down.
     *
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t coast();

    /**
     * @brief Brake the motor (short both motor terminals).
     *
     * Forces both outputs low, providing an electrical brake.
     * The motor stops faster than coast() but generates heating in windings.
     *
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t brake();

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
     * @brief Check and clear the fault status.
     *
     * Reads the current fault flag, then clears it if set.
     * This allows the application to acknowledge and reset
     * fault conditions in the driver.
     *
     * @return true if a fault was detected before clearing,
     *         false if no fault was present.
     */
    bool getAndClearFault();

    /**
     * @brief Set PWM frequency and resolution.
     *
     * @param resolution PWM resolution (e.g., LEDC_TIMER_10_BIT)
     * @param frequency PWM frequency in Hz (default 20kHz).
     * @return esp_err_t ESP_OK if success.
     */
    esp_err_t setPwmValue(ledc_timer_bit_t resolution, uint32_t frequency = 20000);

    /**
     * @brief Set PWM frequency while keeping current resolution.
     *
     * @param frequency Desired PWM frequency in Hz.
     * @return esp_err_t ESP_OK if success, ESP_ERR_INVALID_ARG if out of allowed range.
     */
    esp_err_t setPwmFrequency(uint32_t frequency);

    /**
     * @brief Configure the allowed frequency range.
     *
     * Updates internal min/max frequency values and clamps current frequency into the new range.
     *
     * @param minHz Minimum allowed frequency in Hz.
     * @param maxHz Maximum allowed frequency in Hz.
     */
    void setPwmFrequencyRange(uint32_t minHz, uint32_t maxHz);

    /**
     * @brief Get currently configured minimum PWM frequency.
     *
     * @return Minimum frequency in Hz.
     */
    uint32_t getPwmMinFrequency() const;

    /**
     * @brief Get currently configured maximum PWM frequency.
     *
     * @return Maximum frequency in Hz.
     */
    uint32_t getPwmMaxFrequency() const;

    /**
     * @brief Register a user callback to be invoked on fault events.
     *
     * The callback will be executed when a fault condition is detected.
     * This allows the application to respond asynchronously (e.g., stop motor,
     * log error, trigger recovery).
     *
     * @param cb Callback function to invoke on fault. Pass nullptr to disable.
     */
    void setFaultCallback(const std::function<void()>& cb);

    /**
     * @brief Process a pending fault event.
     *
     * This should be called from the driver's internal logic when a fault
     * interrupt or detection occurs. If a fault callback is registered,
     * it will be invoked here.
     */
    void processFaultEvent();

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
     * @brief Set PWM duty cycle directly in timer ticks.
     *
     * Writes the specified number of ticks to the LEDC channel’s
     * duty register. Use this when precise tick-level control is required.
     *
     * @param pwmChannel LEDC channel to configure.
     * @param ticks Number of ticks corresponding to desired duty cycle.
     * @return esp_err_t ESP_OK if successful, or error code otherwise.
     */
    esp_err_t setPwmDutyTicks(ledc_channel_t pwmChannel, uint32_t ticks);

    /**
     * @brief Convert duty percentage (0–100%) into timer ticks.
     *
     * Calculates the equivalent tick value for a given duty percentage
     * based on the current LEDC timer configuration.
     *
     * @param percent Duty cycle in percentage (0–100).
     * @return uint32_t Number of ticks corresponding to the percentage.
     */
    uint32_t percentToTicks(uint8_t percent);

    /**
     * @brief Gradually change motor speed to a target percentage.
     *
     * This function ramps the current motor speed up or down toward the
     * specified target, using step size and delay values configured in
     * DRV8876Config (rampStepPercent and rampStepDelayMs).
     *
     * - If the target speed is higher than the current speed, the motor will
     *   accelerate in increments until it reaches the target.
     * - If the target speed is lower, the motor will decelerate gradually down
     *   to the target.
     *
     * @param targetPercent Desired motor speed in percent [0–100].
     * @return esp_err_t ESP_OK if successful, or error code if setting PWM fails.
     */
    esp_err_t rampTo(uint8_t targetPercent);

    /**
     * @brief Acquire exclusive access to the LEDC peripheral.
     *
     * Attempts to lock the internal LEDC mutex within the specified timeout.
     * Ensures that only one task can configure or update LEDC resources
     * related to this driver at a time.
     *
     * @param timeoutMs Timeout in milliseconds to wait for the lock.
     *                  Use portMAX_DELAY for indefinite wait.
     * @return true if the lock was successfully acquired, false on timeout or error.
     */
    bool lockLedc(uint32_t timeoutMs);

    /**
     * @brief Release previously acquired LEDC lock.
     *
     * Should be called after finishing LEDC operations
     * to allow other tasks to access the peripheral.
     */
    void unlockLedc();

    /**
     * @brief ISR for handling fault interrupt.
     *
     * @param arg Pointer to DRV8876 instance.
     */
    static IRAM_ATTR void faultISR(void* arg);

    DRV8876Config config;  ///< Config for DRV8876 motor driver

    Direction motorDirection;  ///< Current motor direction
    uint8_t motorSpeed;        ///< Current speed (0–100)

    std::atomic_bool faultTriggered{false};  ///< Fault flag

    const uint8_t minMotorSpeed = 0;    ///< Min motor speed
    const uint8_t maxMotorSpeed = 100;  ///< Max motor speed

    const uint32_t MIN_PWM_FREQ = 1000;   ///< Minimum PWM frequency allowed
    const uint32_t MAX_PWM_FREQ = 30000;  ///< Maximum PWM frequency allowed

    uint32_t maxDuty = 0;  ///< Maximum duty cycle value in timer ticks

    const uint8_t motorStop = 0;  ///< Constant for stop signal

    uint8_t previousSpeedValue = 0;

    bool initialized = false;  ///< DRV class initialized flag

    bool enableMotorStopedLog = true;

    static constexpr char TAG[] = "DRV8876";  ///< Logging tag

    SemaphoreHandle_t ledcMutex = nullptr;  ///< Mutex for synchronizing LEDC access
    uint32_t ledcMutexTimeoutMs = 50;       ///< Default timeout (ms) when waiting for LEDC mutex
    std::function<void()> faultCallback;    ///< User-defined callback invoked on fault events
};

}  // namespace DRV8876
}  // namespace DC_Motor_Controller_Firmware
