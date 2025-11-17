#include "DRV8876.hpp"

namespace DC_Motor_Controller_Firmware {
namespace DRV8876 {

DRV8876::DRV8876(const DRV8876Config& config) : config(config) {
    if (ledcMutex == nullptr) {
        ledcMutex = xSemaphoreCreateMutex();
        if (ledcMutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create LEDC mutex");
        }
    }
}

DRV8876::~DRV8876() { ESP_LOGI(TAG, "Destructor called"); }

DRV8876::DRV8876(DRV8876&& other) noexcept : config(other.config) {}

DRV8876& DRV8876::operator=(DRV8876&& other) noexcept {
    if (this != &other) {
        config = other.config;
    }
    return *this;
}

esp_err_t DRV8876::init() {
    ESP_LOGI(TAG, "Init start (PH=%d, EN=%d, nFAULT=%d, nSLEEP=%d, PWM_CH=%d)", config.phPin, config.enPin, config.nFault, config.nSleep, config.pwmChannel);
    esp_err_t status = ESP_OK;

    if (config.minFrequency == 0) {
        config.minFrequency = 100;
    }

    if (config.maxFrequency == 0 || config.maxFrequency < config.minFrequency) {
        config.maxFrequency = 100000;
    }

    if (config.frequency < config.minFrequency) {
        config.frequency = config.minFrequency;
    }

    if (config.frequency > config.maxFrequency) {
        config.frequency = config.maxFrequency;
    }

    if (config.rampStepPercent == 0) {
        config.rampStepPercent = 5;
    }

    if (config.rampStepDelayMs == 0) {
        config.rampStepDelayMs = 5;
    }

    if (config.minEffectivePwmPercent == 0) {
        config.minEffectivePwmPercent = 3;
    }

    gpio_config_t ioConf = {};
    ioConf.intr_type = GPIO_INTR_DISABLE;
    ioConf.mode = GPIO_MODE_OUTPUT;
    ioConf.pin_bit_mask = (1ULL << config.phPin);
    status = gpio_config(&ioConf);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(PH) failed: %s", esp_err_to_name(status));
        return status;
    }

    ioConf.intr_type = GPIO_INTR_DISABLE;
    ioConf.mode = GPIO_MODE_OUTPUT;
    ioConf.pin_bit_mask = (1ULL << config.nSleep);
    status = gpio_config(&ioConf);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(nSLEEP) failed: %s", esp_err_to_name(status));
        return status;
    }

    gpio_config_t faultConf = {};
    faultConf.intr_type = GPIO_INTR_NEGEDGE;
    faultConf.mode = GPIO_MODE_INPUT;
    faultConf.pin_bit_mask = (1ULL << config.nFault);
    faultConf.pull_up_en = GPIO_PULLUP_ENABLE;
    status = gpio_config(&faultConf);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(nFAULT) failed: %s", esp_err_to_name(status));
        return status;
    }

    status = gpio_install_isr_service(0);
    if (status == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "ISR service already installed");
        status = ESP_OK;
    }
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(status));
        return status;
    }

    status = gpio_isr_handler_add(config.nFault, faultISR, this);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(status));
        return status;
    }
    ESP_LOGI(TAG, "nFAULT ISR attached");

    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = config.resolution;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = config.frequency;
    timer.clk_cfg = LEDC_AUTO_CLK;
    status = ledc_timer_config(&timer);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(status));
        return status;
    }
    ESP_LOGI(TAG, "LEDC timer set (res=%d, freq=%" PRIu32 " Hz)", config.resolution, config.frequency);

    ledc_channel_config_t channelConf = {};
    channelConf.gpio_num = config.enPin;
    channelConf.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConf.channel = config.pwmChannel;
    channelConf.timer_sel = LEDC_TIMER_0;
    channelConf.duty = 0;
    channelConf.hpoint = 0;
    channelConf.flags.output_invert = false;

    status = ledc_channel_config(&channelConf);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(status));
        return status;
    }
    ESP_LOGI(TAG, "LEDC channel configured on GPIO %d", config.enPin);

    initialized = true;

    stop();

    gpio_set_level(config.nSleep, true);
    ESP_LOGI(TAG, "nSLEEP set high");
    vTaskDelay(pdMS_TO_TICKS(10));

    maxDuty = (1u << config.resolution) - 1u;

    ESP_LOGI(TAG, "Init done");
    return ESP_OK;
}

DRV8876Config DRV8876::getConfig() const { return this->config; };

void DRV8876::setDirection(Direction direction) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8878 class is not initialized");
        return;
    }

    motorDirection = direction;
    gpio_set_level(config.phPin, static_cast<bool>(motorDirection));
    ESP_LOGI(TAG, "Direction=%s", motorDirection == Direction::LEFT ? "LEFT" : "RIGHT");
}

void DRV8876::reverseDirection() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8878 class is not initialized");
        return;
    }

    motorDirection = (motorDirection == Direction::LEFT) ? Direction::RIGHT : Direction::LEFT;
    gpio_set_level(config.phPin, static_cast<bool>(motorDirection));
    ESP_LOGI(TAG, "Direction reversed -> %s", motorDirection == Direction::LEFT ? "LEFT" : "RIGHT");
}

esp_err_t DRV8876::setDirectionSafe(Direction newDirection) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    uint8_t originalSpeed = motorSpeed;

    esp_err_t rampDownResult = rampTo(0);
    if (rampDownResult != ESP_OK) {
        return rampDownResult;
    }

    setDirection(newDirection);

    if (originalSpeed > 0 && originalSpeed < config.minEffectivePwmPercent) {
        originalSpeed = config.minEffectivePwmPercent;
    }

    esp_err_t rampUpResult = rampTo(originalSpeed);
    if (rampUpResult != ESP_OK) {
        return rampUpResult;
    }

    return ESP_OK;
}

esp_err_t DRV8876::reverseDirectionSafe() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    Direction reversedDirection = (motorDirection == Direction::LEFT) ? Direction::RIGHT : Direction::LEFT;

    return setDirectionSafe(reversedDirection);
}

esp_err_t DRV8876::setSpeed(uint8_t speed) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8878 class is not initialized");
        return ESP_FAIL;
    }

    enableMotorStopedLog = true;

    if (speed < minMotorSpeed || speed > maxMotorSpeed) {
        ESP_LOGW(TAG, "Speed out of range (%u not in [%u..%u])", speed, minMotorSpeed, maxMotorSpeed);
        return ESP_ERR_INVALID_ARG;
    }

    if (speed > 0 && speed < config.minEffectivePwmPercent) {
        speed = config.minEffectivePwmPercent;
    }

    motorSpeed = speed;
    previousSpeedValue = motorSpeed;
    const uint32_t dutyTicks = percentToTicks(motorSpeed);

    esp_err_t setDutyResult = setPwmDutyTicks(config.pwmChannel, dutyTicks);
    if (setDutyResult != ESP_OK) {
        ESP_LOGE(TAG, "setPwmDutyTicks failed: %s", esp_err_to_name(setDutyResult));
        return setDutyResult;
    }

    const uint32_t maxDutyTicks = (1u << config.resolution) - 1u;

    if (previousSpeedValue != motorSpeed) {
        ESP_LOGI(TAG, "Speed=%u%% (duty=%" PRIu32 "/%" PRIu32 ", res=%u-bit)", motorSpeed, dutyTicks, maxDutyTicks, static_cast<unsigned>(config.resolution));
    }

    previousSpeedValue = motorSpeed;
    return ESP_OK;
}

esp_err_t DRV8876::setSpeed(uint8_t targetPercent, uint32_t rampTimeMs) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    enableMotorStopedLog = true;

    if (targetPercent > 100) {
        targetPercent = 100;
    }

    if (targetPercent > 0 && targetPercent < config.minEffectivePwmPercent) {
        targetPercent = config.minEffectivePwmPercent;
    }

    uint8_t currentSpeed = motorSpeed;
    if (currentSpeed == targetPercent) {
        return ESP_OK;
    }

    int delta = static_cast<int>(targetPercent) - static_cast<int>(currentSpeed);
    uint32_t steps = static_cast<uint32_t>(abs(delta));
    if (steps == 0) {
        return ESP_OK;
    }

    uint32_t delayPerStepMs = rampTimeMs / steps;
    if (delayPerStepMs == 0) {
        delayPerStepMs = 1;
    }

    int stepDirection = (delta > 0) ? 1 : -1;

    uint8_t nextSpeed = currentSpeed;
    while (nextSpeed != targetPercent) {
        nextSpeed = static_cast<uint8_t>(static_cast<int>(nextSpeed) + stepDirection);

        if (nextSpeed > 0 && nextSpeed < config.minEffectivePwmPercent) {
            nextSpeed = config.minEffectivePwmPercent;
        }

        esp_err_t stepResult = setSpeed(nextSpeed);
        if (stepResult != ESP_OK) {
            return stepResult;
        }

        vTaskDelay(pdMS_TO_TICKS(delayPerStepMs));
    }

    return ESP_OK;
}

esp_err_t DRV8876::stop() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8878 class is not initialized");
        return ESP_FAIL;
    }

    motorSpeed = 0;

    esp_err_t pwmStatus = setPwmDuty(config.pwmChannel, motorStop);
    if (pwmStatus != ESP_OK) {
        ESP_LOGE(TAG, "Stop -> setPwmDuty failed: %s", esp_err_to_name(pwmStatus));
        return ESP_FAIL;
    }

    if (enableMotorStopedLog) {
        ESP_LOGI(TAG, "Stopped");
        enableMotorStopedLog = false;
    }

    return ESP_OK;
}

esp_err_t DRV8876::coast() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    motorSpeed = motorStop;
    esp_err_t pwmStatus = setPwmDuty(config.pwmChannel, motorStop);
    if (pwmStatus != ESP_OK) {
        ESP_LOGE(TAG, "Coast -> setPwmDuty failed: %s", esp_err_to_name(pwmStatus));
        return pwmStatus;
    }

    gpio_set_level(config.enPin, motorStop);
    ESP_LOGI(TAG, "Motor set to COAST (outputs disabled, free spin)");
    return ESP_OK;
}

esp_err_t DRV8876::brake() {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    motorSpeed = motorStop;
    esp_err_t pwmStatus = setPwmDuty(config.pwmChannel, motorStop);
    if (pwmStatus != ESP_OK) {
        ESP_LOGE(TAG, "Brake -> setPwmDuty failed: %s", esp_err_to_name(pwmStatus));
        return pwmStatus;
    }

    gpio_set_level(config.phPin, motorStop);
    gpio_set_level(config.enPin, motorStop);
    ESP_LOGI(TAG, "Motor set to BRAKE (outputs shorted)");
    return ESP_OK;
}

uint8_t DRV8876::getMotorSpeed() const { return motorSpeed; }

Direction DRV8876::getMotorDirection() const { return motorDirection; }

bool DRV8876::motorIsRunning() const { return motorSpeed > 0; }

bool DRV8876::isFaultTriggered() const { return faultTriggered.load(std::memory_order_relaxed); }

void DRV8876::clearFaultFlag() { faultTriggered.store(false, std::memory_order_relaxed); }

bool DRV8876::getAndClearFault() { return faultTriggered.exchange(false, std::memory_order_acq_rel); }

esp_err_t DRV8876::setPwmValue(ledc_timer_bit_t resolution, uint32_t frequency) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8878 class is not initialized");
        return ESP_FAIL;
    }

    if (frequency < config.minFrequency || frequency > config.maxFrequency) {
        ESP_LOGW(TAG, "Frequency out of range %" PRIu32 " Hz (allowed %" PRIu32 "..%" PRIu32 ")", frequency, config.minFrequency, config.maxFrequency);
        return ESP_ERR_INVALID_ARG;
    }

    this->config.resolution = resolution;
    this->config.frequency = frequency;

    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = resolution;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = frequency;
    timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t timerConfigStatus = ledc_timer_config(&timer);
    if (timerConfigStatus != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config (setPwmValue) failed: %s", esp_err_to_name(timerConfigStatus));
        return ESP_FAIL;
    }

    maxDuty = (1u << config.resolution) - 1u;

    uint8_t pwmVal = (motorSpeed * 255) / 100;
    esp_err_t dutySetStatus = setPwmDuty(config.pwmChannel, pwmVal);
    if (dutySetStatus != ESP_OK) {
        ESP_LOGE(TAG, "setPwmDuty after timer change failed: %s", esp_err_to_name(dutySetStatus));
        return dutySetStatus;
    }

    ESP_LOGI(TAG, "PWM updated (res=%d, freq=%" PRIu32 " Hz, duty=%" PRIu32 "/255)", resolution, frequency, (uint32_t)pwmVal);

    return ESP_OK;
}

esp_err_t DRV8876::setPwmFrequency(uint32_t frequency) { return setPwmValue(config.resolution, frequency); }

void DRV8876::setPwmFrequencyRange(uint32_t minHz, uint32_t maxHz) {
    if (minHz == 0) {
        minHz = 1;
    }

    if (maxHz < minHz) {
        maxHz = minHz;
    }

    config.minFrequency = minHz;
    config.maxFrequency = maxHz;

    if (config.frequency < config.minFrequency) {
        config.frequency = config.minFrequency;
    }

    if (config.frequency > config.maxFrequency) {
        config.frequency = config.maxFrequency;
    }
}

uint32_t DRV8876::getPwmMinFrequency() const { return config.minFrequency; }

uint32_t DRV8876::getPwmMaxFrequency() const { return config.maxFrequency; }

void DRV8876::setFaultCallback(const std::function<void()>& cb) { faultCallback = cb; }

void DRV8876::processFaultEvent() {
    bool hadFault = getAndClearFault();
    if (hadFault == false) {
        return;
    }

    if (faultCallback) {
        faultCallback();
    }
}

esp_err_t DRV8876::setPwmDuty(ledc_channel_t pwmChannel, uint8_t duty) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8878 class is not initialized");
        return ESP_FAIL;
    }

    if (!lockLedc(ledcMutexTimeoutMs)) {
        ESP_LOGE(TAG, "LEDC lock timeout (set duty)");
        return ESP_ERR_TIMEOUT;
    }

    uint32_t scaledDuty = (duty * ((1 << config.resolution) - 1)) / 255;
    esp_err_t setResult = ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, scaledDuty);
    if (setResult == ESP_OK) {
        setResult = ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
    }

    unlockLedc();

    if (setResult != ESP_OK) {
        ESP_LOGE(TAG, "PWM duty update failed: %s", esp_err_to_name(setResult));
        return setResult;
    }

    ESP_LOGD(TAG, "PWM duty set (scaled=%" PRIu32 ")", scaledDuty);
    return ESP_OK;
}

esp_err_t DRV8876::setPwmDutyTicks(ledc_channel_t channel, uint32_t dutyTicks) {
    const uint32_t maxDutyTicks = (1u << config.resolution) - 1u;
    if (dutyTicks > maxDutyTicks) {
        dutyTicks = maxDutyTicks;
    }

    esp_err_t setResult = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, dutyTicks);
    if (setResult == ESP_OK) {
        setResult = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    }

    if (setResult != ESP_OK) {
        ESP_LOGE(TAG, "PWM ticks update failed: %s", esp_err_to_name(setResult));
        return setResult;
    }

    return ESP_OK;
}

uint32_t DRV8876::percentToTicks(uint8_t percent) {
    if (percent > 100) {
        percent = 100;
    }
    const uint32_t maxDutyTicks = (1u << config.resolution) - 1u;
    return (maxDutyTicks * static_cast<uint32_t>(percent) + 50u) / 100u;
}

esp_err_t DRV8876::rampTo(uint8_t targetPercent) {
    if (initialized == false) {
        ESP_LOGE(TAG, "DRV8876 class is not initialized");
        return ESP_FAIL;
    }

    if (targetPercent > 100) {
        targetPercent = 100;
    }

    if (targetPercent > 0 && targetPercent < config.minEffectivePwmPercent) {
        targetPercent = config.minEffectivePwmPercent;
    }

    uint8_t currentSpeed = motorSpeed;
    if (currentSpeed == targetPercent) {
        return ESP_OK;
    }

    if (currentSpeed < targetPercent) {
        while (currentSpeed < targetPercent) {
            uint8_t nextSpeed = currentSpeed + config.rampStepPercent;
            if (nextSpeed > targetPercent) {
                nextSpeed = targetPercent;
            }

            if (nextSpeed > 0 && nextSpeed < config.minEffectivePwmPercent) {
                nextSpeed = config.minEffectivePwmPercent;
            }

            esp_err_t speedResult = setSpeed(nextSpeed);
            if (speedResult != ESP_OK) {
                return speedResult;
            }

            currentSpeed = nextSpeed;
            vTaskDelay(pdMS_TO_TICKS(config.rampStepDelayMs));
        }
    } else {
        while (currentSpeed > targetPercent) {
            uint8_t nextSpeed = 0;
            if (currentSpeed > config.rampStepPercent) {
                nextSpeed = static_cast<uint8_t>(currentSpeed - config.rampStepPercent);
            } else {
                nextSpeed = 0;
            }

            if (targetPercent > 0) {
                if (nextSpeed > 0 && nextSpeed < config.minEffectivePwmPercent) {
                    nextSpeed = config.minEffectivePwmPercent;
                }
            }

            if (nextSpeed < targetPercent) {
                nextSpeed = targetPercent;
            }

            esp_err_t speedResult = setSpeed(nextSpeed);
            if (speedResult != ESP_OK) {
                return speedResult;
            }

            currentSpeed = nextSpeed;
            vTaskDelay(pdMS_TO_TICKS(config.rampStepDelayMs));
        }
    }

    return ESP_OK;
}

bool DRV8876::lockLedc(uint32_t timeoutMs) {
    if (ledcMutex == nullptr) {
        return false;
    }

    TickType_t ticks;
    if (timeoutMs == portMAX_DELAY) {
        ticks = portMAX_DELAY;
    } else {
        ticks = pdMS_TO_TICKS(timeoutMs);
    }

    BaseType_t takeResult = xSemaphoreTake(ledcMutex, ticks);
    if (takeResult == pdTRUE) {
        return true;
    } else {
        return false;
    }
}

void DRV8876::unlockLedc() {
    if (ledcMutex != nullptr) {
        xSemaphoreGive(ledcMutex);
    }
}

void IRAM_ATTR DRV8876::faultISR(void* arg) {
    auto* self = static_cast<DRV8876*>(arg);
    self->faultTriggered.store(true, std::memory_order_relaxed);
}

}  // namespace DRV8876
}  // namespace DC_Motor_Controller_Firmware
