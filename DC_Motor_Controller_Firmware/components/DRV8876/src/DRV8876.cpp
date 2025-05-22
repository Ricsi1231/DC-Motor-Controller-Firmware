#include "DRV8876.hpp"
#include "esp_log.h"

namespace DC_Motor_Controller_Firmware {
    namespace DRV8876 {
        DRV8876::DRV8876(gpio_num_t phPin, gpio_num_t enPin, gpio_num_t nFault, ledc_channel_t pwmChannel) {
            this->phPin = phPin;
            this->enPin = enPin;
            this->nFault = nFault;
            this->pwmChannel = pwmChannel;
        }

        esp_err_t DRV8876::init() {
            esp_err_t errorEsp = ESP_OK;

            gpio_config_t ioConf = {};
            ioConf.intr_type = GPIO_INTR_DISABLE;
            ioConf.mode = GPIO_MODE_OUTPUT;
            ioConf.pin_bit_mask = (1ULL << phPin);
            
            errorEsp = gpio_config(&ioConf);

            if(errorEsp != ESP_OK) {
                ESP_LOGW(TAG, "DRV8876 Init - Faild init phPin GPIO");
                return ESP_FAIL;
            }

            gpio_config_t faultConf = {};
            faultConf.intr_type = GPIO_INTR_NEGEDGE;         
            faultConf.mode = GPIO_MODE_INPUT;
            faultConf.pin_bit_mask = (1ULL << nFault);
            faultConf.pull_up_en = GPIO_PULLUP_ENABLE;       
            
            errorEsp = gpio_config(&faultConf);

            if(errorEsp != ESP_OK) {
                ESP_LOGW(TAG, "DRV8876 Init - Faild init nFault GPIO");
                return ESP_FAIL;
            }

            errorEsp = gpio_install_isr_service(0);

            if(errorEsp != ESP_OK) {
                ESP_LOGW(TAG, "DRV8876 Init - Faild add ISR service 0");
                return ESP_FAIL;
            }

            errorEsp = gpio_isr_handler_add(nFault, faultISR, this);

            if(errorEsp != ESP_OK) {
                ESP_LOGW(TAG, "DRV8876 Init - Faild init ISR service nFault GPIO");
                return ESP_FAIL;
            }

            ledc_timer_config_t timer = {
                .speed_mode       = LEDC_LOW_SPEED_MODE,
                .duty_resolution  = resolution,
                .timer_num        = LEDC_TIMER_0,
                .freq_hz          = frequency,
                .clk_cfg          = LEDC_AUTO_CLK
            };
            
            errorEsp = ledc_timer_config(&timer);

            if(errorEsp != ESP_OK) {
                ESP_LOGW(TAG, "DRV8876 Init - Failt init pwmTimer config");
                return ESP_FAIL;
            }

            ledc_channel_config_t channelConf = {
                .gpio_num   = enPin,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel    = pwmChannel,
                .intr_type  = LEDC_INTR_DISABLE,
                .timer_sel  = LEDC_TIMER_0,
                .duty       = 0,
                .hpoint     = 0
            };

            errorEsp = ledc_channel_config(&channelConf);

            if(errorEsp != ESP_OK) {
                ESP_LOGW(TAG, "DRV8876 Init - Failt init pwm channel");
                return ESP_FAIL;
            }

            isInitialized = true;

            return ESP_OK;
        }

        void DRV8876::setDirection(Direction direction) {
            if(isInitialized == false) {
                ESP_LOGW(TAG, "DRV8876 setDirection - DRV8876 is not initialized");
                return;
            }
            
            motorDirection = direction;
            gpio_set_level(phPin, static_cast<bool>(motorDirection));
        }

        void DRV8876::reverseDirection() {
            if(isInitialized == false) {
                return;
            }

            if(motorDirection == Direction::LEFT) {
                setDirection(Direction::RIGHT);
            }
            else {
                setDirection(Direction::LEFT);
            }
        }

        esp_err_t DRV8876::setSpeed(uint8_t speed) {
            if(isInitialized == false) {
                ESP_LOGW(TAG, "DRV8876 setSpeed - DRV8876 is not initialized");
                return ESP_FAIL;
            }

            if(speed < minMotorSpeed || speed > maxMotorSpeed) {
                ESP_LOGW(TAG, "Invalid speed variable");
                return ESP_FAIL;
            }
            
            motorSpeed = speed;
            uint8_t pwmVal = (motorSpeed * 255) / 100;
            setPwmDuty(pwmChannel, pwmVal);

            return ESP_OK;
        }

        void DRV8876::stop() {
            if(isInitialized == false) {
                ESP_LOGW(TAG, "DRV8876 stop - DRV8876 is not initialized");
                return;
            }

            gpio_set_level(phPin, motorStop);
            setPwmDuty(pwmChannel, motorStop);
        }

        uint8_t DRV8876::getMotorSpeed()       const { return motorSpeed; }
        Direction DRV8876::getMotorDirection() const { return motorDirection; }
        bool DRV8876::motorIsRunning()         const { return motorSpeed > 0; }

        esp_err_t DRV8876::setPwmDuty(ledc_channel_t pwmChannel, uint8_t duty) {
            esp_err_t returnValue = ESP_OK;
            
            returnValue = ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, duty);

            if(returnValue != ESP_OK) {
                ESP_LOGW(TAG, "DRV8876 setPwmDuty - ledc set duty cycle error");
                return ESP_FAIL;
            }

            returnValue = ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);

            if(returnValue != ESP_OK) {
                ESP_LOGW(TAG, "DRV8876 setPwmDuty - Ledc update duty cycle error");
                return ESP_FAIL;
            }

            return ESP_OK;
        }

        esp_err_t DRV8876::setPwmValue(ledc_timer_bit_t resolution, uint32_t frequency) {
            if(isInitialized == false) {
                ESP_LOGW(TAG, "DRV8876 setSpeed - DRV8876 is not initialized");
                return ESP_FAIL;
            }

            if(frequency < MIN_PWM_FREQ || frequency > MAX_PWM_FREQ) {
                ESP_LOGW(TAG, "DRV8876 setPwmValue - not correct freq value used");
                return ESP_FAIL;
            }

            esp_err_t errorEsp = ESP_OK;

            this->resolution = resolution;
            this->frequency = frequency;

            ledc_timer_config_t timer = {
                .speed_mode       = LEDC_LOW_SPEED_MODE,
                .duty_resolution  = resolution,
                .timer_num        = LEDC_TIMER_0,
                .freq_hz          = frequency,
                .clk_cfg          = LEDC_AUTO_CLK
            };
            
            errorEsp = ledc_timer_config(&timer);

            if(errorEsp != ESP_OK) {
                ESP_LOGW(TAG, "DRV8876 setPwmValue - Fail init pwmTimer config");
            }

            return ESP_OK;
        }

        void IRAM_ATTR DRV8876::faultISR(void* arg) {
            static_cast<DRV8876*>(arg)->faultTriggered = true;
        }

        bool DRV8876::isFaultTriggered() const { return faultTriggered; }
        void DRV8876::clearFaultFlag() { faultTriggered = false; }
    } 
} 
