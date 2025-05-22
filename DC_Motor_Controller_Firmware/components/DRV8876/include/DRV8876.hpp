#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"

namespace DC_Motor_Controller_Firmware {
    namespace DRV8876 {
        enum class Direction : bool {
            LEFT = false,
            RIGHT = true
        };

        class DRV8876 {
        public:
            DRV8876(gpio_num_t phPin, gpio_num_t enPin, gpio_num_t nFault, ledc_channel_t pwmChannel);

            esp_err_t init();

            void setDirection(Direction direction);  
            void reverseDirection();

            esp_err_t setSpeed(uint8_t speed);
            void stop();

            uint8_t getMotorSpeed() const;
            Direction getMotorDirection() const;

            bool motorIsRunning() const;

            bool isFaultTriggered() const;
            void clearFaultFlag();

            esp_err_t setPwmValue(ledc_timer_bit_t resolution, uint32_t frequency = 20000);

        private:
            esp_err_t setPwmDuty(ledc_channel_t pwmChannel, uint8_t duty);

            static IRAM_ATTR void faultISR(void* arg);
            
            gpio_num_t phPin;
            gpio_num_t enPin;
            gpio_num_t nFault;
            ledc_channel_t pwmChannel;

            ledc_timer_bit_t resolution = LEDC_TIMER_10_BIT;
            uint32_t frequency = 20000;

            Direction motorDirection;
            uint8_t motorSpeed;

            volatile bool faultTriggered = false;

            const uint8_t minMotorSpeed = 0;
            const uint8_t maxMotorSpeed = 100;

            const uint32_t MIN_PWM_FREQ = 1000; 
            const uint32_t MAX_PWM_FREQ = 30000;    

            const uint8_t motorStop = 0;

            bool isInitialized = false;

            const char *TAG = "DRV8876";
        };
    }
}

