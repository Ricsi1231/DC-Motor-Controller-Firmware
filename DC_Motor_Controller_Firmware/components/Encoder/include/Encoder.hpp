#pragma once

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

namespace DC_Motor_Controller_Firmware {
    namespace Encoder {

        enum class motorDirection: bool {
            LEFT = true,
            RIGHT = false
        };
        class Encoder {
            public:
                Encoder(gpio_num_t pinA, gpio_num_t pinB, pcnt_unit_config_t unitConfig, uint16_t ppr);
                esp_err_t init();

                esp_err_t resetPositon();

                int32_t getPositionTicks() const;
                float getPositionInDegrees() const;
                int32_t getPositionInRPM() const;

                motorDirection getMotorDriection() const;

            private:
                esp_err_t initPcnt();
                esp_err_t initPcntFilter();
                esp_err_t initPcntIo();
                esp_err_t initTimer();
                esp_err_t initWatchPoint();
                esp_err_t enablePcUnit();

                static void timerCallback(void* arg);

                gpio_num_t pinA;
                gpio_num_t pinB;
                uint16_t ppr;

                pcnt_unit_handle_t pcntUnit;
                pcnt_unit_config_t unitConfig;

                pcnt_channel_handle_t chanA;
                pcnt_channel_handle_t chanB;

                esp_timer_handle_t timer;

                int lastCount;
                float rpm;

                const char *TAG = "ENCODER";
        };
    }
}
