#include "Encoder.hpp"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace DC_Motor_Controller_Firmware::Encoder;

#define ENCODER_A_GPIO GPIO_NUM_15
#define ENCODER_B_GPIO GPIO_NUM_16

#define FORWARD  true
#define BACKWARD false

#define ENCODER_STEPS 100
#define ENCODER_DELAY 1000

uint8_t simulationTime = 0;

void initEncoderSimulation() {
    gpio_reset_pin(ENCODER_A_GPIO);
    gpio_reset_pin(ENCODER_B_GPIO);

    gpio_set_direction(ENCODER_A_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(ENCODER_B_GPIO, GPIO_MODE_OUTPUT);
}

void encoderSimulation(int steps, int delay_us = 1000, bool rotationDirection = true) {
    if (rotationDirection == true) {
        const int states[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};

        for (int i = 0; i < steps; ++i) {
            for (int s = 0; s < 4; ++s) {
                gpio_set_level(ENCODER_A_GPIO, states[s][0]);
                esp_rom_delay_us(delay_us);
                gpio_set_level(ENCODER_B_GPIO, states[s][1]);
                esp_rom_delay_us(delay_us);
            }
        }
    } else {
        const int states[4][2] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};

        for (int i = 0; i < steps; ++i) {
            for (int s = 0; s < 4; ++s) {
                gpio_set_level(ENCODER_A_GPIO, states[s][0]);
                esp_rom_delay_us(delay_us);
                gpio_set_level(ENCODER_B_GPIO, states[s][1]);
                esp_rom_delay_us(delay_us);
            }
        }
    }
}

extern "C" void app_main() {
    constexpr pcnt_unit_config_t pcntUnitConfig = {.low_limit = -1000, .high_limit = 1000, .intr_priority = 0, .flags = {.accum_count = true}};

    constexpr SpeedFilterConfig speedFilterConfig = {.filterType = SpeedFilterType::EMA, .emaAlpha = 0.3f, .iirCutoffHz = 2.0f, .sampleRateHz = 100};

    constexpr DirectionConfig directionConfig = {.hysteresisThreshold = 8, .debounceTimeMs = 100, .enableHysteresis = true};

    constexpr EncoderConfig encoderConfig = {.pinA = GPIO_NUM_1,
                                      .pinB = GPIO_NUM_2,
                                      .unitConfig = pcntUnitConfig,
                                      .pulsesPerRevolution = 1024,
                                      .filterThresholdNs = 1000,
                                      .rpmCalcPeriodUs = 100000,
                                      .maxRpm = 5000,
                                      .enableWatchPoint = true,
                                      .watchLowLimit = 0,
                                      .watchHighLimit = 0,
                                      .openCollectorInputs = false,
                                      .rpmBlendThreshold = 10,
                                      .rpmBlendBand = 3,
                                      .speedFilter = speedFilterConfig,
                                      .direction = directionConfig};

    Encoder encoder(encoderConfig);
    initEncoderSimulation();

    esp_err_t errorStatus = encoder.init();

    if (errorStatus != ESP_OK) {
        ESP_LOGE("ENCODER", "Failed to initialize encoder");
        return;
    }

    errorStatus = encoder.start();

    if (errorStatus != ESP_OK) {
        ESP_LOGE("ENCODER", "Failed to start encoder");
        return;
    }

    while (true) {
        encoderSimulation(ENCODER_STEPS, ENCODER_DELAY, BACKWARD);

        int32_t ticks = encoder.getPositionTicks();
        float degrees = encoder.getPositionInDegrees();
        int32_t rpm = encoder.getRpmRounded();

        ESP_LOGI("ENCODER", "Ticks: %ld, Degrees: %.2f, RPM: %ld", ticks, degrees, rpm);

        if (simulationTime == 10) {
            ESP_LOGI("ENCODER", "Reset Position");
            encoder.resetPosition();

            simulationTime = 0;
        }

        if (encoder.getMotorDirection() == motorDirection::LEFT) {
            ESP_LOGI("ENCODER", "Motor rotating to left");
        } else {
            ESP_LOGI("ENCODER", "Motor rotating to right");
        }

        simulationTime++;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
