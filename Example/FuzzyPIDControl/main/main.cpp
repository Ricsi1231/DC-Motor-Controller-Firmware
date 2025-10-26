#include "motorControl.hpp"
#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "PID.hpp"
#include "FuzzyPIDController.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <cmath>

using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;
using namespace DC_Motor_Controller_Firmware::PID;
using namespace DC_Motor_Controller_Firmware::Control;

static const char* TAG = "MotorExample";

constexpr gpio_num_t PH_PIN = GPIO_NUM_4;
constexpr gpio_num_t EN_PIN = GPIO_NUM_5;
constexpr gpio_num_t FAULT_PIN = GPIO_NUM_6;
constexpr gpio_num_t SLEEP_PIN = GPIO_NUM_7;
constexpr gpio_num_t ENCODER_A = GPIO_NUM_1;
constexpr gpio_num_t ENCODER_B = GPIO_NUM_2;

constexpr DRV8876Config motorConfig{.phPin = PH_PIN,
                                    .enPin = EN_PIN,
                                    .nFault = FAULT_PIN,
                                    .nSleep = SLEEP_PIN,
                                    .pwmChannel = LEDC_CHANNEL_0,
                                    .resolution = LEDC_TIMER_10_BIT,
                                    .frequency = 20000};

constexpr pcnt_unit_config_t unitCfg{.low_limit = -32767, .high_limit = 32767, .intr_priority = 0, .flags = {.accum_count = true}};

constexpr SpeedFilterConfig speedFilterCfg{.filterType = SpeedFilterType::EMA, .emaAlpha = 0.3f, .iirCutoffHz = 2.0f, .sampleRateHz = 100};

constexpr DirectionConfig directionCfg{.hysteresisThreshold = 8, .debounceTimeMs = 100, .enableHysteresis = true};

constexpr EncoderConfig encCfg{.pinA = ENCODER_A,
                               .pinB = ENCODER_B,
                               .unitConfig = unitCfg,
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
                               .speedFilter = speedFilterCfg,
                               .direction = directionCfg};

constexpr PidConfig pidConfig = {.kp = 2.5f,
                                 .ki = 0.08f,
                                 .kd = 0.15f,
                                 .maxOutput = 100.0f,
                                 .maxIntegral = 500.0f,
                                 .errorEpsilon = 0.1f,
                                 .speedEpsilon = 0.2f,
                                 .errorTimeoutSec = 2.0f,
                                 .stuckTimeoutSec = 1.0f};

constexpr MotorControllerConfig motorCfg = {.minSpeed = 3.0f,
                                            .maxSpeed = 95.0f,
                                            .minErrorToMove = 0.3f,
                                            .driftThreshold = 1.0f,
                                            .driftDeadband = 0.8f,
                                            .driftHysteresis = 0.4f,
                                            .stuckPositionEpsilon = 0.1f,
                                            .stuckCountLimit = 30,
                                            .pidWarmupLimit = 15,
                                            .countsPerRevolution = 1024,
                                            .motionTimeoutMs = 5000,
                                            .motionProfileEnabled = true,
                                            .motionProfileType = MotionProfileType::S_CURVE,
                                            .accelLimitPctPerSec = 200.0f,
                                            .jerkLimitPctPerSec2 = 4000.0f,
                                            .Kff_pos = 0.02f,
                                            .Kff_vel = 0.15f,
                                            .settlePosTolDeg = 0.5f,
                                            .settleVelTolDegPerSec = 2.0f,
                                            .settleCountLimit = 8};

DRV8876 motor(motorConfig);
Encoder encoder(encCfg);
PIDController pid(pidConfig);

FuzzyPidConfig fuzzyCfg = {};
FuzzyPIDController fuzzy(fuzzyCfg);
MotorController motorController(encoder, motor, pid, fuzzy, motorCfg);

struct ExampleState {
    int currentSequenceStep = 0;
    bool waitingForMotion = false;
    uint64_t lastStatusLogUs = 0;
    uint64_t sequenceStartUs = 0;
};

static ExampleState exampleState;

void onMotionDoneCallback(const MotorStatus& status, void* user) {
    ESP_LOGI(TAG, "Motion completed - Position: %.2f deg, Error: %.3f deg", status.position, status.error);
    ExampleState* state = static_cast<ExampleState*>(user);
    state->waitingForMotion = false;
}

void onStallCallback(const MotorStatus& status, void* user) {
    ESP_LOGW(TAG, "Motor stall detected at position %.2f deg, stuck count: %d", status.position, status.stuckCount);
    ExampleState* state = static_cast<ExampleState*>(user);
    state->waitingForMotion = false;
}

void onLimitHitCallback(const MotorStatus& status, void* user) {
    ESP_LOGW(TAG, "Soft limit hit - Position: %.2f deg, Target: %.2f deg", status.position, status.target);
    ExampleState* state = static_cast<ExampleState*>(user);
    state->waitingForMotion = false;
}

void runBasicPositioningDemo() {
    ESP_LOGI(TAG, "Starting basic positioning demo");

    const float positions[] = {0.0f, 90.0f, -45.0f, 180.0f, -90.0f, 0.0f};
    const size_t numPositions = sizeof(positions) / sizeof(positions[0]);

    for (size_t i = 0; i < numPositions; i++) {
        ESP_LOGI(TAG, "Moving to position: %.1f deg", positions[i]);

        exampleState.waitingForMotion = true;
        motorController.setTargetDegrees(positions[i]);

        while (exampleState.waitingForMotion) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "Basic positioning demo completed");
}

void runMotionProfilingDemo() {
    ESP_LOGI(TAG, "Starting motion profiling demo");

    MotionProfileType profiles[] = {MotionProfileType::TRAPEZOID, MotionProfileType::S_CURVE};
    const char* profileNames[] = {"Trapezoid", "S-Curve"};

    for (int i = 0; i < 2; i++) {
        ESP_LOGI(TAG, "Testing %s profile", profileNames[i]);

        motorController.configureMotionProfile(profiles[i], 150.0f, 3000.0f);

        exampleState.waitingForMotion = true;
        motorController.setTargetDegrees(270.0f);

        while (exampleState.waitingForMotion) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        exampleState.waitingForMotion = true;
        motorController.setTargetDegrees(0.0f);

        while (exampleState.waitingForMotion) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Motion profiling demo completed");
}

void runSoftLimitsDemo() {
    ESP_LOGI(TAG, "Starting soft limits demo");

    motorController.setSoftLimits(-120.0f, 120.0f, true);
    ESP_LOGI(TAG, "Set soft limits: -120 deg to 120 deg");

    ESP_LOGI(TAG, "Attempting to move to 150 deg (should be limited)");
    exampleState.waitingForMotion = true;
    motorController.setTargetDegrees(150.0f);

    while (exampleState.waitingForMotion) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Attempting to move to -150 deg (should be limited)");
    exampleState.waitingForMotion = true;
    motorController.setTargetDegrees(-150.0f);

    while (exampleState.waitingForMotion) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    motorController.setSoftLimits(-360.0f, 360.0f, false);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Soft limits demo completed");
}

void runPIDTuningDemo() {
    ESP_LOGI(TAG, "Starting PID tuning demo");

    struct PIDTestCase {
        float kp, ki, kd;
        const char* description;
    } testCases[] = {{1.0f, 0.02f, 0.05f, "Conservative gains"}, {3.0f, 0.1f, 0.2f, "Aggressive gains"}, {2.5f, 0.08f, 0.15f, "Balanced gains"}};

    for (auto& testCase : testCases) {
        ESP_LOGI(TAG, "Testing %s: Kp=%.2f, Ki=%.3f, Kd=%.3f", testCase.description, testCase.kp, testCase.ki, testCase.kd);

        motorController.setPID(testCase.kp, testCase.ki, testCase.kd);

        exampleState.waitingForMotion = true;
        motorController.setTargetDegrees(90.0f);

        while (exampleState.waitingForMotion) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        exampleState.waitingForMotion = true;
        motorController.setTargetDegrees(0.0f);

        while (exampleState.waitingForMotion) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "PID tuning demo completed");
}

void runFeedForwardDemo() {
    ESP_LOGI(TAG, "Starting feed-forward demo");

    ESP_LOGI(TAG, "Testing without feed-forward");
    motorController.setFeedForward(0.0f, 0.0f);

    exampleState.waitingForMotion = true;
    motorController.setTargetDegrees(180.0f);

    while (exampleState.waitingForMotion) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Testing with feed-forward gains");
    motorController.setFeedForward(0.02f, 0.15f);

    exampleState.waitingForMotion = true;
    motorController.setTargetDegrees(0.0f);

    while (exampleState.waitingForMotion) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Feed-forward demo completed");
}

void statusMonitorTask(void* param) {
    const TickType_t logInterval = pdMS_TO_TICKS(500);

    while (true) {
        if (exampleState.waitingForMotion) {
            MotorStatus status = motorController.getStatus();

            ESP_LOGI(TAG,
                     "Status - Pos: %6.2f deg | Target: %6.2f deg | Error: %5.2f deg | "
                     "Vel: %6.1f deg/s | PID: %5.1f%% | Done: %s",
                     status.position, status.target, status.error, status.velocity, status.pidOutput, status.motionDone ? "YES" : "NO");
        }

        vTaskDelay(logInterval);
    }
}

void demoSequenceTask(void* param) {
    ESP_LOGI(TAG, "Starting motor controller demo sequence");

    vTaskDelay(pdMS_TO_TICKS(2000));

    motorController.setOnMotionDone(onMotionDoneCallback, &exampleState);
    motorController.setOnStall(onStallCallback, &exampleState);
    motorController.setOnLimitHit(onLimitHitCallback, &exampleState);

    runBasicPositioningDemo();
    runMotionProfilingDemo();
    runSoftLimitsDemo();
    runPIDTuningDemo();
    runFeedForwardDemo();

    ESP_LOGI(TAG, "Demo sequence completed");

    while (true) {
        exampleState.waitingForMotion = true;
        motorController.setTargetDegrees(45.0f);

        while (exampleState.waitingForMotion) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));

        exampleState.waitingForMotion = true;
        motorController.setTargetDegrees(-45.0f);

        while (exampleState.waitingForMotion) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Initializing motor controller example");

    esp_err_t result = encoder.init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize encoder: %s", esp_err_to_name(result));
        return;
    }

    result = encoder.start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start encoder: %s", esp_err_to_name(result));
        return;
    }

    encoder.setInverted(false);
    encoder.setGearRatio(1.0f);

    result = motor.init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motor: %s", esp_err_to_name(result));
        return;
    }

    motorController.setUpdateHz(100);
    motorController.setMotionTimeoutMs(10000);
    motorController.configureControlTask(1, 15);

    motorController.startTask();

    ESP_LOGI(TAG, "Motor controller initialized and started");

    xTaskCreatePinnedToCore(statusMonitorTask, "StatusMonitor", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(demoSequenceTask, "DemoSequence", 8192, NULL, 10, NULL, 0);

    ESP_LOGI(TAG, "Demo tasks created and running");

    vTaskDelete(NULL);
}
