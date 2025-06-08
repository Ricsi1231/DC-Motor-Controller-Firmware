#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "MotorCommHandler.hpp"
#include "PID.hpp"
#include "USB.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;
using namespace DC_Motor_Controller_Firmware::USB;
using namespace DC_Motor_Controller_Firmware::Communication;
using namespace DC_Motor_Controller_Firmware::PID;

const char *TAG = "MAIN";

gpio_num_t PH_PIN = GPIO_NUM_4;
gpio_num_t EN_PIN = GPIO_NUM_5;
gpio_num_t FAULT_PIN = GPIO_NUM_6;
ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

gpio_num_t ENCODER_A = GPIO_NUM_1;
gpio_num_t ENCODER_B = GPIO_NUM_2;
uint16_t ppr = 1024;

pcnt_unit_config_t pcntUnit = {
    .low_limit = -1000,
    .high_limit = 1000,
    .flags = {
        .accum_count = true,
    }};

float setpointDegrees = 0;
bool motionDone = true;

DRV8876 motor(PH_PIN, EN_PIN, FAULT_PIN, PWM_CHANNEL);
Encoder encoder(ENCODER_A, ENCODER_B, pcntUnit, ppr);
USB usb;
MotorCommHandler motorComm(usb);

PidConfig defaultConfig = {
    .kp = 1.2f,
    .ki = 0.05f,         // ↑ stronger correction over time
    .kd = 0.02f,         // ↓ reduce overshoot instability
    .maxOutput = 100.0f,
    .maxIntegral = 300.0f,
    .errorEpsilon = 0.1f,  // ↓ better accuracy
    .speedEpsilon = 0.2f,  // ↓ better precision
    .errorTimeoutSec = 0.5f,
    .stuckTimeoutSec = 0.5f,
};
PIDController pid(defaultConfig);

void motorTest(bool rotation);
void encoderTest();
void testLabview();

bool testMotor = false;
bool testEncoder = false;
bool pidControl = false;
bool labViewTest = true;

esp_err_t errorStatus = ESP_OK;

extern "C" void app_main() {
  errorStatus = motor.init();
  if (errorStatus != ESP_OK) ESP_LOGD(TAG, "Error with motor init");

  errorStatus = encoder.init();
  if (errorStatus != ESP_OK) ESP_LOGD(TAG, "Error with encoder init");

  errorStatus = usb.init();
  if (errorStatus != ESP_OK) ESP_LOGD(TAG, "Error with USB init");

  while (1) {
    if (testMotor) motorTest(true);
    if (testMotor) motorTest(false);
    if (testEncoder) encoderTest();
    if (labViewTest) testLabview();

    vTaskDelay(10);
  }
}

void motorTest(bool rotation) {
  if (rotation) {
    motor.setDirection(Direction::LEFT);
    motor.setSpeed(100);
    motor.stop();
    vTaskDelay(1000);
    motor.setSpeed(60);
    motor.stop();
    ESP_LOGI(TAG, "Motor right");
  } else {
    motor.setDirection(Direction::RIGHT);
    motor.setSpeed(100);
    motor.stop();
    ESP_LOGI(TAG, "Motor LEFT");
    vTaskDelay(1000);
    motor.setSpeed(60);
    motor.stop();
  }
}

void encoderTest() {
  uint32_t ticks = encoder.getPositionTicks();
  float degrees = encoder.getPositionInDegrees();
  int32_t rpm = encoder.getPositionInRPM();
  ESP_LOGI("ENCODER", "Ticks: %lu, Degrees: %.2f, RPM: %ld", ticks, degrees, rpm);
}
void testLabview() {
  motorComm.process();

  static float lastPos = 0;
  static int stuckCounter = 0;
  static int pidWarmupCounter = 0;

  if (motorComm.isNewTargetReceived()) {
    float currentPos = encoder.getPositionInDegrees();
    float offset = motorComm.getTargetDegrees();
    setpointDegrees = currentPos + offset;

    pid.reset();
    motionDone = false;
    lastPos = currentPos;
    stuckCounter = 0;
    pidWarmupCounter = 0;

    motorComm.clearTarget();
  }

  if (motorComm.isNewPIDReceived()) {
    float kp, ki, kd;
    motorComm.getPIDParams(kp, ki, kd);
    pid.setParameters(kp, ki, kd);
  }

  if (motorComm.wasPIDRequested()) {
    float kp, ki, kd;
    pid.getParameters(kp, ki, kd);
    motorComm.sendPIDParams(kp, ki, kd);
  }

  if (motionDone) {
    float drift = setpointDegrees - encoder.getPositionInDegrees();
    if (fabs(drift) > 1.0f) {
      ESP_LOGI("HOLD", "Drift correction: error=%.2f", drift);
      pid.reset();
      motionDone = false;
      stuckCounter = 0;
      pidWarmupCounter = 0;
    }
    return;
  }

  float currentPos = encoder.getPositionInDegrees();
  float output = pid.compute(setpointDegrees, currentPos);

  float speed = fabs(output);
  if (speed < 2.0f && fabs(setpointDegrees - currentPos) > 0.2f)
    speed = 2.0f;
  if (speed > 100.0f) speed = 100.0f;

  if (fabs(currentPos - lastPos) < 0.05f)
    stuckCounter++;
  else
    stuckCounter = 0;
  lastPos = currentPos;

  if (stuckCounter > 50 || (pidWarmupCounter > 10 && pid.isSettled())) {
    ESP_LOGW("LABVIEW", "Motion done or stuck → stopping");
    motor.stop();
    motionDone = true;
    return;
  }
  pidWarmupCounter++;

  motor.setDirection(output < 0 ? Direction::LEFT : Direction::RIGHT);
  motor.setSpeed(speed);
}



