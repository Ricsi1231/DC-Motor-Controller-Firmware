#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "MotorCommHandler.hpp"
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

const char *TAG = "MAIN";

gpio_num_t PH_PIN = GPIO_NUM_4;
gpio_num_t EN_PIN = GPIO_NUM_5;
gpio_num_t FAULT_PIN = GPIO_NUM_6;
ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

gpio_num_t ENCODER_A = GPIO_NUM_1;
gpio_num_t ENCODER_B = GPIO_NUM_2;
uint16_t ppr = 1024;

pcnt_unit_config_t pcntUnit = {.low_limit = -1000,
                               .high_limit = 1000,
                               .flags = {
                                   .accum_count = true,
                               }};

float eprev = 0;
float eintegral = 0;
uint8_t motorTarget = 0;
float setpointDegrees = 0;
bool motionDone = true;

DRV8876 motor(PH_PIN, EN_PIN, FAULT_PIN, PWM_CHANNEL);
Encoder encoder(ENCODER_A, ENCODER_B, pcntUnit, ppr);
DC_Motor_Controller_Firmware::USB::USB usb;
MotorCommHandler motorComm(usb);

void motorTest(bool rotation);
void encoderTest();
void pidMotorControl(double kp, double ki, double kd, float target);
void testLabview();

bool testMotor = false;
bool testEncoder = false;
bool pidControl = false;
bool labViewTest = true;

esp_err_t errorStatus = ESP_OK;

extern "C" void app_main() {
  errorStatus = motor.init();
  if (errorStatus != ESP_OK) {
    ESP_LOGD(TAG, "Error with motor init");
  }

  errorStatus = encoder.init();
  if (errorStatus != ESP_OK) {
    ESP_LOGD(TAG, "Error with encoder init");
  }

  errorStatus = usb.init();
  if (errorStatus != ESP_OK) {
    ESP_LOGD(TAG, "Error with USB init");
  }

  while (1) {
    if (testMotor == true) {
      motorTest(true);
      motorTest(false);
    }

    if (testEncoder == true) {
      encoderTest();
    }

    if (pidControl == true) {
      pidMotorControl(1, 0, 0, 180);
    }

    if (labViewTest == true) {
      testLabview();
    }

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
  ESP_LOGI("ENCODER", "Ticks: %lu, Degrees: %.2f, RPM: %ld", ticks, degrees,
           rpm);
}

void pidMotorControl(double kp, double ki, double kd, float target) {
  static int64_t prevT = 0;
  static float lastPos = 0;
  static int64_t stuckStartTime = 0;
  static int64_t smallErrorStartTime = 0;

  int64_t currT = esp_timer_get_time();
  float deltaT = (float)(currT - prevT) / 1e6f;
  if (deltaT <= 0)
    deltaT = 0.001f;
  prevT = currT;

  float pos = encoder.getPositionInDegrees();
  float e = target - pos;
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;

  float maxIntegral = 1000.0f;
  if (eintegral > maxIntegral)
    eintegral = maxIntegral;
  if (eintegral < -maxIntegral)
    eintegral = -maxIntegral;

  float u = kp * e + ki * eintegral + kd * dedt;
  float motorSpeed = fabs(u);
  if (motorSpeed > 100.0f)
    motorSpeed = 100.0f;

  if (fabs(e) < 5.0f && motorSpeed < 10.0f) {
    if (smallErrorStartTime == 0)
      smallErrorStartTime = currT;
    if ((currT - smallErrorStartTime) / 1e6f > 0.6f) {
      motor.stop();
      motionDone = true;
      eintegral = 0;
      eprev = 0;
      ESP_LOGW("PID", "Forced stop due to low error + speed timeout");
      return;
    }
  } else {
    smallErrorStartTime = 0;
  }

  if (fabs(pos - lastPos) < 0.05f && motorSpeed < 7.0f) {
    if (stuckStartTime == 0)
      stuckStartTime = currT;
    if ((currT - stuckStartTime) / 1e6f > 0.5f) {
      motor.stop();
      motionDone = true;
      eintegral = 0;
      eprev = 0;
      ESP_LOGW("PID", "Forced stop due to zero motion");
      return;
    }
  } else {
    stuckStartTime = 0;
  }
  lastPos = pos;

  if (fabs(e) < 2.0f && motorSpeed < 7.0f) {
    motor.stop();
    motionDone = true;
    eintegral = 0;
    eprev = 0;
    return;
  }

  if (u < 0) {
    motor.setDirection(Direction::LEFT);
  } else {
    motor.setDirection(Direction::RIGHT);
  }

  if (motorSpeed < 7.0f) {
    motor.setSpeed(0);
  } else {
    motor.setSpeed(motorSpeed);
  }

  eprev = e;

  ESP_LOGI("PID", "e=%.2f dedt=%.2f u=%.2f speed=%.2f pos=%.2f", e, dedt, u,
           motorSpeed, pos);
}

void testLabview() {
  float kp = 1.2, ki = 0.01, kd = 0.04;

  motorComm.process();

  if (motorComm.isNewTargetReceived()) {
    float currentPos = encoder.getPositionInDegrees();
    float offset = motorComm.getTargetDegrees();
    setpointDegrees = currentPos + offset;

    eprev = 0;
    eintegral = 0;
    motionDone = false;

    motorComm.clearTarget();
  }

  if (motorComm.isNewPIDReceived()) {
    motorComm.getPIDParams(kp, ki, kd);
  }

  if (motorComm.wasPIDRequested()) {
    motorComm.getPIDParams(kp, ki, kd);
    motorComm.sendPIDParams(kp, ki, kd);
  }

  if (!motionDone) {
    pidMotorControl(kp, ki, kd, setpointDegrees);
  }
}