#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "DRV8876.hpp"
#include "Encoder.hpp"
#include "esp_timer.h"
#include "math.h"

using namespace DC_Motor_Controller_Firmware::DRV8876;
using namespace DC_Motor_Controller_Firmware::Encoder;

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

long prevT = 0;
float eprev = 0;
float eintegral = 0;

DRV8876 motor(PH_PIN, EN_PIN, FAULT_PIN, PWM_CHANNEL);
Encoder encoder(ENCODER_A, ENCODER_B, pcntUnit, ppr);

void motorTest(bool rotation);
void encoderTest();
void pidMotorControl(double kp, double ki, double kd, uint8_t target);

bool testMotor = false bool testEncoder = false;
bool pidControl = false;

esp_err_t errorStatus = ESP_OK;

extern "C" void app_main() {
  errorStatus = motor.init();
  if (errorStatus != ESP_OK) {
    ESP_LOGD(TAG, "Error with motor init");
  }

  encoder.init();
  if (errorStatus != ESP_OK) {
    ESP_LOGD(TAG, "Error with encoder init");
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
      pidMotorControl(1, 0, 0, 40);
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
  } else {
    motor.setDirection(Direction::RIGHT);
    motor.setSpeed(100);
    motor.stop();

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

void pidMotorControl(double kp, double ki, double kd, uint8_t target) {
  int64_t currT = esp_timer_get_time();
  float deltaT = (float)(currT - prevT) / (1e06);
  prevT = deltaT;

  int pos = encoder.getPositionInDegrees();
  int e = target - pos;

  float dedt = (e - eprev) / (deltaT);
  eintegral = eintegral + e * deltaT;

  float u = kp * e + ki * eintegral + kd * dedt;

  float motorSpeed = fabs(u);

  if (motorSpeed > 100) {
    motorSpeed = 100;
  }

  if (u < 0) {
    motor.setDirection(Direction::LEFT);
  } else {
    motor.setDirection(Direction::RIGHT);
  }

  motor.setSpeed(motorSpeed);

  eprev = e;
}
