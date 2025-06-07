#pragma once

#include "Encoder.hpp"
#include "esp_log.h"
#include <math.h>

namespace DC_Motor_Controller_Firmware {
namespace Encoder {
Encoder::Encoder(gpio_num_t pinA, gpio_num_t pinB,
                 pcnt_unit_config_t unitConfig, uint16_t ppr) {
  this->pinA = pinA;
  this->pinB = pinB;
  this->ppr = ppr;
  this->unitConfig = unitConfig;

  this->chanA = nullptr;
  this->chanB = nullptr;
  this->timer = nullptr;
  this->lastCount = 0;
  this->rpm = 0.0f;
}

esp_err_t Encoder::init() {
  esp_err_t returnValue = ESP_OK;

  returnValue = initPcnt();

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder init - Error to init Pcnt");
    return ESP_FAIL;
  }

  returnValue = initPcntFilter();

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder init - Error to init Pcnt filter");
    return ESP_FAIL;
  }

  returnValue = initPcntIo();

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder init - Error to init Pcnt Io");
    return ESP_FAIL;
  }

  returnValue = initTimer();

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder init - Error to init Esp timer");
    return ESP_FAIL;
  }

  returnValue = initWatchPoint();

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder init - Error to init Pcnt watch point");
    return ESP_FAIL;
  }

  returnValue = enablePcUnit();

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder init - Error to enable Pcnt unit");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t Encoder::resetPositon() {
  esp_err_t returnValue = ESP_OK;

  lastCount = 0;
  returnValue = pcnt_unit_clear_count(pcntUnit);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder resetPositon - Faild reset Pcnt value");
    return ESP_FAIL;
  }

  return ESP_OK;
}

int32_t Encoder::getPositionTicks() const {
  int count = 0;

  pcnt_unit_get_count(pcntUnit, &count);

  return count;
}

float Encoder::getPositionInDegrees() const {
  int count = 0;
  pcnt_unit_get_count(pcntUnit, &count);

  float degree = (static_cast<float>(count) / ppr) * 360.0f;
  return degree;
}

int32_t Encoder::getPositionInRPM() const { return rpm; }

motorDirection Encoder::getMotorDriection() const {
  if (rpm < 0) {
    return motorDirection::LEFT;
  } else {
    return motorDirection::RIGHT;
  }
}

esp_err_t Encoder::initPcnt() {
  esp_err_t returnValue = ESP_OK;

  returnValue = pcnt_new_unit(&unitConfig, &pcntUnit);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initPcnt - Error to add new pcnt unit");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t Encoder::initPcntFilter() {
  esp_err_t returnValue = ESP_OK;

  pcnt_glitch_filter_config_t filterConfig = {.max_glitch_ns = 1000};

  returnValue = pcnt_unit_set_glitch_filter(pcntUnit, &filterConfig);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initPcntFilter - Error to set Pcnt gitch filter");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t Encoder::initPcntIo() {
  esp_err_t returnValue = ESP_OK;

  pcnt_chan_config_t encoderChannelAConfig = {.edge_gpio_num = pinA,
                                              .level_gpio_num = pinB};

  pcnt_chan_config_t encoderChannelBConfig = {.edge_gpio_num = pinB,
                                              .level_gpio_num = pinA};

  returnValue = pcnt_new_channel(pcntUnit, &encoderChannelAConfig, &chanA);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initPcntIo - Error to set new Pcnt channel");
    return ESP_FAIL;
  }

  returnValue =
      pcnt_channel_set_edge_action(chanA, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initPcntIo - Error to set channel A");
    return ESP_FAIL;
  }

  returnValue = pcnt_channel_set_level_action(
      chanA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initPcntIo - Error to set channel A section");
    return ESP_FAIL;
  }

  returnValue = pcnt_new_channel(pcntUnit, &encoderChannelBConfig, &chanB);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initPcntIo - Error to set channel B");
    return ESP_FAIL;
  }

  returnValue =
      pcnt_channel_set_edge_action(chanB, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE);

  if (returnValue != ESP_OK) {
    return ESP_FAIL;
  }

  returnValue = pcnt_channel_set_level_action(
      chanB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initPcntIo - Error to set channel B section");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t Encoder::initWatchPoint() {
  esp_err_t returnValue = ESP_OK;

  returnValue = pcnt_unit_add_watch_point(pcntUnit, unitConfig.low_limit);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG,
             "Encoder initWatchPoint - Error to low limit if Pcnt watchpoint");
    return ESP_FAIL;
  }

  returnValue = pcnt_unit_add_watch_point(pcntUnit, unitConfig.high_limit);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG,
             "Encoder initWatchPoint - Error to high limit if Pcnt watchpoint");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t Encoder::enablePcUnit() {
  esp_err_t returnValue = ESP_OK;

  returnValue = pcnt_unit_enable(pcntUnit);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initWatchPoint - Failed anebale Pcnt unit");
    return ESP_FAIL;
  }

  returnValue = pcnt_unit_clear_count(pcntUnit);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initWatchPoint - Failed clear Pcnt unit");
    return ESP_FAIL;
  }

  returnValue = pcnt_unit_start(pcntUnit);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initWatchPoint - Failed start Pcnt unit");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t Encoder::initTimer() {
  esp_err_t returnValue = ESP_OK;

  const esp_timer_create_args_t timerConfig = {.callback = &timerCallback,
                                               .arg = this,
                                               .dispatch_method =
                                                   ESP_TIMER_TASK,
                                               .name = "rpm_timer"};

  returnValue = esp_timer_create(&timerConfig, &timer);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initTimer - Failed create init Esp timer");
    return ESP_FAIL;
  }

  returnValue = esp_timer_start_periodic(timer, 100000);

  if (returnValue != ESP_OK) {
    ESP_LOGW(TAG, "Encoder initTimer - Failed to set timer periodic");
    return ESP_FAIL;
  }

  return ESP_OK;
}

void Encoder::timerCallback(void *arg) {
  Encoder *self = static_cast<Encoder *>(arg);

  int currentCount = 0;
  pcnt_unit_get_count(self->pcntUnit, &currentCount);

  int delta = currentCount - self->lastCount;
  self->lastCount = currentCount;

  float newRpm = (static_cast<float>(delta) / self->ppr) * 600.0f;
  self->rpm = 0.3f * newRpm + 0.7f * self->rpm;
}
} // namespace Encoder
} // namespace DC_Motor_Controller_Firmware
