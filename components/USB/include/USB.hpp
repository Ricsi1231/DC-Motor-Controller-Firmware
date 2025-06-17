#pragma once

#include "tusb_cdc_acm.h"

namespace DC_Motor_Controller_Firmware {
namespace USB {
class USB {
  typedef struct {
    uint8_t buffer[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
    size_t bufferSize;
    uint8_t itf;
  } usbMessage;

public:
  USB();
  ~USB();

  esp_err_t init();
  esp_err_t sendData(uint8_t *data, size_t dataSize);
  esp_err_t receiveData(uint8_t *data, size_t *rxBufferSize);

  esp_err_t sendFormattedString(const char *fmt, ...);
  esp_err_t sendString(const char *str);

  bool usbIsConnected() const;
  bool newDataIsReceived() const;

  void flushRxBuffer();

private:
  static void usbCallback(int itf, cdcacm_event_t *event);
  static void serialPortState(int itf, cdcacm_event_t *event);

  static bool serialIsOpen;
  bool isInitialized = false;

  static usbMessage message;
  static uint8_t rxBuffer[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
  static size_t rxSize;

  static const char *TAG;

  static tinyusb_cdcacm_itf_t USB_INTERFACE_PORT;
};
} // namespace USB
} // namespace DC_Motor_Controller_Firmware