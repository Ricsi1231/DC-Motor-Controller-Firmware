#include "USB.hpp"
#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

namespace DC_Motor_Controller_Firmware {
namespace USB {
bool USB::serialIsOpen = false;
uint8_t USB::rxBuffer[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1] = {0};
USB::usbMessage USB::message = {};

static portMUX_TYPE usbSpinLock = portMUX_INITIALIZER_UNLOCKED;

tinyusb_cdcacm_itf_t USB::USB_INTERFACE_PORT = TINYUSB_CDC_ACM_0;

const char* USB::TAG = "USB";

size_t USB::rxSize;

static inline bool messageArrived = false;

USB::USB() { rxSize = 0; }

USB::~USB() {}

esp_err_t USB::init() {
    esp_err_t returnValue = ESP_OK;

    const tinyusb_config_t usbCfg = {
        .device_descriptor = NULL, .string_descriptor = NULL, .external_phy = false, .configuration_descriptor = NULL, .vbus_monitor_io = -1};

    tinyusb_config_cdcacm_t amcCfg = {.usb_dev = TINYUSB_USBDEV_0,
                                      .cdc_port = TINYUSB_CDC_ACM_0,
                                      .rx_unread_buf_sz = 64,
                                      .callback_rx = &usbCallback,
                                      .callback_rx_wanted_char = NULL,
                                      .callback_line_state_changed = NULL,
                                      .callback_line_coding_changed = NULL};

    returnValue = tinyusb_driver_install(&usbCfg);

    if (returnValue != ESP_OK) {
        ESP_LOGW(TAG, "USB init - cannot install usb config");
        return ESP_FAIL;
    }

    returnValue = tusb_cdc_acm_init(&amcCfg);

    if (returnValue != ESP_OK) {
        ESP_LOGW(TAG, "USB init - cannot install acm config, trying to use other port");

        amcCfg.cdc_port = TINYUSB_CDC_ACM_1;
        USB::USB_INTERFACE_PORT = TINYUSB_CDC_ACM_1;

        returnValue = tusb_cdc_acm_init(&amcCfg);

        if (returnValue != ESP_OK) {
            ESP_LOGW(TAG, "USB init - cannot install acm config, port or other problem");
            return ESP_FAIL;
        }
    }

    returnValue = tinyusb_cdcacm_register_callback(USB_INTERFACE_PORT, CDC_EVENT_LINE_STATE_CHANGED, &serialPortState);

    if (returnValue != ESP_OK) {
        ESP_LOGW(TAG, "USB init - cannot register usb state callback function");
        return ESP_FAIL;
    }

    isInitialized = true;

    return ESP_OK;
}

void USB::serialPortState(int itf, cdcacm_event_t* event) { serialIsOpen = static_cast<bool>(event->line_state_changed_data.dtr); }

void USB::usbCallback(int itf, cdcacm_event_t* event) {
    size_t length = 0;
    esp_err_t errorState = tinyusb_cdcacm_read(static_cast<tinyusb_cdcacm_itf_t>(itf), rxBuffer, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &length);

    if (errorState != ESP_OK || length == 0) {
        return;
    }

    if (length == 1 && rxBuffer[0] == '\n') {
        return;
    }

    rxBuffer[length] = '\0';

    taskENTER_CRITICAL_ISR(&usbSpinLock);
    messageArrived = true;
    taskEXIT_CRITICAL_ISR(&usbSpinLock);

    ESP_LOGW(TAG, "Raw message: '%s'", reinterpret_cast<char*>(rxBuffer));
}

esp_err_t USB::receiveData(uint8_t* data, size_t* rxBufferSize) {
    if (!serialIsOpen || !isInitialized || !messageArrived) return ESP_FAIL;

    taskENTER_CRITICAL(&usbSpinLock);
    size_t len = strlen(reinterpret_cast<const char*>(rxBuffer));
    memcpy(data, rxBuffer, len + 1);
    *rxBufferSize = len;
    messageArrived = false;
    taskEXIT_CRITICAL(&usbSpinLock);

    return ESP_OK;
}

esp_err_t USB::sendData(uint8_t* data, size_t dataSize) {
    esp_err_t errorState = ESP_OK;
    size_t usbDataSize;

    if (serialIsOpen == false) {
        ESP_LOGW(TAG, "USB sendData - Serial port is not opened.");
        return ESP_FAIL;
    }

    if (isInitialized == false) {
        ESP_LOGW(TAG, "USB sendData - Serial port is not initialized.");
        return ESP_FAIL;
    }

    usbDataSize = tinyusb_cdcacm_write_queue(USB_INTERFACE_PORT, data, dataSize);

    if (usbDataSize != dataSize) {
        ESP_LOGW(TAG, "USB sendData - Not all bytes queued.");
        return ESP_FAIL;
    }

    errorState = tinyusb_cdcacm_write_flush(USB_INTERFACE_PORT, 0);

    if (errorState != ESP_OK) {
        ESP_LOGW(TAG, "USB sendData - USB cannot send data.");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t USB::sendString(const char* str) {
    if (!serialIsOpen || !isInitialized || str == nullptr) {
        return ESP_FAIL;
    }

    size_t dataSize = strlen(str);

    size_t queued = tinyusb_cdcacm_write_queue(USB_INTERFACE_PORT, reinterpret_cast<const uint8_t*>(str), dataSize);

    if (queued != dataSize) {
        ESP_LOGW(TAG, "USB sendString - Not all bytes queued (%d/%d).", queued, dataSize);
        return ESP_FAIL;
    }

    tinyusb_cdcacm_write_flush(USB_INTERFACE_PORT, 0);

    return ESP_OK;
}

esp_err_t USB::sendFormattedString(const char* fmt, ...) {
    if (!serialIsOpen || !isInitialized) {
        ESP_LOGW(TAG, "USB sendFormattedString - USB not ready.");
        return ESP_FAIL;
    }

    esp_err_t errorStatus = ESP_OK;
    char buffer[64];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);

    va_end(args);

    errorStatus = sendString(buffer);

    if (errorStatus != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void USB::flushRxBuffer() {
    taskENTER_CRITICAL(&usbSpinLock);
    message.bufferSize = 0;
    memset(message.buffer, 0, sizeof(message.buffer));
    taskEXIT_CRITICAL(&usbSpinLock);
}

bool USB::usbIsConnected() const { return serialIsOpen && isInitialized; }
bool USB::newDataIsReceived() const { return rxSize > 0; }
}  // namespace USB
}  // namespace DC_Motor_Controller_Firmware