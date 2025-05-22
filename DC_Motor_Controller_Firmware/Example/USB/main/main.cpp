#include "USB.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

using namespace DC_Motor_Controller_Firmware::USB;

static const char *TAG = "main";

static USB usb;

extern "C" void app_main(void) {
    esp_err_t usbState = usb.init();

    if (usbState != ESP_OK) {
        ESP_LOGE(TAG, "USB init failed: %s", esp_err_to_name(usbState));
        return;
    }

    uint8_t rxData[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
    size_t rxSize = 0;

    while(true) {
        if(usb.receiveData(rxData, &rxSize) == ESP_OK) {
            if(rxSize > 0) {
                ESP_LOGI(TAG, "Received %d bytes", rxSize);
                ESP_LOG_BUFFER_HEXDUMP(TAG, rxData, rxSize, ESP_LOG_INFO);

                usb.sendData(rxData, rxSize);
            }
        }
            
        rxSize = 0;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}