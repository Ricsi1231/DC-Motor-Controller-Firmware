#include "UartBus.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace DC_Motor_Controller_Firmware::UART;
using namespace DC_Motor_Controller_Firmware::PeripheryBus;

static const char* TAG = "MAIN";

extern "C" void app_main() {
    UartConfig uartConfig = {
        .port = UART_NUM_1,
        .txPin = GPIO_NUM_17,
        .rxPin = GPIO_NUM_18,
        .rtsPin = GPIO_NUM_NC,
        .ctsPin = GPIO_NUM_NC,
        .baudRate = 115200,
        .dataBits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stopBits = UART_STOP_BITS_1,
        .flowControl = UART_HW_FLOWCTRL_DISABLE,
        .rxBufferSize = 1024,
        .txBufferSize = 0,
        .readTimeoutMs = 100,
    };

    UartBus uart(uartConfig);

    BusStatus status = uart.init();
    if (status != BusStatus::OK) {
        ESP_LOGE(TAG, "UART init failed (status=%u)", static_cast<unsigned>(status));
        return;
    }

    DeviceDescriptor device = {.id = 0, .config = 0};
    status = uart.addDevice(device);
    if (status != BusStatus::OK) {
        ESP_LOGE(TAG, "addDevice failed (status=%u)", static_cast<unsigned>(status));
        return;
    }

    ESP_LOGI(TAG, "UART bus ready (type=%u)", static_cast<unsigned>(uart.getType()));

    const uint8_t txMsg[] = "Hello from UartBus!\r\n";

    while (true) {
        status = uart.transfer(txMsg, sizeof(txMsg) - 1);
        if (status != BusStatus::OK) {
            ESP_LOGW(TAG, "transfer failed (status=%u)", static_cast<unsigned>(status));
        } else {
            ESP_LOGI(TAG, "Sent %u bytes", static_cast<unsigned>(sizeof(txMsg) - 1));
        }

        uint8_t rxBuf[128] = {};
        size_t rxSize = sizeof(rxBuf);
        status = uart.transferAndReceive(nullptr, 0, rxBuf, &rxSize);
        if (status == BusStatus::OK && rxSize > 0) {
            ESP_LOGI(TAG, "Received %u bytes", static_cast<unsigned>(rxSize));
            ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, rxSize, ESP_LOG_INFO);

            uart.transfer(rxBuf, rxSize);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
