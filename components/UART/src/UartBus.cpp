#include "UartBus.hpp"

#include "esp_log.h"

namespace DC_Motor_Controller_Firmware {
namespace UART {

using PeripheryBus::BusStatus;
using PeripheryBus::DeviceDescriptor;
using PeripheryBus::TypeId;

UartBus::UartBus(const UartConfig& config) : config(config) {
    busMutex = xSemaphoreCreateMutex();
    if (busMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create bus mutex");
    }
}

UartBus::~UartBus() {
    if (initialized) {
        deinit();
    }
    if (busMutex != nullptr) {
        vSemaphoreDelete(busMutex);
        busMutex = nullptr;
    }
}

UartBus::UartBus(UartBus&& other) noexcept
    : config(other.config), initialized(other.initialized), deviceAdded(other.deviceAdded), busy(other.busy.load()), busMutex(other.busMutex) {
    other.initialized = false;
    other.busMutex = nullptr;
}

UartBus& UartBus::operator=(UartBus&& other) noexcept {
    if (this != &other) {
        if (initialized) {
            deinit();
        }
        if (busMutex != nullptr) {
            vSemaphoreDelete(busMutex);
        }
        config = other.config;
        initialized = other.initialized;
        deviceAdded = other.deviceAdded;
        busy.store(other.busy.load());
        busMutex = other.busMutex;
        other.initialized = false;
        other.busMutex = nullptr;
    }
    return *this;
}

TypeId UartBus::getType() const { return TypeId::UART; }

BusStatus UartBus::init() {
    if (!lockBus(busMutexTimeoutMs)) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return BusStatus::TIMEOUT;
    }

    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        unlockBus();
        return BusStatus::OK;
    }

    ESP_LOGI(TAG, "Init start (port=%d, TX=%d, RX=%d, baud=%" PRIu32 ")", config.port, config.txPin, config.rxPin, config.baudRate);

    uart_config_t uartConfig = {};
    uartConfig.baud_rate = static_cast<int>(config.baudRate);
    uartConfig.data_bits = config.dataBits;
    uartConfig.parity = config.parity;
    uartConfig.stop_bits = config.stopBits;
    uartConfig.flow_ctrl = config.flowControl;
    uartConfig.source_clk = UART_SCLK_DEFAULT;

    esp_err_t ret = uart_param_config(config.port, &uartConfig);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        unlockBus();
        return BusStatus::ERROR;
    }

    ret = uart_set_pin(config.port, config.txPin, config.rxPin, config.rtsPin, config.ctsPin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        unlockBus();
        return BusStatus::ERROR;
    }

    ret = uart_driver_install(config.port, static_cast<int>(config.rxBufferSize), static_cast<int>(config.txBufferSize), 0, nullptr, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        unlockBus();
        return BusStatus::ERROR;
    }

    initialized = true;
    ESP_LOGI(TAG, "Init done");
    unlockBus();
    return BusStatus::OK;
}

BusStatus UartBus::deinit() {
    if (!lockBus(busMutexTimeoutMs)) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return BusStatus::TIMEOUT;
    }

    if (!initialized) {
        ESP_LOGW(TAG, "Not initialized");
        unlockBus();
        return BusStatus::NOT_INITIALIZED;
    }

    uart_flush(config.port);
    uart_driver_delete(config.port);

    initialized = false;
    deviceAdded = false;
    busy.store(false);

    ESP_LOGI(TAG, "Deinitialized");
    unlockBus();
    return BusStatus::OK;
}

BusStatus UartBus::addDevice(const DeviceDescriptor& descriptor) {
    if (!lockBus(busMutexTimeoutMs)) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return BusStatus::TIMEOUT;
    }

    if (!initialized) {
        ESP_LOGE(TAG, "Not initialized");
        unlockBus();
        return BusStatus::NOT_INITIALIZED;
    }

    if (descriptor.id != 0) {
        ESP_LOGE(TAG, "UART is point-to-point, device id must be 0 (got %u)", descriptor.id);
        unlockBus();
        return BusStatus::INVALID_ARGUMENT;
    }

    if (deviceAdded) {
        ESP_LOGE(TAG, "Device already added (UART supports only one device)");
        unlockBus();
        return BusStatus::INVALID_ARGUMENT;
    }

    deviceAdded = true;
    ESP_LOGI(TAG, "Device added (id=%u, config=%" PRIu32 ")", descriptor.id, descriptor.config);
    unlockBus();
    return BusStatus::OK;
}

BusStatus UartBus::transfer(const uint8_t* txData, size_t txSize, bool takeMutex) {
    if (!initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return BusStatus::NOT_INITIALIZED;
    }

    if (txData == nullptr || txSize == 0) {
        return BusStatus::INVALID_ARGUMENT;
    }

    if (takeMutex) {
        if (!lockBus(config.readTimeoutMs)) {
            ESP_LOGW(TAG, "Failed to acquire mutex");
            return BusStatus::TIMEOUT;
        }
    }

    busy.store(true);

    int written = uart_write_bytes(config.port, txData, txSize);
    if (written < 0) {
        ESP_LOGE(TAG, "uart_write_bytes failed");
        busy.store(false);
        if (takeMutex) {
            unlockBus();
        }
        return BusStatus::ERROR;
    }

    esp_err_t ret = uart_wait_tx_done(config.port, pdMS_TO_TICKS(config.readTimeoutMs));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_wait_tx_done failed: %s", esp_err_to_name(ret));
        busy.store(false);
        if (takeMutex) {
            unlockBus();
        }
        return BusStatus::TIMEOUT;
    }

    busy.store(false);

    if (takeMutex) {
        unlockBus();
    }

    return BusStatus::OK;
}

BusStatus UartBus::transferAndReceive(const uint8_t* txData, size_t txSize, uint8_t* rxData, size_t* rxSize, bool takeMutex) {
    if (!initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return BusStatus::NOT_INITIALIZED;
    }

    if (rxData == nullptr || rxSize == nullptr || *rxSize == 0) {
        return BusStatus::INVALID_ARGUMENT;
    }

    if (takeMutex) {
        if (!lockBus(config.readTimeoutMs)) {
            ESP_LOGW(TAG, "Failed to acquire mutex");
            return BusStatus::TIMEOUT;
        }
    }

    busy.store(true);

    if (txData != nullptr && txSize > 0) {
        int written = uart_write_bytes(config.port, txData, txSize);
        if (written < 0) {
            ESP_LOGE(TAG, "uart_write_bytes failed");
            busy.store(false);
            if (takeMutex) {
                unlockBus();
            }
            return BusStatus::ERROR;
        }

        esp_err_t ret = uart_wait_tx_done(config.port, pdMS_TO_TICKS(config.readTimeoutMs));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "uart_wait_tx_done failed: %s", esp_err_to_name(ret));
            busy.store(false);
            if (takeMutex) {
                unlockBus();
            }
            return BusStatus::TIMEOUT;
        }
    }

    int bytesRead = uart_read_bytes(config.port, rxData, *rxSize, pdMS_TO_TICKS(config.readTimeoutMs));
    if (bytesRead < 0) {
        ESP_LOGE(TAG, "uart_read_bytes failed");
        *rxSize = 0;
        busy.store(false);
        if (takeMutex) {
            unlockBus();
        }
        return BusStatus::ERROR;
    }

    *rxSize = static_cast<size_t>(bytesRead);
    busy.store(false);

    if (takeMutex) {
        unlockBus();
    }

    return BusStatus::OK;
}

bool UartBus::isBusy() const { return busy.load(); }

BusStatus UartBus::abort() {
    if (!lockBus(busMutexTimeoutMs)) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return BusStatus::TIMEOUT;
    }

    if (!initialized) {
        unlockBus();
        return BusStatus::NOT_INITIALIZED;
    }

    uart_flush(config.port);
    busy.store(false);

    ESP_LOGI(TAG, "Transfer aborted, buffers flushed");
    unlockBus();
    return BusStatus::OK;
}

bool UartBus::lockBus(uint32_t timeoutMs) {
    if (busMutex == nullptr) {
        return false;
    }

    TickType_t ticks;
    if (timeoutMs == portMAX_DELAY) {
        ticks = portMAX_DELAY;
    } else {
        ticks = pdMS_TO_TICKS(timeoutMs);
    }

    BaseType_t takeResult = xSemaphoreTake(busMutex, ticks);
    return (takeResult == pdTRUE);
}

void UartBus::unlockBus() {
    if (busMutex != nullptr) {
        xSemaphoreGive(busMutex);
    }
}

}  // namespace UART
}  // namespace DC_Motor_Controller_Firmware
