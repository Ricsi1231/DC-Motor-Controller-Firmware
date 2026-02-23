/**
 * @file UartBus.hpp
 * @brief UART bus implementation of the IPeripheryBus interface.
 *
 * Provides a concrete UART transport using the ESP-IDF UART driver.
 * UART is point-to-point, so only a single device (id=0) is supported.
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "IPeripheryBus.hpp"

namespace DC_Motor_Controller_Firmware {
namespace UART {

/**
 * @struct UartConfig
 * @brief Configuration for the UartBus.
 */
struct UartConfig {
    uart_port_t port;                   ///< UART port number (e.g. UART_NUM_1)
    gpio_num_t txPin;                   ///< TX GPIO pin
    gpio_num_t rxPin;                   ///< RX GPIO pin
    gpio_num_t rtsPin;                  ///< RTS GPIO pin (GPIO_NUM_NC to disable)
    gpio_num_t ctsPin;                  ///< CTS GPIO pin (GPIO_NUM_NC to disable)
    uint32_t baudRate;                  ///< Baud rate
    uart_word_length_t dataBits;        ///< Data bits (e.g. UART_DATA_8_BITS)
    uart_parity_t parity;               ///< Parity mode (e.g. UART_PARITY_DISABLE)
    uart_stop_bits_t stopBits;          ///< Stop bits (e.g. UART_STOP_BITS_1)
    uart_hw_flowcontrol_t flowControl;  ///< Hardware flow control mode
    size_t rxBufferSize;                ///< RX ring buffer size in bytes
    size_t txBufferSize;                ///< TX ring buffer size in bytes (0 = blocking writes)
    uint32_t readTimeoutMs;             ///< Default read timeout in milliseconds
};

/**
 * @class UartBus
 * @brief Concrete IPeripheryBus implementation for UART.
 */
class UartBus : public PeripheryBus::IPeripheryBus {
  public:
    /**
     * @brief Constructor.
     * @param config UART configuration structure.
     */
    explicit UartBus(const UartConfig& config);

    /**
     * @brief Destructor. Calls deinit() if still initialized.
     */
    ~UartBus() override;

    /**
     * @brief Deleted copy constructor.
     */
    UartBus(const UartBus&) = delete;

    /**
     * @brief Deleted copy assignment operator.
     */
    UartBus& operator=(const UartBus&) = delete;

    /**
     * @brief Move constructor.
     * @param other Source instance to move from.
     */
    UartBus(UartBus&& other) noexcept;

    /**
     * @brief Move assignment operator.
     * @param other Source instance to move from.
     * @return Reference to the assigned UartBus instance.
     */
    UartBus& operator=(UartBus&& other) noexcept;

    /**
     * @brief Get the bus type identifier.
     * @return TypeId::UART.
     */
    PeripheryBus::TypeId getType() const override;

    /**
     * @brief Initialize the UART peripheral.
     * @return BusStatus::OK on success, else error status.
     */
    PeripheryBus::BusStatus init() override;

    /**
     * @brief De-initialize the UART peripheral and release resources.
     * @return BusStatus::OK on success, else error status.
     */
    PeripheryBus::BusStatus deinit() override;

    /**
     * @brief Register a device on the bus. UART supports only one device (id=0).
     * @param descriptor Device descriptor (id must be 0).
     * @return BusStatus::OK on success, BusStatus::INVALID_ARGUMENT otherwise.
     */
    PeripheryBus::BusStatus addDevice(const PeripheryBus::DeviceDescriptor& descriptor) override;

    /**
     * @brief Transmit data over UART.
     * @param txData Pointer to the data buffer to send.
     * @param txSize Number of bytes to send.
     * @param takeMutex If true, acquire the bus mutex before the operation.
     * @return BusStatus::OK on success, else error status.
     */
    PeripheryBus::BusStatus transfer(const uint8_t* txData, size_t txSize, bool takeMutex = true) override;

    /**
     * @brief Transmit and/or receive data over UART.
     * @param txData Pointer to the data buffer to send (may be nullptr for receive-only).
     * @param txSize Number of bytes to send.
     * @param rxData Pointer to the buffer to store received data.
     * @param rxSize On input: buffer capacity; on output: actual bytes received.
     * @param takeMutex If true, acquire the bus mutex before the operation.
     * @return BusStatus::OK on success, else error status.
     */
    PeripheryBus::BusStatus transferAndReceive(const uint8_t* txData, size_t txSize, uint8_t* rxData, size_t* rxSize, bool takeMutex = true) override;

    /**
     * @brief Check if the bus is currently busy.
     * @return true if busy, false otherwise.
     */
    bool isBusy() const override;

    /**
     * @brief Abort any ongoing transfer and flush UART buffers.
     * @return BusStatus::OK on success, else error status.
     */
    PeripheryBus::BusStatus abort() override;

  private:
    UartConfig config;                     ///< UART configuration
    bool initialized = false;              ///< Initialization flag
    bool deviceAdded = false;              ///< Device registration flag
    std::atomic<bool> busy{false};         ///< Atomic busy flag for lock-free status reads
    SemaphoreHandle_t busMutex = nullptr;  ///< FreeRTOS mutex for thread-safe bus access

    static constexpr char TAG[] = "UartBus";  ///< Logging tag
};

}  // namespace UART
}  // namespace DC_Motor_Controller_Firmware
