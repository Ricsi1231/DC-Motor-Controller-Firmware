/**
 * @file USB.hpp
 * @brief USB CDC-ACM interface for serial communication using TinyUSB.
 *
 * This class provides methods to send and receive data via USB using
 * the TinyUSB stack (CDC ACM class). Supports formatted output, status flags,
 * and internal receive buffer handling.
 */

#pragma once

#include "tinyusb_cdc_acm.h"

namespace DC_Motor_Controller_Firmware {
namespace USB {

/**
 * @class USB
 * @brief Manages TinyUSB CDC-ACM communication (USB serial).
 */
class USB {
    /**
     * @brief Struct to hold a received USB message.
     */
    typedef struct {
        uint8_t buffer[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1]; /**< Internal RX buffer */
        size_t bufferSize;                                 /**< Size of received data */
        uint8_t itf;                                       /**< Interface ID */
    } usbMessage;

  public:
    /**
     * @brief Constructor for USB class.
     */
    USB();

    /**
     * @brief Destructor.
     */
    ~USB();

    /**
     * @brief Initialize TinyUSB CDC interface.
     *
     * @return esp_err_t ESP_OK if successful
     */
    esp_err_t init();

    /**
     * @brief Send raw data over USB.
     *
     * @param data Pointer to data buffer
     * @param dataSize Number of bytes to send
     * @return esp_err_t ESP_OK if successful
     */
    esp_err_t sendData(uint8_t* data, size_t dataSize);

    /**
     * @brief Receive data from USB.
     *
     * @param data Pointer to output buffer
     * @param rxBufferSize Pointer to store received byte count
     * @return esp_err_t ESP_OK if successful
     */
    esp_err_t receiveData(uint8_t* data, size_t* rxBufferSize);

    /**
     * @brief Send a formatted string over USB (printf-style).
     *
     * @param fmt Format string
     * @param ... Variable arguments
     * @return esp_err_t ESP_OK if successful
     */
    esp_err_t sendFormattedString(const char* fmt, ...);

    /**
     * @brief Send a null-terminated string over USB.
     *
     * @param str C-style string to send
     * @return esp_err_t ESP_OK if successful
     */
    esp_err_t sendString(const char* str);

    /**
     * @brief Check if USB serial is connected and open.
     *
     * @return true if connected
     * @return false if not connected
     */
    bool usbIsConnected() const;

    /**
     * @brief Check if new data has been received.
     *
     * @return true if new data is present
     * @return false if buffer is empty
     */
    bool newDataIsReceived() const;

    /**
     * @brief Clear the internal RX buffer.
     */
    void flushRxBuffer();

  private:
    /**
     * @brief TinyUSB callback for CDC events (connection, line state, etc.).
     *
     * @param itf Interface number
     * @param event Event type
     */
    static void usbCallback(int itf, cdcacm_event_t* event);

    /**
     * @brief Called when the USB serial port is opened/closed by host.
     *
     * @param itf Interface number
     * @param event Event type
     */
    static void serialPortState(int itf, cdcacm_event_t* event);

    static bool serialIsOpen;    ///< Flag: is USB serial open
    bool isInitialized = false;  ///< Initialization status

    static usbMessage message;                                   ///< Last received message
    static uint8_t rxBuffer[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];  ///< RX buffer
    static size_t rxSize;                                        ///< Size of data in RX buffer

    static const char* TAG;  ///< Log tag

    static tinyusb_cdcacm_itf_t USB_INTERFACE_PORT;  ///< USB CDC interface used
};

}  // namespace USB
}  // namespace DC_Motor_Controller_Firmware
