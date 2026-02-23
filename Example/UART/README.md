# UART Example -- Bus Communication via IPeripheryBus

This example demonstrates how to use the `UartBus` class to send and receive data over UART using the `IPeripheryBus` interface. The `UartBus` class provides a thread-safe, mutex-protected UART transport.

---

## What It Does

- Initializes the UART peripheral with full configuration (baud rate, data bits, parity, etc.)
- Registers a single point-to-point device on the bus
- Periodically transmits a message using `transfer()`
- Receives incoming data using `transferAndReceive()` and logs it as a hex dump
- Echoes received data back to the sender

---

## Pin Configuration

| Signal | GPIO Pin |
|--------|----------|
| TX     | GPIO17   |
| RX     | GPIO18   |
| RTS    | Not used |
| CTS    | Not used |

---

## Configuration

```cpp
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
uart.init();
```

---

## Use Case

Useful for communicating with external peripherals (sensors, actuators, other MCUs) over UART through the generic `IPeripheryBus` abstraction. Wire TX to RX for a loopback self-test.

---

## Dependencies

- `UART` component
- `Interfaces` component (`IPeripheryBus`)
- ESP-IDF UART driver
