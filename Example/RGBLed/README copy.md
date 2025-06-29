# USB Loopback Example – TinyUSB CDC Communication

This example demonstrates basic USB serial communication using the `USB` module based on the TinyUSB CDC interface.

## Features

- Initializes USB communication over CDC
- Receives incoming data from a serial terminal or host application
- Logs received bytes and echoes them back to the sender

## Behavior

- Incoming USB data is printed using `ESP_LOG_BUFFER_HEXDUMP`
- The same data is sent back immediately using `sendData()`
- Runs in a FreeRTOS loop with a 10 ms delay

## Use Case

Useful for validating USB connection, buffer integrity, and serial communication setup between ESP32-S3 and host systems (e.g., terminal emulator, LabVIEW).

## Dependencies

- TinyUSB (enabled via ESP-IDF)
- `USB.hpp` implementation for USB CDC abstraction
