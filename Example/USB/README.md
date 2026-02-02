# USB Loopback Example -- TinyUSB CDC Communication

This example demonstrates basic USB serial communication using the `USB` class based on the TinyUSB CDC interface. The `USB` class implements the `IComm` interface.

---

## What It Does

- Initializes USB CDC communication
- Receives incoming data from a serial terminal or host application
- Logs received bytes using `ESP_LOG_BUFFER_HEXDUMP`
- Echoes them back to the sender using `sendData()`
- Runs in a FreeRTOS loop with a 10 ms delay

---

## Use Case

Useful for validating USB connection, buffer integrity, and serial communication setup between ESP32-S3 and host systems (e.g., terminal emulator, LabVIEW).

---

## Dependencies

- `USB` component
- TinyUSB (enabled via ESP-IDF)
