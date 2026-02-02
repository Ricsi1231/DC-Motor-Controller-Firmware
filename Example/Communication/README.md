# MotorCommHandler Example -- USB Command Interface

This example demonstrates how to use the `MotorCommHandler` module to handle USB-based communication for motor control. The `MotorCommHandler` creates and owns its USB interface internally.

---

## What It Does

- Creates a `MotorCommHandler` that internally initializes USB CDC communication
- Parses custom serial protocol commands
- Receives position targets and PID gain updates
- Sends motor status responses and acknowledgments
- Responds to `GET_PID` requests with current PID values

---

## Usage

```cpp
static MotorCommHandler motorCommHandler;

extern "C" void app_main() {
    esp_err_t errorStatus = motorCommHandler.init();  // Initializes USB internally

    while (true) {
        motorCommHandler.process();
        // Handle targets, PID updates, etc.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

## Supported Commands

| Command                    | Description                        |
|----------------------------|------------------------------------|
| `SET_DEG:<value>`          | Set target position in degrees     |
| `SET_PID:<kp>,<ki>,<kd>`  | Set PID parameters                 |
| `GET_PID`                  | Request current PID values         |
| `STOP`                     | Emergency stop                     |
| `ENABLE`                   | Enable motor                       |
| `DISABLE`                  | Disable motor                      |
| `RESET_ALL`                | Reset all state                    |

---

## Use Case

This example is intended for integration testing with external tools (e.g., LabVIEW or serial terminal) to verify motor control protocol handling via USB.

---

## Dependencies

- `MotorCommHandler`, `USB`, `Interfaces` components
- TinyUSB (enabled via ESP-IDF)
