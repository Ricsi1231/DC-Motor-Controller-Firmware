# MotorCommHandler Example – USB Command Interface

This example demonstrates how to use the `MotorCommHandler` module to handle USB-based communication for motor control, including:

- Receiving position targets
- Receiving and responding to PID gain updates
- Sending acknowledgments or motor state responses

## Features

- Initializes USB CDC communication
- Parses custom serial protocol using `MotorCommHandler`
- Logs received target degrees and PID parameters
- Sends motor status responses or acknowledges repeated targets
- Responds to `GET_PID` requests with current PID values

## Behavior

- New target positions are processed and logged
- If the target is the same as the previous one, a "motor reached" message is sent
- PID parameters are stored and echoed back if requested

## Use Case

This example is intended for integration testing with external tools (e.g., LabVIEW or serial terminal) to verify motor control protocol handling via USB.
