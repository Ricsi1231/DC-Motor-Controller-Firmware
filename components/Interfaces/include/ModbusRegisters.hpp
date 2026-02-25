/**
 * @file ModbusRegisters.hpp
 * @brief Complete Modbus RTU register map for the DC motor controller.
 *
 * Single source of truth for all Modbus register addresses, command codes,
 * and protocol constants. Used by both the Modbus slave implementation
 * and any master/tooling.
 *
 * Register layout follows standard Modbus conventions:
 *  - Holding Registers (FC03/FC06/FC16) — read/write configuration & commands
 *  - Input Registers   (FC04)           — read-only status & telemetry
 *  - Coils             (FC01/FC05)      — read/write booleans
 *  - Discrete Inputs   (FC02)           — read-only booleans
 *
 * Multi-register values use big-endian word order (high word first):
 *  - float32 → 2 consecutive registers (IEEE 754)
 *  - uint32  → 2 consecutive registers
 *  - int64   → 4 consecutive registers
 *
 * Command flow: write parameters to holding registers → write CommandCode
 * to CMD_EXECUTE → poll ACK_RESULT for validation feedback, then poll
 * STATUS_CMD_STATUS or DI_COMMAND_BUSY for execution progress.
 */

#pragma once

#include <cstdint>

namespace DC_Motor_Controller_Firmware {
namespace Modbus {

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/**
 * @enum ControlMode
 * @brief Active control mode reported by the motor controller.
 */
enum class ControlMode : uint16_t {
    IDLE = 0,             ///< No active command
    POSITION = 1,         ///< Move to target degrees
    SPEED = 2,            ///< Continuous speed control
    SPEED_TIMED = 3,      ///< Speed control with duration
    POSITION_REPEAT = 4,  ///< Position control with N repetitions
};

/**
 * @enum CommandCode
 * @brief Codes written to CMD_EXECUTE to trigger actions.
 */
enum class CommandCode : uint16_t {
    NOP = 0x0000,                   ///< No operation
    SET_POSITION = 0x0001,          ///< Move to target position
    SET_SPEED = 0x0002,             ///< Run at target speed continuously
    SET_SPEED_TIMED = 0x0003,       ///< Run at speed for duration then stop
    SET_POSITION_REPEAT = 0x0004,   ///< Move to position N times (origin-return between)
    STOP = 0x0005,                  ///< Controlled stop
    COAST = 0x0006,                 ///< Free-spin stop
    BRAKE = 0x0007,                 ///< Active braking stop
    APPLY_PID_CONFIG = 0x0010,      ///< Apply staged PID parameters
    APPLY_FF_CONFIG = 0x0011,       ///< Apply staged feed-forward parameters
    APPLY_CTRL_CONFIG = 0x0012,     ///< Apply staged motor controller config
    APPLY_SETTLE_CONFIG = 0x0013,   ///< Apply staged settle detection config
    APPLY_STALL_CONFIG = 0x0014,    ///< Apply staged stall detection config
    APPLY_PROFILE_CONFIG = 0x0015,  ///< Apply staged motion profile config
    APPLY_SOFT_LIMITS = 0x0016,     ///< Apply staged soft limits
    APPLY_GUARD_CONFIG = 0x0017,    ///< Apply staged motion guard config
    APPLY_ENCODER_CONFIG = 0x0018,  ///< Apply staged encoder config
    APPLY_DEVICE_CONFIG = 0x0019,   ///< Apply staged Modbus/UART config
    APPLY_LED_COMMAND = 0x0020,     ///< Execute LED command with staged values
};

/**
 * @enum CommandStatus
 * @brief Status of the last executed command.
 */
enum class CommandStatus : uint16_t {
    IDLE = 0,       ///< No command active
    EXECUTING = 1,  ///< Command in progress
    COMPLETED = 2,  ///< Command finished successfully
    ERROR = 3,      ///< Command failed
    TIMEOUT = 4,    ///< Command timed out
    STALLED = 5,    ///< Motor stalled during command
};

/**
 * @enum AckResult
 * @brief Application-level acknowledgment result for the last write/command.
 */
enum class AckResult : uint16_t {
    SUCCESS = 0,          ///< Command accepted and validated
    INVALID_COMMAND = 1,  ///< Unknown or unsupported CommandCode
    INVALID_VALUE = 2,    ///< Parameter value is invalid (NaN, Inf, etc.)
    OUT_OF_RANGE = 3,     ///< Parameter outside allowed bounds
    BUSY = 4,             ///< Another command is still executing
    NOT_ENABLED = 5,      ///< Required subsystem not enabled (e.g. motor task)
};

/**
 * @enum AckErrorDetail
 * @brief Detailed reason for a non-SUCCESS AckResult.
 */
enum class AckErrorDetail : uint16_t {
    NONE = 0,                 ///< No error
    UNKNOWN_COMMAND = 1,      ///< CommandCode not recognized
    VALUE_TOO_LOW = 2,        ///< Value below minimum bound
    VALUE_TOO_HIGH = 3,       ///< Value above maximum bound
    NAN_OR_INF = 4,           ///< Float value is NaN or Inf
    MOTOR_DISABLED = 5,       ///< Motor task not running
    COMMAND_IN_PROGRESS = 6,  ///< Previous command still executing
    INVALID_STATE = 7,        ///< Device in wrong state for this command
};

/**
 * @enum MotorDirectionCode
 * @brief Motor rotation direction for command parameters.
 */
enum class MotorDirectionCode : uint16_t {
    LEFT = 0,   ///< Counter-clockwise
    RIGHT = 1,  ///< Clockwise
};

/**
 * @enum EncoderDirectionCode
 * @brief Encoder counting direction as reported by hardware.
 */
enum class EncoderDirectionCode : uint16_t {
    RIGHT = 0,  ///< Clockwise
    LEFT = 1,   ///< Counter-clockwise
};

/**
 * @enum MotionProfileTypeCode
 * @brief Motion profile type selection.
 */
enum class MotionProfileTypeCode : uint16_t {
    TRAPEZOID = 0,  ///< Trapezoidal velocity profile
    S_CURVE = 1,    ///< S-curve velocity profile
};

/**
 * @enum ParityCode
 * @brief UART parity setting for Modbus communication.
 */
enum class ParityCode : uint16_t {
    NONE = 0,  ///< No parity
    EVEN = 1,  ///< Even parity
    ODD = 2,   ///< Odd parity
};

/**
 * @enum LedPresetCode
 * @brief Predefined LED color presets (mirrors PresetColor).
 */
enum class LedPresetCode : uint16_t {
    RED = 0,      ///< Pure red
    GREEN = 1,    ///< Pure green
    BLUE = 2,     ///< Pure blue
    YELLOW = 3,   ///< Red + green
    CYAN = 4,     ///< Green + blue
    MAGENTA = 5,  ///< Red + blue
    WHITE = 6,    ///< All channels on
};

// ---------------------------------------------------------------------------
// Device constants
// ---------------------------------------------------------------------------

static constexpr uint16_t kFirmwareVersionMajor = 1;   ///< Firmware major version
static constexpr uint16_t kFirmwareVersionMinor = 2;   ///< Firmware minor version
static constexpr uint16_t kFirmwareVersionPatch = 0;   ///< Firmware patch version
static constexpr uint16_t kDeviceTypeId = 0x4443;      ///< "DC" in ASCII
static constexpr uint16_t kHardwareRevision = 0x0100;  ///< HW rev 1.0

// ---------------------------------------------------------------------------
// Modbus address limits
// ---------------------------------------------------------------------------

static constexpr uint16_t kMinModbusAddress = 1;      ///< Minimum valid slave address
static constexpr uint16_t kMaxModbusAddress = 247;    ///< Maximum valid slave address
static constexpr uint16_t kDefaultModbusAddress = 1;  ///< Factory default slave address

// ---------------------------------------------------------------------------
// Register size helpers (number of 16-bit registers per value)
// ---------------------------------------------------------------------------

static constexpr uint16_t kFloat32RegisterCount = 2;  ///< IEEE 754 float32 → 2 regs
static constexpr uint16_t kUint32RegisterCount = 2;   ///< uint32_t → 2 regs
static constexpr uint16_t kInt64RegisterCount = 4;    ///< int64_t → 4 regs

// ---------------------------------------------------------------------------
// Holding Registers (FC03 / FC06 / FC16) — Read/Write
// ---------------------------------------------------------------------------

/**
 * @namespace HoldingRegister
 * @brief Addresses for Modbus holding registers (FC03/FC06/FC16).
 *
 * Groups are spaced by 0x0020 (32 registers) to allow future expansion.
 * Config registers use a stage-then-apply pattern: write parameters first,
 * then write the corresponding APPLY_* CommandCode to CMD_EXECUTE.
 */
namespace HoldingRegister {

/// @brief Command & Control registers (0x0000–0x001F).
namespace Command {
static constexpr uint16_t EXECUTE = 0x0000;           ///< Write CommandCode to trigger
static constexpr uint16_t TARGET_POSITION = 0x0001;   ///< float32: target position (deg)
static constexpr uint16_t TARGET_SPEED = 0x0003;      ///< uint16: target speed (0–100%)
static constexpr uint16_t DIRECTION = 0x0004;         ///< uint16: MotorDirectionCode
static constexpr uint16_t DURATION_MS = 0x0005;       ///< uint32: duration for timed speed (ms)
static constexpr uint16_t REPETITION_COUNT = 0x0007;  ///< uint16: repetitions for position-repeat
static constexpr uint16_t RAMP_TIME_MS = 0x0008;      ///< uint32: speed ramp time (ms)
}  // namespace Command

/// @brief PID configuration registers (0x0020–0x003F).
namespace PID {
static constexpr uint16_t KP = 0x0020;                ///< float32: proportional gain
static constexpr uint16_t KI = 0x0022;                ///< float32: integral gain
static constexpr uint16_t KD = 0x0024;                ///< float32: derivative gain
static constexpr uint16_t MAX_OUTPUT = 0x0026;        ///< float32: max PID output
static constexpr uint16_t MAX_INTEGRAL = 0x0028;      ///< float32: anti-windup integral limit
static constexpr uint16_t ERROR_EPSILON = 0x002A;     ///< float32: error dead zone
static constexpr uint16_t SPEED_EPSILON = 0x002C;     ///< float32: speed dead zone
static constexpr uint16_t ERROR_TIMEOUT = 0x002E;     ///< float32: error timeout (sec)
static constexpr uint16_t STUCK_TIMEOUT = 0x0030;     ///< float32: stuck timeout (sec)
static constexpr uint16_t DERIVATIVE_ALPHA = 0x0032;  ///< float32: derivative filter alpha
}  // namespace PID

/// @brief Feed-forward configuration registers (0x0040–0x005F).
namespace FeedForward {
static constexpr uint16_t KPOS = 0x0040;  ///< float32: position feed-forward gain
static constexpr uint16_t KVEL = 0x0042;  ///< float32: velocity feed-forward gain
}  // namespace FeedForward

/// @brief Motor controller configuration registers (0x0060–0x007F).
namespace Controller {
static constexpr uint16_t MIN_SPEED = 0x0060;          ///< float32: minimum speed (%)
static constexpr uint16_t MAX_SPEED = 0x0062;          ///< float32: maximum speed (%)
static constexpr uint16_t MIN_ERROR_TO_MOVE = 0x0064;  ///< float32: minimum error to start moving (deg)
static constexpr uint16_t COUNTS_PER_REV = 0x0066;     ///< uint16: encoder counts per revolution
static constexpr uint16_t UPDATE_HZ = 0x0067;          ///< uint16: control loop update rate (Hz)
static constexpr uint16_t MOTION_TIMEOUT = 0x0068;     ///< uint32: motion timeout (ms)
}  // namespace Controller

/// @brief Settle detection configuration registers (0x0080–0x009F).
namespace Settle {
static constexpr uint16_t POS_TOL = 0x0080;      ///< float32: position tolerance (deg)
static constexpr uint16_t VEL_TOL = 0x0082;      ///< float32: velocity tolerance (deg/s)
static constexpr uint16_t COUNT_LIMIT = 0x0084;  ///< uint16: consecutive samples to declare settled
}  // namespace Settle

/// @brief Stall detection configuration registers (0x00A0–0x00BF).
namespace Stall {
static constexpr uint16_t POS_EPSILON = 0x00A0;        ///< float32: position epsilon (deg)
static constexpr uint16_t STUCK_COUNT_LIMIT = 0x00A2;  ///< uint16: stuck count threshold
static constexpr uint16_t PID_WARMUP_LIMIT = 0x00A3;   ///< uint16: PID warmup iterations
static constexpr uint16_t MIN_ERROR_TO_MOVE = 0x00A4;  ///< float32: minimum error to consider moving (deg)
}  // namespace Stall

/// @brief Motion profile configuration registers (0x00C0–0x00DF).
namespace Profile {
static constexpr uint16_t TYPE = 0x00C0;         ///< uint16: MotionProfileTypeCode
static constexpr uint16_t ACCEL_LIMIT = 0x00C1;  ///< float32: acceleration limit (%/s)
static constexpr uint16_t JERK_LIMIT = 0x00C3;   ///< float32: jerk limit (%/s^2)
static constexpr uint16_t MAX_SPEED = 0x00C5;    ///< float32: max profiled speed (%)
}  // namespace Profile

/// @brief Soft limits configuration registers (0x00E0–0x00FF).
namespace SoftLimit {
static constexpr uint16_t MIN = 0x00E0;  ///< float32: minimum angle (deg)
static constexpr uint16_t MAX = 0x00E2;  ///< float32: maximum angle (deg)
}  // namespace SoftLimit

/// @brief Motion guard configuration registers (0x0100–0x011F).
namespace Guard {
static constexpr uint16_t DRIFT_DEADBAND = 0x0100;    ///< float32: drift deadband (deg)
static constexpr uint16_t DRIFT_HYSTERESIS = 0x0102;  ///< float32: drift hysteresis (deg)
}  // namespace Guard

/// @brief Encoder configuration registers (0x0120–0x013F).
namespace Encoder {
static constexpr uint16_t GEAR_RATIO = 0x0120;        ///< float32: gear ratio
static constexpr uint16_t GLITCH_FILTER_NS = 0x0122;  ///< uint32: glitch filter period (ns)
}  // namespace Encoder

/// @brief Device / Modbus configuration registers (0x0140–0x015F).
namespace Device {
static constexpr uint16_t MODBUS_ADDRESS = 0x0140;  ///< uint16: slave address (1–247)
static constexpr uint16_t BAUD_RATE = 0x0141;       ///< uint32: UART baud rate
static constexpr uint16_t PARITY = 0x0143;          ///< uint16: ParityCode
}  // namespace Device

/// @brief RGB LED control registers (0x0160–0x017F).
namespace Led {
static constexpr uint16_t COLOR_PRESET = 0x0160;      ///< uint16: LedPresetCode
static constexpr uint16_t RED = 0x0161;               ///< uint16: red channel (0–255)
static constexpr uint16_t GREEN = 0x0162;             ///< uint16: green channel (0–255)
static constexpr uint16_t BLUE = 0x0163;              ///< uint16: blue channel (0–255)
static constexpr uint16_t BRIGHTNESS = 0x0164;        ///< uint16: brightness (0–100%)
static constexpr uint16_t FADE_DURATION_MS = 0x0165;  ///< uint16: fade duration (ms)
static constexpr uint16_t BLINK_DELAY_MS = 0x0166;    ///< uint16: blink delay (ms)
static constexpr uint16_t BLINK_COUNT = 0x0167;       ///< uint16: number of blinks
}  // namespace Led

}  // namespace HoldingRegister

// ---------------------------------------------------------------------------
// Input Registers (FC04) — Read-Only
// ---------------------------------------------------------------------------

/**
 * @namespace InputRegister
 * @brief Addresses for Modbus input registers (FC04, read-only).
 *
 * These registers expose real-time status, telemetry, and parameter
 * readback. The PID readback group lets a master verify that staged
 * configuration was actually applied.
 */
namespace InputRegister {

/// @brief Device information registers (0x0000–0x001F).
namespace Info {
static constexpr uint16_t FW_VERSION_MAJOR = 0x0000;  ///< uint16: firmware major version
static constexpr uint16_t FW_VERSION_MINOR = 0x0001;  ///< uint16: firmware minor version
static constexpr uint16_t FW_VERSION_PATCH = 0x0002;  ///< uint16: firmware patch version
static constexpr uint16_t DEVICE_TYPE_ID = 0x0003;    ///< uint16: device type (0x4443 = "DC")
static constexpr uint16_t HW_REVISION = 0x0004;       ///< uint16: hardware revision
}  // namespace Info

/// @brief Motor status registers (0x0020–0x003F).
namespace Status {
static constexpr uint16_t TARGET_POS = 0x0020;       ///< float32: target position (deg)
static constexpr uint16_t CURRENT_POS = 0x0022;      ///< float32: current position (deg)
static constexpr uint16_t ERROR = 0x0024;            ///< float32: position error (deg)
static constexpr uint16_t VELOCITY = 0x0026;         ///< float32: angular velocity (deg/s)
static constexpr uint16_t PID_OUTPUT = 0x0028;       ///< float32: PID output
static constexpr uint16_t STUCK_COUNT = 0x002A;      ///< uint16: stuck/stall counter
static constexpr uint16_t MOTOR_SPEED = 0x002B;      ///< uint16: motor speed (0–100%)
static constexpr uint16_t MOTOR_DIRECTION = 0x002C;  ///< uint16: MotorDirectionCode
static constexpr uint16_t CONTROL_MODE = 0x002D;     ///< uint16: ControlMode
static constexpr uint16_t CMD_STATUS = 0x002E;       ///< uint16: CommandStatus
}  // namespace Status

/// @brief Encoder telemetry registers (0x0040–0x005F).
namespace Encoder {
static constexpr uint16_t POSITION_TICKS = 0x0040;  ///< int32: raw encoder ticks
static constexpr uint16_t POSITION_DEG = 0x0042;    ///< float32: position (deg)
static constexpr uint16_t RPM_FILTERED = 0x0044;    ///< float32: filtered RPM
static constexpr uint16_t RPM_RAW = 0x0046;         ///< float32: raw RPM
static constexpr uint16_t RPM_ROUNDED = 0x0048;     ///< int16: rounded RPM
static constexpr uint16_t DIRECTION = 0x0049;       ///< uint16: EncoderDirectionCode
static constexpr uint16_t TOTAL_COUNTS = 0x004A;    ///< int64: total accumulated counts (4 regs)
static constexpr uint16_t OVERFLOWS = 0x004E;       ///< uint32: counter overflow count
static constexpr uint16_t MISSED_EDGES = 0x0050;    ///< uint32: missed edge count
}  // namespace Encoder

/// @brief PID readback registers (0x0060–0x007F).
namespace PID {
static constexpr uint16_t KP = 0x0060;               ///< float32: applied Kp
static constexpr uint16_t KI = 0x0062;               ///< float32: applied Ki
static constexpr uint16_t KD = 0x0064;               ///< float32: applied Kd
static constexpr uint16_t LAST_ERROR = 0x0066;       ///< float32: last PID error
static constexpr uint16_t LAST_DERIVATIVE = 0x0068;  ///< float32: last derivative term
static constexpr uint16_t OUTPUT = 0x006A;           ///< float32: last PID output
}  // namespace PID

/// @brief ADC telemetry registers (0x0080–0x009F) — future hardware.
namespace ADC {
static constexpr uint16_t TEMPERATURE = 0x0080;  ///< float32: board temperature (C)
static constexpr uint16_t BUS_VOLTAGE = 0x0082;  ///< float32: bus voltage (V)
static constexpr uint16_t RAW_TEMP = 0x0084;     ///< uint16: raw ADC counts (temperature)
static constexpr uint16_t RAW_VOLTAGE = 0x0085;  ///< uint16: raw ADC counts (voltage)
}  // namespace ADC

/// @brief Command acknowledgment registers (0x00A0–0x00BF).
namespace Ack {
static constexpr uint16_t LAST_CMD = 0x00A0;        ///< uint16: echo of last CommandCode received
static constexpr uint16_t RESULT = 0x00A1;          ///< uint16: AckResult code
static constexpr uint16_t ERROR_DETAIL = 0x00A2;    ///< uint16: AckErrorDetail code
static constexpr uint16_t ERROR_REGISTER = 0x00A3;  ///< uint16: address of register that failed validation
static constexpr uint16_t TIMESTAMP_MS = 0x00A4;    ///< uint32: uptime (ms) when last ACK was generated
}  // namespace Ack

}  // namespace InputRegister

// ---------------------------------------------------------------------------
// Coils (FC01 / FC05) — Read/Write Booleans
// ---------------------------------------------------------------------------

/**
 * @namespace Coil
 * @brief Addresses for Modbus coils (FC01/FC05, read/write booleans).
 *
 * Write-once coils (ENCODER_RESET, PID_RESET, FAULT_CLEAR, SAVE_CONFIG,
 * RESTORE_DEFAULTS) auto-clear after the action completes.
 */
namespace Coil {

static constexpr uint16_t MOTOR_ENABLE = 0x0000;           ///< Enable/disable motor task
static constexpr uint16_t EMERGENCY_STOP = 0x0001;         ///< Write 1 to trigger e-stop
static constexpr uint16_t MOTION_PROFILE_ENABLE = 0x0002;  ///< Enable/disable motion profile
static constexpr uint16_t SOFT_LIMIT_ENFORCE = 0x0003;     ///< Enable/disable soft limits
static constexpr uint16_t ENCODER_INVERT = 0x0004;         ///< Invert encoder direction
static constexpr uint16_t ENCODER_RESET = 0x0005;          ///< Write 1 to zero encoder position
static constexpr uint16_t PID_RESET = 0x0006;              ///< Write 1 to reset PID state
static constexpr uint16_t FAULT_CLEAR = 0x0007;            ///< Write 1 to clear fault flag
static constexpr uint16_t LED_ON = 0x0008;                 ///< LED on/off
static constexpr uint16_t SAVE_CONFIG = 0x0009;            ///< Write 1 to persist config to NVS
static constexpr uint16_t RESTORE_DEFAULTS = 0x000A;       ///< Write 1 to restore factory defaults

}  // namespace Coil

// ---------------------------------------------------------------------------
// Discrete Inputs (FC02) — Read-Only Booleans
// ---------------------------------------------------------------------------

/**
 * @namespace DiscreteInput
 * @brief Addresses for Modbus discrete inputs (FC02, read-only booleans).
 */
namespace DiscreteInput {

static constexpr uint16_t MOTION_DONE = 0x0000;        ///< Motion complete and settled
static constexpr uint16_t MOTOR_RUNNING = 0x0001;      ///< Motor speed > 0
static constexpr uint16_t FAULT_ACTIVE = 0x0002;       ///< DRV8876 fault pin asserted
static constexpr uint16_t ENCODER_STALLED = 0x0003;    ///< Encoder reports stalled
static constexpr uint16_t ENCODER_NOISY = 0x0004;      ///< Encoder reports noisy signal
static constexpr uint16_t ENCODER_SATURATED = 0x0005;  ///< Encoder near counter saturation
static constexpr uint16_t PID_SETTLED = 0x0006;        ///< PID reports settled
static constexpr uint16_t SOFT_LIMIT_HIT = 0x0007;     ///< Soft limit currently active
static constexpr uint16_t LED_IS_ON = 0x0008;          ///< LED currently on
static constexpr uint16_t COMMAND_BUSY = 0x0009;       ///< A command is currently executing

}  // namespace DiscreteInput

}  // namespace Modbus
}  // namespace DC_Motor_Controller_Firmware
