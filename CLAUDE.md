# DC Motor Controller Firmware - Coding Standards & Conventions

This document serves as a reference for maintaining consistent coding standards when working on the DC Motor Controller Firmware project.

## Project Overview

ESP32-based DC motor controller firmware using ESP-IDF framework with modern C++ practices.

**Current Branch:** `5-fuzzy-pid-controller`
**Main Branch:** `main`

---

## 0. Git Workflow Rules - CRITICAL

**NEVER push changes to the remote repository without explicit user permission.**

- Always complete all work and let the user review changes first
- User will explicitly ask to push when ready
- Use `git status` and `git diff` to show changes for user review
- Create commits only when user requests
- DO NOT use `git push` unless the user explicitly asks for it

---

## 1. Code Formatting

**Base Style:** Google C++ Style Guide (configured in `.clang-format`)

- **Indentation:** 4 spaces (never tabs)
- **Column Limit:** 160 characters per line
- **Brace Style:** Attach (opening brace on same line)
- **Pointer/Reference:** Left-aligned (`int* ptr`, `Type& ref`)
- **Access Modifiers:** Indented -2 spaces relative to class
- **Maximum Empty Lines:** 1 between code sections

**Formatting Script:** Use `./scripts/format.sh` to auto-format code before committing.

---

## 2. Naming Conventions

| Element | Convention | Examples |
|---------|-----------|----------|
| **Classes** | PascalCase | `PIDController`, `MotorCommHandler`, `I2CMasterManager` |
| **Structs** | PascalCase | `PidConfig`, `MessageHeader`, `SetDegMessage` |
| **Enums** | `enum class`, PascalCase | `CommandType`, `ResponseStatus`, `RobotMode` |
| **Enum Values** | UPPER_CASE | `SET_DEG`, `SET_PID`, `SUCCESS`, `ERROR` |
| **Methods** | camelCase | `init()`, `setSpeed()`, `getMotorSpeed()` |
| **Variables** | camelCase | `motorSpeed`, `targetDegrees`, `ledCount` |
| **Constants** | UPPER_CASE | `MIN_MSG_SIZE`, `MAX_PORTS`, `DEFAULT_TIMEOUT_MS` |
| **Class Constants** | `static constexpr` | `static constexpr char TAG[] = "Component";` |
| **Directories** | PascalCase | `MotorCommHandler/`, `PIDController/`, `USBProtocol/` |
| **Files** | PascalCase + extension | `MotorCommHandler.hpp`, `PIDController.cpp` |

**IMPORTANT:** Never use C-style snake_case naming (e.g., `motor_comm_handler`, `pid_controller`) for directories or files.

**Variable Naming - Code Readability:**
- **ALWAYS use full, descriptive variable names** - Never use abbreviations to save characters
- **BAD:** `bwp`, `odr`, `cfg`, `btn`, `idx`, `tmp`, `buf`, `ret`
- **GOOD:** `bandwidth`, `outputDataRate`, `config`, `button`, `index`, `temporary`, `buffer`, `status`
- **Exception:** Well-known acronyms like `i2c`, `spi`, `usb`, `gpio` are acceptable
- Code readability is more important than saving characters
- Other developers must understand variable purpose immediately without context

**Return Value Naming:**
- **NEVER use `ret` for return values** - Always use descriptive names
- **GOOD options:** `status`, `result`, `error`, or context-specific names
- **Examples:**
  - `esp_err_t status = i2cDevice->write(...);`
  - `esp_err_t initResult = initialize();`
  - `esp_err_t writeStatus = writeRegister(...);`
  - `bool validationResult = validate(...);`

**Static constexpr Members (TAG):**
- **ALWAYS declare static constexpr TAG as a private member in the class header**
- **DO NOT define TAG in the .cpp file** - the declaration in the header is sufficient
- **Pattern:**
  ```cpp
  // Header file (.hpp)
  class Component {
  private:
      static constexpr char TAG[] = "ComponentName";  ///< Logging tag
  };

  // Implementation file (.cpp) - NO TAG definition needed
  Component::Component() {
      // Constructor implementation - can use TAG directly
      ESP_LOGI(TAG, "Component initialized");
  }
  ```
- Since C++17, inline static constexpr members don't need separate definition
- This is the pattern used in AS5600 and BMI088 reference implementations

---

## 3. Header File Documentation (ALWAYS REQUIRED)

### File Header
```cpp
/**
 * @file FileName.hpp
 * @brief Brief one-line description of file purpose.
 *
 * Detailed multi-line description explaining the component's
 * functionality, features, and usage context.
 */

#pragma once

// Includes organized by type:
// 1. System/library includes
// 2. ESP-IDF includes
// 3. Project includes

namespace DC_Motor_Controller_Firmware::ComponentName {

// Content here

}  // namespace DC_Motor_Controller_Firmware::ComponentName
```

### Class Documentation
```cpp
/**
 * @class ClassName
 * @brief Brief description of class purpose.
 *
 * Detailed description of functionality, features, and usage patterns.
 * Include any important usage notes or constraints.
 *
 * DO NOT include usage examples in header file documentation.
 * Usage examples belong in separate example folders/files.
 */
class ClassName {
public:
    /**
     * @brief Brief description of what method does.
     *
     * Optional detailed description if needed.
     *
     * @param paramName Description of parameter
     * @param anotherParam Description with type info if helpful
     * @return ReturnType Description of return value
     */
    ReturnType methodName(ParamType paramName);

private:
    float member;  ///< Description of member variable
    bool flag = false;  ///< Description with default value noted
};
```

### Struct Documentation
```cpp
/**
 * @struct StructName
 * @brief Brief description of struct purpose.
 */
struct StructName {
    float value;  ///< Description of member
    uint8_t count;  ///< Description with units if applicable
};
```

### Enum Documentation
```cpp
/**
 * @enum EnumName
 * @brief Brief description of enumeration purpose.
 */
enum class EnumName : uint8_t {
    VALUE_ONE = 0x01, /**< Description of this value */
    VALUE_TWO = 0x02  /**< Description of this value */
};
```

---

## 4. File Organization

### Header Files (.hpp)
1. File documentation block
2. `#pragma once`
3. System/library includes
4. ESP-IDF includes
5. Project includes
6. Namespace opening
7. Forward declarations (if needed)
8. Enums and type aliases
9. Config structs
10. Main class declaration
11. Namespace closing

### Implementation Files (.cpp)
1. Include own header first
2. System/library includes
3. ESP-IDF includes
4. Project includes
5. `using namespace` declarations (sparingly)
6. Static TAG definition: `static constexpr char TAG[] = "ComponentName";`
7. Constructor/destructor implementations
8. Public method implementations
9. Private method implementations
10. Static/ISR functions at end

---

## 5. Common Patterns

### Configuration Pattern
```cpp
struct ComponentConfig {
    float parameter1;  ///< Description
    uint32_t parameter2 = 1000;  ///< Description with default
};

class Component {
public:
    explicit Component(const ComponentConfig& config);
    esp_err_t init();  ///< Two-phase initialization
};
```

### Thread Safety
- Use `std::atomic` for simple flags
- Use mutexes for shared resource access
- Use spinlocks (`portMUX_TYPE`) for ISR-safe critical sections
- Mark ISR functions with `IRAM_ATTR`
```cpp
std::atomic<bool> flag{false};
SemaphoreHandle_t mutex = nullptr;
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
```

### Error Handling
```cpp
esp_err_t Component::init() {
    // Validate input
    if (invalidCondition) {
        ESP_LOGE(TAG, "Error description with context");
        return ESP_ERR_INVALID_ARG;
    }

    // Perform operation
    esp_err_t ret = espFunction();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Operation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized = true;
    return ESP_OK;
}
```

### Logging
Always define a TAG and use ESP-IDF logging:
```cpp
static constexpr char TAG[] = "ComponentName";

ESP_LOGI(TAG, "Informational message");
ESP_LOGW(TAG, "Warning message");
ESP_LOGE(TAG, "Error message");
ESP_LOGD(TAG, "Debug message");
```

---

## 6. Modern C++ Features to Use

- `nullptr` instead of NULL
- `enum class` for type-safe enumerations
- `constexpr` for compile-time constants
- `explicit` constructors to prevent implicit conversions
- `std::atomic` for thread-safe simple types
- `std::function` for callbacks
- `std::vector` for dynamic arrays
- Range-based for loops
- Move semantics where appropriate
- `noexcept` on move operations
- Delete copy constructors when appropriate

### Bit Operations and Boolean Conversions

**NEVER** write unreadable bit-check expressions like:
```cpp
// BAD - unreadable, hard to debug
return (status & StatusBits::MAGNET_DETECTED) != 0;

// BAD - multiple bit checks in sequence, unreadable
status.magnetDetected = (statusReg & StatusBits::MAGNET_DETECTED) != 0;
status.magnetTooWeak = (statusReg & StatusBits::MAGNET_TOO_WEAK) != 0;
status.magnetTooStrong = (statusReg & StatusBits::MAGNET_TOO_STRONG) != 0;
```

**ALWAYS** use clear, explicit boolean conversions with intermediate variables:
```cpp
// GOOD - readable and clear
bool isMagnetDetected = (status & StatusBits::MAGNET_DETECTED) != 0;
return isMagnetDetected;

// GOOD - each bit check separated into its own line with clear variable
bool magnetDetected = (statusReg & StatusBits::MAGNET_DETECTED) != 0;
bool magnetTooWeak = (statusReg & StatusBits::MAGNET_TOO_WEAK) != 0;
bool magnetTooStrong = (statusReg & StatusBits::MAGNET_TOO_STRONG) != 0;

status.magnetDetected = magnetDetected;
status.magnetTooWeak = magnetTooWeak;
status.magnetTooStrong = magnetTooStrong;
```

Or use helper functions for repeated bit operations:
```cpp
// GOOD - encapsulated logic
bool checkBit(uint8_t value, uint8_t mask) {
    return (value & mask) != 0;
}
return checkBit(status, StatusBits::MAGNET_DETECTED);
```

**Key principle:** Never perform bit operations and assignments in the same line. Always use intermediate variables for clarity and debuggability.

### Function Call Returns

**NEVER** return function calls directly in a single line:
```cpp
// BAD - hard to debug, no variable name for clarity
return read16Bit(AS5600Regs::MAGNITUDE_H, AS5600Regs::MAGNITUDE_L, magnitude);
```

**ALWAYS** store function results in named variables before returning:
```cpp
// GOOD - clear, debuggable, readable
esp_err_t result = read16Bit(AS5600Regs::MAGNITUDE_H, AS5600Regs::MAGNITUDE_L, magnitude);
return result;
```

**Benefits:**
- Easy to set breakpoints and inspect values during debugging
- Variable name adds context about what the function does
- Makes code review easier
- Consistent with error handling patterns

**Key principle:** Code readability is paramount. Break complex expressions into named intermediate variables or helper functions.

---

## 7. Namespace Conventions

Use nested namespaces with pattern: `ProjectName::ComponentName`

```cpp
namespace DC_Motor_Controller_Firmware::PID {
    // Component code
}

namespace DC_Motor_Controller_Firmware::USB {
    // Component code
}
```

Existing namespaces:
- `DC_Motor_Controller_Firmware::PID`
- `DC_Motor_Controller_Firmware::DRV8876`
- `DC_Motor_Controller_Firmware::USB`
- `DC_Motor_Controller_Firmware::Control`
- `DC_Motor_Controller_Firmware::Encoder`
- `DC_Motor_Controller_Firmware::RGB`
- `DC_Motor_Controller_Firmware::Communication`
- `DC_Motor_Controller_Firmware::Logic`

---

## 8. Binary Protocol Structures

For network/USB communication structs:
```cpp
#pragma pack(push, 1)
/**
 * @struct MessageName
 * @brief Brief description of message.
 */
struct MessageName {
    uint8_t field1;  ///< Description
    uint16_t field2;  ///< Description
};
#pragma pack(pop)
```

---

## 9. CMake Component Structure

Each component directory contains:
```
ComponentName/
├── CMakeLists.txt
├── include/
│   └── ComponentName.hpp
└── src/
    └── ComponentName.cpp
```

**Example:**
```
MotorCommHandler/
├── CMakeLists.txt
├── include/
│   └── MotorCommHandler.hpp
└── src/
    └── MotorCommHandler.cpp
```

**CMakeLists.txt:**
```cmake
idf_component_register(
    SRCS "src/ComponentName.cpp"
    INCLUDE_DIRS "include"
    REQUIRES [dependencies]
)
```

---

## 10. Pre-Commit Checklist

Before committing code:

1. Run `./scripts/format.sh` to auto-format code
2. Run `./scripts/cppcheck.sh` for static analysis
3. Build project: `idf.py build`
4. Verify all new header files have complete documentation:
   - File @file block
   - Class/struct @class/@struct blocks
   - All public methods documented with @brief, @param, @return
   - All member variables have inline comments (`///< description`)
   - DO NOT include usage examples in header files (examples belong in example folders)
5. Check logging uses appropriate TAG
6. Verify thread-safety considerations
7. Ensure error handling is present

**IMPORTANT - Git Commit Messages:**
- **NEVER** include AI-generated attribution (e.g., "Generated with Claude Code") in commit messages
- Commit messages should be professional and focus on the technical changes
- Use conventional commit format: `type(scope): description`
- Do not attribute code authorship to AI tools in commits

---

## 11. Code Quality Standards

- **Documentation:** All public APIs must have complete Doxygen documentation
- **Testing:** Verify functionality before committing
- **Error Handling:** All operations that can fail must check and log errors
- **Thread Safety:** Document and implement appropriate synchronization
- **Resource Management:** Use RAII pattern, ensure proper cleanup in destructors
- **Validation:** Validate input parameters, especially in public APIs
- **Logging:** Use appropriate log levels (ERROR, WARN, INFO, DEBUG)

---

## 12. Tools

- **Formatter:** `./scripts/format.sh` - Runs clang-format on all source files
- **Static Analysis:** `./scripts/cppcheck.sh` - Runs cppcheck for code quality
- **Build:** `idf.py build` - ESP-IDF build system
- **Clean:** `idf.py fullclean` - Clean all build artifacts

---

## Quick Reference Examples

### Minimal Header Template
```cpp
/**
 * @file NewComponent.hpp
 * @brief Brief description.
 *
 * Detailed description.
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace DC_Motor_Controller_Firmware::NewComponent {

/**
 * @class NewComponent
 * @brief Brief description.
 */
class NewComponent {
public:
    /**
     * @brief Constructor.
     */
    NewComponent();

    /**
     * @brief Initialize component.
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t init();

private:
    static constexpr char TAG[] = "NewComponent";
    bool initialized = false;  ///< Initialization status
};

}  // namespace DC_Motor_Controller_Firmware::NewComponent
```

---

**Last Updated:** 2025-11-17
**Maintained By:** Claude Code AI Assistant
