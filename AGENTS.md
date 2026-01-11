# AGENTS.md - Coding Guidelines for AI Agents

This document provides essential information for AI coding agents working on the Fire TV BLE Remote project.

## Project Overview

This is an ESP-IDF (v5.5.2) embedded C project for ESP32-C3 that implements a Bluetooth Low Energy (BLE) HID remote control for Fire TV. The project uses FreeRTOS and ESP32's BLE stack to create a wireless remote with physical buttons mapped to HID keyboard and consumer control reports.

**Target Hardware:** ESP32-C3 Super Mini  
**Framework:** ESP-IDF 5.5.2  
**Language:** C (embedded systems)

## Build, Test, and Lint Commands

### Building the Project

```bash
# Full build (from project root)
idf.py build

# Clean build
idf.py fullclean
idf.py build

# Set target (if not already configured)
idf.py set-target esp32c3

# Build configuration menu
idf.py menuconfig
```

### Flashing and Monitoring

```bash
# Flash to device
idf.py -p /dev/ttyACM0 flash

# Monitor serial output
idf.py -p /dev/ttyACM0 monitor

# Flash and monitor in one command
idf.py -p /dev/ttyACM0 flash monitor

# Exit monitor: Ctrl+]
```

### Testing

```bash
# Run pytest-based tests (requires pytest-embedded-idf)
pytest pytest_hello_world.py

# Run specific test
pytest pytest_hello_world.py::test_hello_world -v

# Run with specific target
pytest pytest_hello_world.py --target esp32c3
```

### Linting and Static Analysis

```bash
# clangd is configured for IDE integration
# Configuration in .clangd removes -f* and -m* flags

# Manual static analysis (if cppcheck is available)
cppcheck --enable=all --inconclusive main/

# Check compilation database
cat build/compile_commands.json
```

## Code Style Guidelines

### File Organization

```
fire-remote/
├── main/               # Main application code
│   ├── main.c         # Entry point and core logic
│   └── CMakeLists.txt # Component build configuration
├── CMakeLists.txt     # Project-level build config
├── sdkconfig          # ESP-IDF configuration (generated)
└── pytest_*.py        # Test files
```

### Imports and Headers

- **Standard libraries first**, then FreeRTOS, then ESP-IDF specific headers
- Group by category with blank lines between groups:
  1. Standard C library (`stdio.h`, `stdlib.h`, `string.h`)
  2. FreeRTOS headers (`freertos/FreeRTOS.h`, `freertos/task.h`)
  3. ESP system headers (`esp_system.h`, `esp_log.h`)
  4. ESP-IDF component headers (BT, NVS, driver, etc.)

```c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "driver/gpio.h"
```

### Naming Conventions

- **Macros/Constants:** `UPPER_SNAKE_CASE` (e.g., `HID_TAG`, `BTN_UP`, `GPIO_NUM_2`)
- **Functions:** `lower_snake_case` (e.g., `send_consumer_control`, `init_gpio`)
- **Variables:** `lower_snake_case` (e.g., `hid_conn_id`, `is_connected`, `last_state`)
- **Static variables:** prefix with `static` and use descriptive names
- **Global variables:** minimize usage; use `static` for file-scope globals

### Code Formatting

- **Indentation:** 4 spaces (no tabs)
- **Line length:** Keep under 100 characters when reasonable
- **Braces:** K&R style (opening brace on same line for functions and control structures)
- **Spacing:** Space after keywords (`if`, `while`, `for`), no space before `(`

```c
void button_task(void *pvParameters) {
    uint8_t last_state[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    
    while (1) {
        if (gpio_get_level(BTN_UP) == 0 && last_state[0] == 1) {
            ESP_LOGI(HID_TAG, "UP pressed");
            send_keyboard_key(0x52);
            last_state[0] = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

### Types and Type Safety

- Use ESP-IDF types: `uint8_t`, `uint16_t`, `uint32_t`, `esp_err_t`
- Always check return values from ESP-IDF functions:
  - Use `ESP_ERROR_CHECK()` for critical operations
  - Manual checking for non-critical operations
- Prefer explicit types over implicit (`uint8_t` vs `unsigned char`)

```c
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
}
ESP_ERROR_CHECK(ret);
```

### Error Handling

- **Critical errors:** Use `ESP_ERROR_CHECK()` - causes abort on failure
- **Recoverable errors:** Check and handle explicitly
- **Logging levels:** 
  - `ESP_LOGE()` - Errors
  - `ESP_LOGW()` - Warnings
  - `ESP_LOGI()` - Info (normal operation events)
  - `ESP_LOGD()` - Debug (verbose details)

```c
if (!is_connected || hid_gatts_if == 0) {
    ESP_LOGW(HID_TAG, "Cannot send report: device not connected");
    return;
}
```

### FreeRTOS Conventions

- Task functions: `void task_name(void *pvParameters)`
- Stack size: Typical values 2048-4096 bytes
- Priority: 1-5 (higher = more priority)
- Delays: Always use `vTaskDelay(pdMS_TO_TICKS(ms))` for millisecond delays

```c
xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
```

### GPIO and Hardware

- Use `GPIO_NUM_x` constants for pin definitions
- Configure GPIO with `gpio_config_t` structs
- Pull-ups enabled for active-low buttons (common for ESP32)

### Comments

- Use `//` for single-line comments
- Document non-obvious logic, hardware mappings, protocol details
- Include references to specifications when implementing protocols

## CMake Configuration

When adding new source files, update `main/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "main.c" "new_file.c"
                      INCLUDE_DIRS "."
                      REQUIRES nvs_flash bt driver)
```

## Important Notes

- **Minimal build enabled:** Only required components are included
- **No dynamic memory in ISRs:** Use queues to communicate with tasks
- **Thread safety:** Use FreeRTOS mutexes/semaphores for shared resources
- **Always check connection state** before sending BLE notifications
- **Port configuration:** Default is `/dev/ttyACM0` (see `.vscode/settings.json`)
