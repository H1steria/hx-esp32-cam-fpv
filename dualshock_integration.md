# DualShock 3 Integration Documentation

This document summarizes the integration of DualShock 3 controller input for controlling an ESP32-CAM FPV system, including LED toggling and motor movement with continuous speed control, while retaining existing keyboard controls.

## Phase 0: New Library for DualShock

All DualShock 3 related logic is encapsulated in a new library within `gs/src/dualshock/`.

-   **Directory and Files:** The `gs/src/dualshock/` directory, `Dualshock3.h`, and `Dualshock3.cpp` were created.
-   **Makefile Update:** `gs/Makefile` was updated to include `Dualshock3.cpp` in the build process.
-   **`Dualshock3` Class Implementation:**
    -   Manages the joystick device (`/dev/input/js0`), handles connection/disconnection, and reads `js_event`s.
    -   Processes button events (button 0 for flash) and axis events (Axis 0 for X-axis, Axis 1 for Y-axis).
    -   Applies a configurable `JOYSTICK_DEADZONE` (0.15f) and scales active axis values to a range of -100 to 100 for `joystick_x` and `joystick_y`.
    -   Runs input reading logic in a dedicated thread.
    -   Includes a `std::function<void(uint8_t command, int8_t joystick_x, int8_t joystick_y)> m_control_callback;` member and `setControlCallback` method.
    -   Invokes the callback immediately when a button or axis event is processed.
    -   Manages "last sent packet" state internally (`last_control_packet_sent_tp` and `last_sent_control_packet`) to avoid flooding.

## Phase 1: Packet Structure Modification

The `Ground2Air_Control_Packet` was modified to directly carry scaled X and Y joystick values.

-   **`Ground2Air_Control_Packet` in `components/common/packets.h`:**
    -   The `int8_t speed;` field was removed.
    -   `int8_t joystick_x;` and `int8_t joystick_y;` fields were added.
-   **New Command in `gs/src/main.h`:**
    -   `#define CMD_JOYSTICK_MOVE 6` was added to `gs/src/main.h`.

## Phase 2: DualShock 3 Input Integration in `gs/src/main.cpp`

The ground station application (`gs/src/main.cpp`) was updated to use the new DualShock 3 library and send control packets with X/Y joystick values.

-   **Headers:** `#include "dualshock/Dualshock3.h"` and `#include <cmath>` were added.
-   **Global Variables:** An instance of `Dualshock3`, `s_dualshock3`, was declared globally. `last_control_packet_sent_tp` and `last_sent_control_packet` were removed as global variables.
-   **`main()` Function Update:**
    -   `s_dualshock3` is initialized and its input reading thread is started.
    -   Its callback is set to a lambda function that constructs and sends the `Ground2Air_Control_Packet` using `s_comms.send()`, incorporating sending frequency and change detection.
    -   Proper cleanup for `s_dualshock3` is ensured on application exit.
-   **`handleDualshockInput()`:** This function was removed.
-   **`handleKeyboardInput()` Modification:**
    -   This function now only handles keyboard inputs for movement (W, A, S, D) and flash (F) if DualShock is not connected.
    -   For keyboard movement, it sends a `Ground2Air_Control_Packet` with `command = CMD_JOYSTICK_MOVE` and default `joystick_x`/`joystick_y` values. For 'F' key, it sends `CMD_FLASH`.

## Phase 3: Update `components/air/main.cpp` to Handle New Packet

The air unit's main logic was updated to interpret the new control packet format with X/Y joystick values.

-   **`send_i2c_command` in `components/air/i2c.h` and `components/air/i2c.cpp`:**
    -   The function signature was changed to `void send_i2c_command(uint8_t command, int8_t val1 = 0, int8_t val2 = 0);`.
    -   The implementation in `components/air/i2c.cpp` was updated to send the `command` byte, and then `val1` and `val2` if the command is `I2C_CMD_JOYSTICK_MOVE`.
-   **`components/air/main.h`:** `#define I2C_CMD_NONE 0` and `#define I2C_CMD_JOYSTICK_MOVE 6` were added.
-   **`handle_ground2air_control_packet` in `components/air/main.cpp`:**
    -   This function now only handles `CMD_FLASH`, `CMD_JOYSTICK_MOVE`, and `CMD_NONE`.
    -   For `CMD_FLASH`, existing LED toggling logic remains.
    -   For `CMD_JOYSTICK_MOVE`, it calls `send_i2c_command(CMD_JOYSTICK_MOVE, src.joystick_x, src.joystick_y)`.
    -   For `CMD_NONE`, it calls `send_i2c_command(CMD_NONE, 0, 0)` to stop all movement.
    -   Discrete movement commands (`I2C_CMD_FORWARD`, `I2C_CMD_BACKWARD`, `I2C_CMD_RIGHT`, `I2C_CMD_LEFT`) were removed from this function.

## Phase 4: Update `esp32_aux/src/main.cpp` to Handle Continuous Motor Control

The auxiliary ESP32 unit was updated to receive and act upon the continuous joystick control.

-   **I2C Data Reception in `esp32_aux/src/main.cpp`:**
    -   `I2C_CMD_NONE` and `I2C_CMD_JOYSTICK_MOVE` were defined.
    -   The `i2c_slave_read_buffer` logic was modified to read up to 3 bytes and parse them based on the command.
-   **Refactor `Motor` Class:**
    -   A new public method `void Motor::set_movement(int8_t joystick_x, int8_t joystick_y);` was added to `esp32_aux/src/motor.h`.
    -   The implementation in `esp32_aux/src/motor.cpp` includes LEDC setup for PWM control and translates `joystick_x` and `joystick_y` values into appropriate PWM signals for individual motors, enabling continuous movement.
    -   The discrete movement methods (`forward()`, `backward()`, `right()`, `left()`) were removed from `esp32_aux/src/motor.cpp`.
-   **Update `switch` statement in `esp32_aux/src/main.cpp`:**
    -   If `I2C_CMD_JOYSTICK_MOVE` is received, it calls `motor.set_movement(received_joystick_x, received_joystick_y)`.
    -   If `I2C_CMD_NONE` is received, it calls `motor.stop()`.
    -   For `I2C_CMD_FLASH`, the `esp32_aux` unit continues to log "FLASH command received - no action on motors".
