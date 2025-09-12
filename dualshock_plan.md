You are absolutely right. For joystick control, a less discrete motor management system is essential. I will refactor the control packet and motor logic to handle continuous joystick input.

Here is the updated detailed plan, incorporating the refactoring for less discrete motor control:

## DualShock 3 Integration Plan

This document outlines the steps to integrate DualShock 3 controller input for controlling an ESP32-CAM FPV system, including LED toggling and motor movement with continuous speed control, while retaining existing keyboard controls.

### Phase 0: New Library for DualShock

To keep the codebase organized, all DualShock 3 related logic will be encapsulated in a new library within `gs/src/`.

1.  **Create `gs/src/dualshock/` directory:** A new directory will be created at `gs/src/dualshock/`.
2.  **Create `gs/src/dualshock/Dualshock3.h` and `gs/src/dualshock/Dualshock3.cpp`:** These files will define and implement a `Dualshock3` class.
3.  **Update `gs/Makefile`:** The `gs/Makefile` will be modified to include the new `Dualshock3.cpp` file in the build process.
4.  **`Dualshock3` Class Implementation:**
    *   The `Dualshock3` class will manage the joystick device (`/dev/input/js0`), handle its connection/disconnection, and read `js_event`s.
    *   It will process button events (e.g., button 0 for flash) and axis events (Axis 0 for X-axis, Axis 1 for Y-axis).
    *   It will apply a configurable `JOYSTICK_DEADZONE` (e.g., 0.15f) to filter out minor joystick movements and scale the active axis values to a range of -100 to 100 for `joystick_x` and `joystick_y` representation.
    *   It will run its input reading logic in a dedicated thread to avoid blocking the main application loop.
    *   **New:** It will include a `std::function<void(uint8_t command, int8_t joystick_x, int8_t joystick_y)> m_control_callback;` member.
    *   **New:** It will provide a `setControlCallback(std::function<void(uint8_t command, int8_t joystick_x, int8_t joystick_y)> callback)` method.
    *   **New:** Inside `Dualshock3::joystick_input_thread_proc()`, when a button or axis event is processed, if `m_control_callback` is set, it will immediately invoke the callback with the appropriate command and joystick values.
    *   **New:** The `Dualshock3` class will manage the "last sent packet" state internally to avoid flooding, including `last_control_packet_sent_tp` and `last_sent_control_packet`.

### Phase 1: Packet Structure Modification

The `Ground2Air_Control_Packet` will be modified to directly carry the scaled X and Y joystick values.

1.  **Modify `Ground2Air_Control_Packet` in `components/common/packets.h`:**
    *   Remove the previously planned `int8_t speed;` field.
    *   Add `int8_t joystick_x;` and `int8_t joystick_y;` fields to the `Ground2Air_Control_Packet` structure. These will represent the scaled joystick values (-100 to 100).
2.  **Define New Command in `gs/src/main.h`:**
    *   Add `#define CMD_JOYSTICK_MOVE 6` to `gs/src/main.h` to indicate that the `joystick_x` and `joystick_y` fields in the control packet are valid and should be used for continuous movement.

### Phase 2: DualShock 3 Input Integration in `gs/src/main.cpp`

The ground station application (`gs/src/main.cpp`) will be updated to use the new DualShock 3 library and send control packets with X/Y joystick values.

1.  **Include Headers:**
    *   `#include "dualshock/Dualshock3.h"` will be added.
    *   `#include <cmath>` will be included. (Removed `<atomic>` as it's no longer needed for shared state).
2.  **Global Variables:**
    *   An instance of the `Dualshock3` class, `s_dualshock3`, will be declared globally.
    *   **Removed:** `Clock::time_point last_control_packet_sent_tp` and `Ground2Air_Control_Packet last_sent_control_packet` will be removed from `main.cpp` as they are now managed within the `Dualshock3` class.
3.  **`main()` Function Update:**
    *   The `s_dualshock3` instance will be initialized, and its internal input reading thread will be started.
    *   **New:** In `main()`, after `s_dualshock3` is initialized, set its callback to a lambda function that constructs and sends the `Ground2Air_Control_Packet` using `s_comms.send()`. This lambda will also incorporate the logic for sending frequency and change detection, using the internal state of `Dualshock3`.
    *   Proper cleanup for the `s_dualshock3` instance will be ensured on application exit.
4.  **Removed:** The `handleDualshockInput()` function will no longer be needed, as the `Dualshock3` class will directly trigger packet sending via the callback.
5.  **Modify `handleKeyboardInput()` in `gs/src/main.cpp`:**
    *   This function will now *only* handle keyboard inputs for movement (W, A, S, D) and flash (F).
    *   It will be called *after* checking if DualShock is connected.
    *   If `s_dualshock3.isConnected()` is true, `handleKeyboardInput()` will skip processing movement/flash commands to avoid conflicts.
    *   If `s_dualshock3.isConnected()` is false, then `handleKeyboardInput()` will process keyboard inputs. For keyboard movement (W, A, S, D), it will send a `Ground2Air_Control_Packet` with `command = CMD_JOYSTICK_MOVE` and default `joystick_x`/`joystick_y` values (e.g., W -> `joystick_y = 50`, A -> `joystick_x = -50`, etc.). For 'F' key, it will send `CMD_FLASH`. The packet sending logic will also include the frequency/change check.

### Phase 3: Update `components/air/main.cpp` to Handle New Packet

The air unit's main logic will be updated to interpret the new control packet format with X/Y joystick values.

1.  **Modify `send_i2c_command` in `components/air/i2c.h` and `components/air/i2c.cpp`:**
    *   The function signature will be changed to `void send_i2c_command(uint8_t command, int8_t val1 = 0, int8_t val2 = 0);`. This allows sending 1, 2, or 3 bytes depending on the command.
    *   The implementation in `components/air/i2c.cpp` will be updated to send the `command` byte, and then `val1` and `val2` if the command requires them (e.g., `CMD_JOYSTICK_MOVE`).
2.  **Update `handle_ground2air_control_packet` in `components/air/main.cpp`:**
    *   This function's signature will be updated to reflect the new `Ground2Air_Control_Packet` structure (including `joystick_x` and `joystick_y`).
    *   For `CMD_FLASH`, the existing LED toggling logic will remain.
    *   For `CMD_JOYSTICK_MOVE`, it will call `send_i2c_command(CMD_JOYSTICK_MOVE, src.joystick_x, src.joystick_y)`.
    *   For `CMD_NONE`, it will call `send_i2c_command(CMD_NONE, 0, 0)` to stop all movement.

### Phase 4: Update `esp32_aux/src/main.cpp` to Handle Continuous Motor Control

The auxiliary ESP32 unit will be updated to receive and act upon the continuous joystick control.

1.  **I2C Data Reception:** The `i2c_slave_read_buffer` logic in `esp32_aux/src/main.cpp` will be modified to read up to 3 bytes. It will first read the command byte. If the command is `CMD_JOYSTICK_MOVE`, it will then read the `joystick_x` and `joystick_y` bytes.
2.  **Refactor `Motor` Class:**
    *   A new public method `void Motor::set_movement(int8_t joystick_x, int8_t joystick_y);` will be added to the `Motor` class.
    *   This method will contain the core logic to translate the `joystick_x` and `joystick_y` values (ranging from -100 to 100) into appropriate PWM signals for the individual motors (e.g., left and right motor speeds). This will enable continuous, less discrete movement.
    *   The existing `forward()`, `backward()`, `right()`, `left()`, `stop()` methods can be retained, but the primary control for joystick input will be `set_movement`.
3.  **Update `switch` statement:**
    *   If `CMD_JOYSTICK_MOVE` is received, it will call `motor.set_movement(received_joystick_x, received_joystick_y)`.
    *   If `CMD_NONE` is received, it will call `motor.stop()`.
    *   For `I2C_CMD_FLASH`, the `esp32_aux` unit will continue to log "FLASH command received - no action on motors".

### Documentation

A new Markdown file, `doc/dualshock_integration.md`, will be created to document this plan and the implementation details.

This revised plan fully addresses the need for less discrete motor control, maintains keyboard functionality, and organizes the DualShock logic into a dedicated library, with a focus on immediate, callback-driven event handling.
