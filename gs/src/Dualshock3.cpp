#include "Dualshock3.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <errno.h> // For errno
#include "main.h" // For CMD_FLASH, CMD_JOYSTICK_MOVE, CMD_NONE

Dualshock3::Dualshock3() :
    m_joystick_fd(-1),
    m_joystick_device("/dev/input/js0"),
    m_joystick_connected(false),
    m_running(false),
    m_last_sent_control_packet({}), // Initialize with default values
    m_last_control_packet_sent_tp(Clock::now()),
    m_last_axis_packet_sent_tp(Clock::now())
{
    m_last_sent_control_packet.command = CMD_NONE;
    m_last_sent_control_packet.joystick_x = 0;
    m_last_sent_control_packet.joystick_y = 0;
}

Dualshock3::~Dualshock3() {
    stop();
    closeJoystick();
}

void Dualshock3::send_control_packet(uint8_t command, int8_t joystick_x, int8_t joystick_y) {
    if (!m_joystick_connected.load() || !m_control_callback) {
        return;
    }

    // Check if the command or joystick values have changed significantly
    bool changed = false;
    if (command != m_last_sent_control_packet.command ||
        joystick_x != m_last_sent_control_packet.joystick_x ||
        joystick_y != m_last_sent_control_packet.joystick_y) {
        changed = true;
    }

    // If the command is CMD_NONE and previous was active, ensure it's sent
    if (command == CMD_NONE && (m_last_sent_control_packet.command != CMD_NONE || m_last_sent_control_packet.joystick_x != 0 || m_last_sent_control_packet.joystick_y != 0)) {
        changed = true; // Force send stop command
    }

    // Force send CMD_FLASH even if values haven't changed, as it's a momentary action
    if (command == CMD_FLASH) {
        changed = true;
    }

    // Apply minimum delay for axis packets if values are changing AND not returning to zero
    if (command == CMD_JOYSTICK_MOVE && changed && (joystick_x != 0 || joystick_y != 0)) {
        if (Clock::now() - m_last_axis_packet_sent_tp < AXIS_SEND_MIN_INTERVAL) {
            return; // Don't send if axis values changed too soon and not returning to zero
        }
    }

    if (changed) {
        std::cout << "Dualshock3: Sending control packet - Command: " << (int)command
                  << ", Joystick X: " << (int)joystick_x
                  << ", Joystick Y: " << (int)joystick_y << std::endl;
        m_control_callback(command, joystick_x, joystick_y);
        m_last_sent_control_packet.command = command;
        m_last_sent_control_packet.joystick_x = joystick_x;
        m_last_sent_control_packet.joystick_y = joystick_y;
        m_last_control_packet_sent_tp = Clock::now(); // Update for all commands
        if (command == CMD_JOYSTICK_MOVE) {
            m_last_axis_packet_sent_tp = Clock::now(); // Update only for axis commands
        }
    }
}

void Dualshock3::start() {
    if (!m_running.load()) {
        m_running.store(true);
        m_input_thread = std::thread(&Dualshock3::joystick_input_thread_proc, this);
    }
}

void Dualshock3::stop() {
    if (m_running.load()) {
        m_running.store(false);
        if (m_input_thread.joinable()) {
            m_input_thread.join();
        }
    }
}

bool Dualshock3::openJoystick() {
    if (m_joystick_fd >= 0) {
        close(m_joystick_fd);
        m_joystick_fd = -1;
    }

    m_joystick_fd = open(m_joystick_device, O_RDONLY | O_NONBLOCK);
    if (m_joystick_fd >= 0) {
        std::cout << "Dualshock3: Controller initialized from " << m_joystick_device << std::endl;
        return true;
    } else {
        std::cerr << "Dualshock3: Error opening joystick device: " << strerror(errno) << std::endl;
        return false;
    }
}

void Dualshock3::closeJoystick() {
    if (m_joystick_fd >= 0) {
        close(m_joystick_fd);
        m_joystick_fd = -1;
        std::cout << "Dualshock3: Controller disconnected." << std::endl;
    }
}

bool Dualshock3::isJoystickDevicePresent() {
    return access(m_joystick_device, F_OK) == 0;
}

void Dualshock3::joystick_input_thread_proc() {
    std::cout << "Dualshock3: Joystick input thread started." << std::endl;
    bool was_connected = false;

    // Local variables to track current joystick state for immediate sending
    bool current_flash_button_state = false;
    int8_t current_joystick_x = 0;
    int8_t current_joystick_y = 0;

    // Local variables to store axis values for combined logging
    float last_raw_x = 0.0f;
    float last_processed_x = 0.0f;
    int8_t last_scaled_x = 0;
    float last_raw_y = 0.0f;
    float last_processed_y = 0.0f;
    int8_t last_scaled_y = 0;

    while (m_running.load()) {
        bool is_connected = isJoystickDevicePresent();

        if (is_connected && !was_connected) {
            if (openJoystick()) {
                m_joystick_connected.store(true);
                std::cout << "Dualshock3: Controller connected and reinitialized." << std::endl;
                // Reset local state on reconnection
                current_flash_button_state = false;
                current_joystick_x = 0;
                current_joystick_y = 0;
                last_raw_x = 0.0f;
                last_processed_x = 0.0f;
                last_scaled_x = 0;
                last_raw_y = 0.0f;
                last_processed_y = 0.0f;
                last_scaled_y = 0;
                send_control_packet(CMD_NONE, 0, 0); // Send stop command on connect
            } else {
                m_joystick_connected.store(false);
                std::cerr << "Dualshock3: Error reinitializing controller." << std::endl;
            }
        } else if (!is_connected && was_connected) {
            closeJoystick();
            m_joystick_connected.store(false);
            std::cout << "Dualshock3: Controller disconnected. Waiting for reconnection..." << std::endl;
            was_connected = false;
            // Ensure stop command is sent if disconnected
            send_control_packet(CMD_NONE, 0, 0);
        }

        was_connected = is_connected;

        if (m_joystick_connected.load() && m_joystick_fd >= 0) {
            js_event event;
            ssize_t bytes = read(m_joystick_fd, &event, sizeof(event));

            if (bytes == sizeof(event)) {
                event.type &= ~JS_EVENT_INIT; // Ignore init events

                switch (event.type) {
                    case JS_EVENT_BUTTON:
                        // Assuming button 0 is the flash button
                        if (event.number == 0) { 
                            current_flash_button_state = (event.value == 1);
                            std::cout << "Button " << (int)event.number << (event.value == 1 ? " pressed" : " released") << std::endl;
                            // Send flash command immediately
                            if (event.value == 1){
                                send_control_packet(CMD_FLASH, 0, 0); 
                            }
                        }
                        break;

                    case JS_EVENT_AXIS: {
                        float raw_axis_value = event.value / (float)JOYSTICK_MAX_VALUE;
                        float processed_axis_value = raw_axis_value;

                        // Scale to -100 to 100
                        int8_t scaled_value = static_cast<int8_t>(std::round(raw_axis_value * 100.0f));

                        if (event.number == 0) { // X-axis (left/right)
                            current_joystick_x = scaled_value;
                            last_raw_x = raw_axis_value;
                            last_processed_x = raw_axis_value; // Store raw for now, deadzone applied later
                            last_scaled_x = scaled_value;
                        } else if (event.number == 1) { // Y-axis (forward/backward)
                            current_joystick_y = scaled_value;
                            last_raw_y = raw_axis_value;
                            last_processed_y = raw_axis_value; // Store raw for now, deadzone applied later
                            last_scaled_y = scaled_value;
                        }
                        
                        // Apply combined deadzone after both axes have been updated
                        // Only set to 0 if both axes are within the deadzone threshold
                        if (std::abs(current_joystick_x) < (JOYSTICK_DEADZONE * 100) &&
                            std::abs(current_joystick_y) < (JOYSTICK_DEADZONE * 100)) {
                            current_joystick_x = 0;
                            current_joystick_y = 0;
                            last_processed_x = 0.0f; // Update processed values for logging
                            last_processed_y = 0.0f;
                            last_scaled_x = 0;
                            last_scaled_y = 0;
                        } else {
                            // If outside the combined deadzone, rescale values to maintain full range
                            // This part is tricky as it depends on how JOYSTICK_DEADZONE is defined.
                            // For simplicity, if not in deadzone, we use the scaled raw value.
                            // If a more complex rescaling is needed, it should be implemented here.
                            // For now, we'll just ensure the values are not 0 if they are outside the deadzone.
                            // The original code had a rescaling for individual axes, which is now removed.
                            // We will keep the scaled_value as is if it's outside the deadzone.
                            // The problem description implies that if one axis is at an extreme, the other should not be zeroed out.
                            // So, if current_joystick_x is not 0, it should be its scaled value.
                            // If current_joystick_y is not 0, it should be its scaled value.
                            // The `processed_axis_value` in the log should reflect the `current_joystick_x/y` scaled back to 0-1 range.
                            last_processed_x = current_joystick_x / 100.0f;
                            last_processed_y = current_joystick_y / 100.0f;
                            last_scaled_x = current_joystick_x;
                            last_scaled_y = current_joystick_y;
                        }

                        // Only send joystick move command if there's actual movement or if it's a transition to 0,0 from a non-zero state
                        // The send_control_packet function itself handles not sending if values haven't changed.
                        if (current_joystick_x != m_last_sent_control_packet.joystick_x ||
                            current_joystick_y != m_last_sent_control_packet.joystick_y) {
                            send_control_packet(CMD_JOYSTICK_MOVE, current_joystick_x, current_joystick_y);
                        }
                        break;
                    }
                }
                // Log combined X and Y axis values after processing any axis event
                if (event.type == JS_EVENT_AXIS) {
                    std::cout << "Axis X raw: " << last_raw_x << ", processed: " << last_processed_x << " (scaled: " << (int)last_scaled_x
                              << ") | Axis Y raw: " << last_raw_y << ", processed: " << last_processed_y << " (scaled: " << (int)last_scaled_y << ")" << std::endl;
                }
            } else if (bytes < 0 && errno != EAGAIN) {
                // Error reading (possible disconnection)
                closeJoystick();
                m_joystick_connected.store(false);
                std::cerr << "Dualshock3: Read error. Controller disconnected. Waiting for reconnection..." << std::endl;
                was_connected = false;
                send_control_packet(CMD_NONE, 0, 0); // Send stop command on error
            }
        }
        // Periodically send CMD_NONE if no active input, to ensure stop command is received
        // if (m_joystick_connected.load() && current_joystick_x == 0 && current_joystick_y == 0 && !current_flash_button_state) {
        //     send_control_packet(CMD_NONE, 0, 0);
        // }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Small pause
    }
    std::cout << "Dualshock3: Joystick input thread stopped." << std::endl;
}
