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
    m_last_control_packet_sent_tp(Clock::now())
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

    // Also send periodically to ensure commands are received, even if values don't change
    if (!changed && (Clock::now() - m_last_control_packet_sent_tp < PACKET_SEND_INTERVAL)) {
        return; // Don't send if not changed and too soon
    }

    // If the command is CMD_NONE and previous was active, ensure it's sent
    if (command == CMD_NONE && (m_last_sent_control_packet.command != CMD_NONE || m_last_sent_control_packet.joystick_x != 0 || m_last_sent_control_packet.joystick_y != 0)) {
        changed = true; // Force send stop command
    }

    if (changed) {
        m_control_callback(command, joystick_x, joystick_y);
        m_last_sent_control_packet.command = command;
        m_last_sent_control_packet.joystick_x = joystick_x;
        m_last_sent_control_packet.joystick_y = joystick_y;
        m_last_control_packet_sent_tp = Clock::now();
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
                            send_control_packet(CMD_FLASH, 0, 0); 
                        }
                        break;

                    case JS_EVENT_AXIS: {
                        float raw_axis_value = event.value / (float)JOYSTICK_MAX_VALUE;
                        float processed_axis_value = raw_axis_value;

                        // Apply deadzone and rescale
                        if (std::abs(processed_axis_value) < JOYSTICK_DEADZONE) {
                            processed_axis_value = 0.0f;
                        } else {
                            // Rescale value outside deadzone to maintain full range
                            if (processed_axis_value > 0) {
                                processed_axis_value = (processed_axis_value - JOYSTICK_DEADZONE) / (1.0f - JOYSTICK_DEADZONE);
                            } else {
                                processed_axis_value = (processed_axis_value + JOYSTICK_DEADZONE) / (1.0f - JOYSTICK_DEADZONE);
                            }
                        }

                        // Scale to -100 to 100
                        int8_t scaled_value = static_cast<int8_t>(std::round(processed_axis_value * 100.0f));

                        if (event.number == 0) { // X-axis (left/right)
                            current_joystick_x = scaled_value;
                            std::cout << "Axis X raw: " << raw_axis_value << ", processed: " << processed_axis_value << " (scaled: " << (int)scaled_value << ")" << std::endl;
                        } else if (event.number == 1) { // Y-axis (forward/backward)
                            current_joystick_y = -scaled_value; // Invert Y-axis for forward being positive
                            std::cout << "Axis Y raw: " << raw_axis_value << ", processed: " << processed_axis_value << " (scaled: " << (int)scaled_value << ")" << std::endl;
                        }
                        // Send joystick move command immediately
                        send_control_packet(CMD_JOYSTICK_MOVE, current_joystick_x, current_joystick_y);
                        break;
                    }
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
        if (m_joystick_connected.load() && current_joystick_x == 0 && current_joystick_y == 0 && !current_flash_button_state) {
            send_control_packet(CMD_NONE, 0, 0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Small pause
    }
    std::cout << "Dualshock3: Joystick input thread stopped." << std::endl;
}
