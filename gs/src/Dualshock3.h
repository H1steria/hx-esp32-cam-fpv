#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cmath> // For std::abs, std::round
#include <functional> // For std::function
#include "Clock.h" // For Clock::time_point
#include "packets.h" // For Ground2Air_Control_Packet

class Dualshock3 {
public:
    Dualshock3();
    ~Dualshock3();

    void start();
    void stop();

    bool isConnected() const { return m_joystick_connected.load(); }
    
    // Callback for control commands
    using ControlCallback = std::function<void(uint8_t command, int8_t joystick_x, int8_t joystick_y)>;
    void setControlCallback(ControlCallback callback) { m_control_callback = callback; }

private:
    void joystick_input_thread_proc();
    bool openJoystick();
    void closeJoystick();
    bool isJoystickDevicePresent();
    void send_control_packet(uint8_t command, int8_t joystick_x, int8_t joystick_y);

    int m_joystick_fd;
    const char* m_joystick_device;
    std::atomic<bool> m_joystick_connected;
    std::thread m_input_thread;
    std::atomic<bool> m_running;

    // Last sent packet state for throttling
    Ground2Air_Control_Packet m_last_sent_control_packet;
    Clock::time_point m_last_control_packet_sent_tp;

    ControlCallback m_control_callback;

    const float JOYSTICK_DEADZONE = 0.30f; // Deadzone applied to normalized float values
    const int JOYSTICK_MAX_VALUE = 32767;
    const std::chrono::milliseconds PACKET_SEND_INTERVAL = std::chrono::milliseconds(50); // Send every 50ms
};
