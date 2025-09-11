## Project Overview
The `hx-esp32-cam-fpv` project is an open-source digital FPV (First Person View) system based on ESP32 microcontrollers and camera modules . Its main goal is to provide a low-cost, compact, and low-power wireless video transmission solution for FPV applications, particularly for small inav-based planes  . The system enables real-time video streaming from an air unit (transmitter) to a ground station (receiver) with features like low latency, FEC encoding, and integrated OSD and telemetry . The target audience includes FPV enthusiasts, hobbyists, and developers interested in building or customizing digital FPV systems with ESP32 hardware .

## Architecture & Structure
The system comprises two main components: an Air Unit (VTX) and a Ground Station (VRX), communicating over a 2.4GHz WiFi link using packet injection and FEC encoding .

### High-level Architecture Overview
The Air Unit, typically an ESP32/ESP32S3 MCU with an OV2640/OV5640 camera, captures video, performs FEC encoding, and transmits it via WiFi . It can also record video to an SD card . The Ground Station, often a Single Board Computer (SBC) like a Radxa Zero 3W or Raspberry Pi, receives the WiFi stream using RTL8812AU cards in monitor mode, decodes the video, and displays it via SDL2/OpenGL .

### Key Directories and their Purposes
*   `air_firmware_esp32cam/`: Contains PlatformIO project files for the ESP32-CAM air unit firmware .
*   `components/air/`: Shared components for the air unit firmware .
*   `components/common/`: Common components shared between air unit and ground station .
*   `components/esp32-camera/`: Modified ESP32 camera component .
*   `gs/`: Contains the source code and Makefile for the Ground Station software .
*   `doc/`: Documentation files, including build instructions and development guides .
*   `scripts/`: Shell scripts for installation and boot configuration .

### Main Components and How They Interact
*   **Camera Module (OV2640/OV5640)**: Captures video frames as JPEG images and transmits them to the ESP32 via I2S .
*   **ESP32/ESP32S3 MCU**: Receives JPEG data from the camera, optionally writes to SD card, performs FEC encoding, and streams over WiFi using packet injection .
*   **WiFi Communication**: Uses 2.4GHz WiFi with custom packet-based protocol and FEC encoding for reliable transmission  .
*   **Ground Station Software (`gs` binary)**: Receives WiFi packets, performs FEC decoding, reconstructs JPEG frames, and uses TurboJPEG for fast decoding . It renders video and OSD elements using OpenGL ES and ImGui .
*   **RTL8812AU WiFi Cards**: Used by the Ground Station in monitor mode for receiving WiFi packets . Dual cards can be used for diversity reception .

### Data Flow and System Design
The video data flows from the camera sensor, through hardware JPEG encoding, into ESP32 DMA buffers . Adaptive quality control adjusts compression based on bandwidth . On the air unit, data is FEC encoded and injected into WiFi packets . On the ground station, WiFi packets are received, FEC decoded, and JPEG frames are reconstructed . TurboJPEG decodes frames, which are then uploaded as OpenGL ES textures for rendering to the display with OSD elements . The link is bidirectional, allowing the ground station to send configuration and telemetry data back to the air unit .

# Project Structure & Logging

## Project Structure Overview

The `hx-esp32-cam-fpv` project is organized into several key directories:

### Main Directories
*   `air_firmware_esp32cam/`: Contains PlatformIO project files for the ESP32-CAM air unit firmware
*   `components/`: Shared components used by both air unit and ground station
  *   `components/air/`: Components specific to the air unit firmware
  *   `components/common/`: Common components shared between air unit and ground station
  *   `components/esp32-camera/`: Modified ESP32 camera component
*   `gs/`: Contains the source code and Makefile for the Ground Station software
*   `doc/`: Documentation files, including build instructions and development guides
*   `scripts/`: Shell scripts for installation and boot configuration

### Key Subsystems
1.  **Air Unit (VTX)**: Runs on ESP32/ESP32S3 with OV2640/OV5640 camera, captures video, performs FEC encoding, and transmits via WiFi
2.  **Ground Station (VRX)**: Runs on SBC (Radxa Zero 3W or Raspberry Pi), receives WiFi stream, decodes video, and displays via SDL2/OpenGL
3.  **Communication Layer**: Custom packet-based protocol with FEC encoding over 2.4GHz WiFi

## Logging System

### Ground Station Logging (`gs/src/Log.h`)

The ground station uses a custom logging system based on the `fmt` library:

```cpp
// Log levels
enum class LogLevel : uint8_t {
    DBG,    // Debug messages (disabled in Release)
    INFO,   // Informational messages
    WARNING,// Warning messages
    ERR     // Error messages
};

// Logging macros
#define LOGD(fmt, ...) logf(LogLevel::DBG, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) logf(LogLevel::INFO, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) logf(LogLevel::WARNING, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) logf(LogLevel::ERR, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
```

**Usage Examples:**
```cpp
LOGI("Temperature: {:.2f}°C, Humidity: {:.2f}%", temperature, humidity);
LOGE("Error initializing camera: {}", error_message);
LOGW("Low battery: {}%", battery_level);
LOGD("Debug: frame size = {}", frame_size);
```

**Key Points:**
*   Uses `fmt::format` for string formatting (similar to Python's format syntax)
*   Automatically includes file name and line number
*   Different log levels for different purposes
*   Debug logs are disabled in Release builds

### Air Unit Logging (`components/common/safe_printf.h`)

The air unit uses a simpler logging approach due to ESP32 constraints:

```cpp
#define SAFE_PRINTF(...)  \
    do { xSemaphoreTake(s_safe_printf_mux, portMAX_DELAY); \
        ets_printf(__VA_ARGS__); \
        xSemaphoreGive(s_safe_printf_mux); \
    } while (false)
```

**Usage Example:**
```cpp
SAFE_PRINTF("Camera init failed with error 0x%x\n", err);
```

**Key Points:**
*   Thread-safe logging using mutex
*   Uses ESP32's `ets_printf` for output
*   Simple printf-style formatting
*   Defined in `components/common/safe_printf.h`

### Best Practices for Logging

1.  **Ground Station**: Use appropriate LOG* macros with descriptive messages
2.  **Air Unit**: Use SAFE_PRINTF for thread-safe logging
3.  **Format Strings**: Use `{}` placeholders for the `fmt` library in ground station, or standard printf format specifiers for air unit
4.  **Log Levels**: Choose appropriate levels (LOGD for debugging, LOGI for general info, LOGW for warnings, LOGE for errors)
5.  **Performance**: Avoid excessive logging in time-critical sections

# Development Partnership and How We Should Partner

We build production code together. I handle implementation details while you guide architecture and catch complexity early.

## Core Workflow: Research → Plan → Implement → Validate

**Start every feature with:** "Let me research the codebase and create a plan before implementing."

1. **Research** - Understand existing patterns and architecture
2. **Plan** - Propose approach and verify with you
3. **Implement** - Build with tests and error handling
4. **Validate** - ALWAYS run formatters, linters, and tests after implementation

## Code Organization

**Keep functions small and focused:**
- If you need comments to explain sections, split into functions
- Group related functionality into clear packages
- Prefer many small files over few large ones

## Architecture Principles

**This is always a feature branch:**
- Delete old code completely - no deprecation needed
- No "removed code" or "added this line" comments - just do it

**Prefer explicit over implicit:**
- Clear function names over clever abstractions
- Obvious data flow over hidden magic
- Direct dependencies over service locators

## Maximize Efficiency

**Parallel operations:** Run multiple searches, reads, and greps in single messages
**Multiple agents:** Split complex tasks - one for tests, one for implementation
**Batch similar work:** Group related file edits together

## Problem Solving

**When stuck:** Stop. The simple solution is usually correct.

**When uncertain:** "Let me ultrathink about this architecture."

**When choosing:** "I see approach A (simple) vs B (flexible). Which do you prefer?"

Your redirects prevent over-engineering. When uncertain about implementation, stop and ask for guidance.

## Testing Strategy

**Match testing approach to code complexity:**
- Complex business logic: Write tests first (TDD)
- Simple CRUD operations: Write code first, then tests
- Hot paths: Add benchmarks after implementation

**Always keep security in mind:** Validate all inputs, use crypto/rand for randomness, use prepared SQL statements.

**Performance rule:** Measure before optimizing. No guessing.

## Progress Tracking

- **Use Todo lists** for task management
- **Clear naming** in all code

Focus on maintainable solutions over clever abstractions.

# Command Functionality Documentation

## Analysis of Control Command Sending (GS to ESP32)

This section explains how control commands are sent from the Ground Station (GS) to the ESP32 and how this behavior can be modified to use buttons in the graphical interface instead of physical buttons on a Raspberry Pi.

### Control Mechanism Summary

The system does not send individual "commands" like "start recording." Instead, it works as follows:

1.  **Constant Status Packet:** The Ground Station (GS) continuously sends a data packet called `Ground2Air_Config_Packet` to the ESP32. This packet contains all the camera and communication channel configuration.
2.  **Button Counters:** Within that packet, in the nested `DataChannelConfig` structure, there are special fields that act as counters, for example, `air_record_btn`.
3.  **Interaction in the GS:** When you press a button in the GS graphical interface, the value of one of these counters is incremented by 1.
4.  **Periodic Sending:** A separate communication thread (`comms_thread_proc`) takes the latest version of the configuration packet (with the counter already incremented) and sends it over WiFi to the ESP32. This occurs approximately every 500 milliseconds.
5.  **Detection in the ESP32:** The ESP32 firmware receives this packet, saves the last known value of the counters, and in each new packet, checks if the value has changed. If the counter has been incremented, the ESP32 executes the associated action (such as starting or stopping recording).

This method is robust because it doesn't matter if a packet is lost; as long as one with the updated value eventually arrives, the action will be executed.

### Detailed Flow in the Code

Here are the key parts of the code that implement this flow:

#### 1. The Data Structure (The "Command")

The main packet that is sent is defined in `components/common/packets.h`.

*   `Ground2Air_Config_Packet`: This is the structure that contains all the information that travels from the GS to the ESP32.
*   `DataChannelConfig`: Within the previous packet, this structure contains the button counters that we're interested in:
    *   `uint8_t air_record_btn = 0;`
    *   `uint8_t profile1_btn = 0;`
    *   `uint8_t profile2_btn = 0;`

    The comment in the code is key: `//incremented each time button is pressed on gs`.

#### 2. The Sending Thread (Where it's sent)

In `gs/src/main.cpp`, the function `comms_thread_proc()` runs in a separate thread and is responsible for all communication.

*   **Lines 535-549:** Within its infinite loop, this block of code executes every 500ms. It prepares and sends the global configuration packet `s_ground2air_config_packet`.

    ```cpp
    // gs/src/main.cpp:537
    if ( s_got_config_packet )
    {
        std::lock_guard<std::mutex> lg(s_ground2air_config_packet_mutex);
        auto& config = s_ground2air_config_packet;
        // ... (prepare the packet)
        s_comms.send(&config, sizeof(config), true);
    }
    ```

#### 3. The Graphical Interface (How the command is modified)

The interface is drawn in the `run()` function within `gs/src/main.cpp`. The logic of the existing buttons gives us the pattern to follow.

*   **Lines 2131-2134:** Here you can see how the "Air Record" button in the debug window implements this mechanism.

    ```cpp
    // gs/src/main.cpp:2131
    if ( ImGui::Button("Air Record") )
    {
        config.dataChannel.air_record_btn++;
    }
    ```

    When the button is clicked, the `air_record_btn` counter in the local variable `config` is simply incremented.

*   **Line 2268:** At the end of the interface loop, the modified local configuration (`config`) is copied back to the global variable `s_ground2air_config_packet`, leaving it ready for the communication thread to send.

    ```cpp
    // gs/src/main.cpp:2268
    s_ground2air_config_packet = config;
    ```

### How to Implement Your Own Buttons

Now that you understand the flow, you can easily add your own buttons.

1.  **Go to the file** `gs/src/main.cpp`.
2.  **Find the `run()` function** and the lambda expression `auto f = [&config,&argv]` that is defined inside (around line 1182). This is the heart of the graphical interface.
3.  **Add a new ImGui window** for your controls. You can put this code next to the other windows, for example, after the `//------------ debug window` block.

    ```cpp
    // Put this inside the 'f' lambda, for example, before line 2167

    ImGui::Begin("My Controls"); // Creates a new window called "My Controls"

    // Button to activate profile 1
    if (ImGui::Button("Activate Profile 1"))
    {
        // When clicked, we increment the profile 1 button counter.
        // The ESP32 must be programmed to react to this change.
        config.dataChannel.profile1_btn++;
    }

    ImGui::SameLine(); // Place the next button on the same line

    // Button to activate profile 2
    if (ImGui::Button("Activate Profile 2"))
    {
        config.dataChannel.profile2_btn++;
    }

    // You can add more logic here if necessary

    ImGui::End(); // Close the window
    ```

4.  **Ready!** When you compile and run, you'll see a new window with your buttons. When clicked, you'll be modifying the configuration packet that is sent to the ESP32, replicating exactly the functionality that the physical buttons on the Raspberry Pi had.

### Keyboard Handling Section in the Station

```cpp
if ( ImGui::IsKeyPressed(ImGuiKey_S))
{
    s_groundstation_config.stats = !s_groundstation_config.stats;
}
```

# Air Firmware Documentation

## 1. Introduction

The `air_firmware_esp32cam` firmware is a real-time video transmission solution designed for ESP32-CAM devices. This firmware allows capturing video from an OV2640 or OV5640 camera module, compressing it in JPEG format, and transmitting it through a Wi-Fi connection to a receiving station (ground station).

## 2. General System Architecture

The system is based on the ESP-IDF (Espressif IoT Development Framework) and uses multiple components to handle different aspects of operation:

- **Video Capture**: Uses the camera sensor connected to the ESP32-CAM
- **Image Processing**: Application of quality, brightness, contrast, etc. settings
- **JPEG Compression**: Compression of captured frames
- **Wi-Fi Transmission**: Sending data through a point-to-point Wi-Fi connection
- **FEC Encoding**: Application of Forward Error Correction to improve transmission robustness
- **OSD (On-Screen Display)**: Overlay of information on the transmitted video
- **Control and Configuration**: Handling configuration commands from the base station

## 3. Main Components

### 3.1 Video Capture and Processing

The firmware uses the `esp32-camera` library to interact with the image sensor. Capture is performed in JPEG mode directly from the sensor, which reduces processing load on the ESP32.

**Key Features:**
- Support for OV2640 and OV5640 sensors
- Configurable resolutions from QVGA (320x240) to UXGA (1600x1200)
- Control of image parameters: brightness, contrast, saturation, sharpness, white balance, etc.
- Configurable frame rate (FPS)

### 3.2 Wi-Fi Transmission and Packet Protocol

Communication is performed through a Wi-Fi connection in station (STA) or access point (AP) mode, using custom packets for data transmission.

**Packet Types:**
1. **Video Packets (Air2Ground_Video_Packet)**
   - Contain JPEG data fragments
   - Include frame index and part information
   - Indicate the end of a frame

2. **OSD Packets (Air2Ground_OSD_Packet)**
   - Contain on-screen display overlay information
   - Include system statistics
   - Updated periodically

3. **Configuration Packets (Air2Ground_Config_Packet)**
   - Contain the current camera and transmission configuration
   - Sent to synchronize configuration between air and ground

4. **Telemetry Packets (Air2Ground_Data_Packet)**
   - Contain system telemetry data
   - Used to send additional information such as control data

### 3.3 FEC System (Forward Error Correction)

To improve transmission robustness in environments with interference, the firmware implements an FEC system based on the Reed-Solomon algorithm.

**Operation:**
- Data is divided into blocks of size K
- N-K additional parity packets are generated
- At the receiver, original data can be recovered even if some packets are lost
- K and N parameters are configurable

### 3.4 OSD (On-Screen Display)

The OSD system allows overlaying information on the transmitted video, such as system statistics, connection status, etc.

**Displayed Information:**
- Wi-Fi signal quality (RSSI)
- Current frame rate
- Recording status
- Sensor temperature
- Transmission statistics
- SD card space available (if available)

### 3.5 Configuration and Control

The firmware allows remote configuration through configuration packets sent from the base station.

**Configurable Parameters:**
- Video resolution
- Image quality
- Brightness, contrast, saturation
- Wi-Fi channel and power
- FEC parameters
- Frame rate limit

### 3.7 Temperature Handling

The firmware includes a sensor temperature monitoring system to prevent overheating.

**Functions:**
- Periodic reading of sensor temperature
- Temperature reporting in OSD packets
- Possible implementation of thermal throttling (not implemented in current version)

## 4. Data Flow Diagram

```
[Camera] → JPEG → [Callback camera_data_available]
                            ↓
                    [JPEG Data Processing]
                            ↓
              [Division into Air2Ground_Video_Packets]
                            ↓
                    [FEC Encoding (s_fec_encoder)]
                            ↓
                [Wi-Fi Transmission Queue (s_wlan_outgoing_queue)]
                            ↓
                        [Wi-Fi Transmission]
                            ↓
                    [Reception at base station]
                            ↓
                [FEC Decoding (s_fec_decoder)]
                            ↓
                    [Packet Processing]
                            ↓
                        [Display]
```

## 5. Transmission Packet Description

### 5.1 Air2Ground_Video_Packets

These packets contain the fragmented JPEG video data.

**Structure:**
- `type`: Packet type (Video)
- `resolution`: Video resolution
- `frame_index`: Frame index
- `part_index`: Part index within the frame
- `last_part`: Indicator of last part of the frame
- `size`: Total packet size
- `pong`: Used to measure latency
- `version`: Protocol version
- `airDeviceId`: Air device ID
- `gsDeviceId`: Ground station device ID
- `crc`: Verification code

### 5.2 Air2Ground_OSD_Packets

These packets contain OSD information and system statistics.

**Structure:**
- `type`: Packet type (OSD)
- `size`: Total packet size
- `pong`: Used to measure latency
- `version`: Protocol version
- `airDeviceId`: Air device ID
- `gsDeviceId`: Ground station device ID
- `stats`: System statistics (see AirStats structure)
- `buffer`: OSD buffer data
- `crc`: Verification code

### 5.3 Air2Ground_Config_Packets

These packets contain the current device configuration.

**Structure:**
- `type`: Packet type (Config)
- `size`: Total packet size
- `version`: Protocol version
- `airDeviceId`: Air device ID
- `gsDeviceId`: Ground station device ID
- `camera`: Camera configuration (see CameraConfig structure)
- `dataChannel`: Data channel configuration (see DataChannelConfig structure)
- `crc`: Verification code

### 5.4 Ground2Air Packets

These packets are sent from the base station to the air device.

**Types:**
- `Config`: Device configuration
- `Connect`: Connection initialization
- `Control`: Control commands
- `Telemetry`: Telemetry data

## 6. GPIO Pin Assignment

The GPIO pin assignment follows the typical ESP32-CAM module scheme:

**Camera Pins (OV2640/OV5640):**
- `Y2_GPIO_NUM`: GPIO5
- `Y3_GPIO_NUM`: GPIO18
- `Y4_GPIO_NUM`: GPIO19
- `Y5_GPIO_NUM`: GPIO21
- `Y6_GPIO_NUM`: GPIO36
- `Y7_GPIO_NUM`: GPIO39
- `Y8_GPIO_NUM`: GPIO34
- `Y9_GPIO_NUM`: GPIO35
- `XCLK_GPIO_NUM`: GPIO0
- `PCLK_GPIO_NUM`: GPIO22
- `VSYNC_GPIO_NUM`: GPIO25
- `HREF_GPIO_NUM`: GPIO23
- `SIOD_GPIO_NUM`: GPIO26
- `SIOC_GPIO_NUM`: GPIO27
- `PWDN_GPIO_NUM`: GPIO32
- `RESET_GPIO_NUM`: -1 (not used)

**Control Pins:**
- `GPIO_CONTROL_PIN`: GPIO4 (configurable for external control)

**UART Pins:**
- `TXD0_PIN`: GPIO1
- `RXD0_PIN`: GPIO3

## 7. Initialization Procedure

1. **System Initialization:**
   - UART configuration for debugging
   - NVS (Non-Volatile Storage) initialization
   - Reading stored configuration

2. **Wi-Fi Configuration:**
   - Wi-Fi subsystem initialization
   - Channel, rate, and power configuration
   - Promiscuous mode configuration for packet reception

3. **FEC Initialization:**
   - Encoder and decoder FEC configuration
   - Transmission and reception queue initialization

4. **Camera Initialization:**
   - Image sensor configuration
   - Application of configuration parameters
   - Start of video capture

5. **Task Creation:**
   - Wi-Fi transmission task
   - Wi-Fi reception task
   - Main processing loop

## 8. Video Capture and Transmission

### 8.1 Capture Process

1. The image sensor captures frames in JPEG format
2. JPEG data is sent to the `camera_data_available` callback
3. In the callback, data is processed and divided into packets
4. Packets are FEC encoded and added to the transmission queue

### 8.2 Adaptive Quality Control

The firmware implements an adaptive quality control system to maintain stable transmission:

1. **Bandwidth Measurement:**
   - Monitoring of Wi-Fi queue usage
   - Counting transmission errors
   - Measuring effective frame rate

2. **Quality Adjustment:**
   - Dynamic modification of JPEG quality
   - Sharpness adjustment according to quality
   - Maximum frame size control

3. **Overload Prevention:**
   - Detection of Wi-Fi queue overload
   - Quality reduction when congestion is detected
   - Temporary transmission stop if necessary

## 9. Received Packet Processing

### 9.1 Packet Reception

1. Packets are received through the `packet_received_cb` callback
2. Wi-Fi channel and MAC address are verified
3. Packets are filtered according to device ID
4. Data is FEC decoded and added to the reception queue

### 9.2 Configuration Packet Handling

1. Configuration packets are processed in `handle_ground2air_config_packet`
2. Configuration parameters are updated
3. Changes are applied in real-time when possible
4. Some changes require restart or camera reconfiguration

### 9.3 Control Packet Handling

1. Control packets are processed in `handle_ground2air_control_packet`
2. Specific actions are executed such as GPIO activation
3. Confirmations are sent when necessary

## 10. Statistics Collection

The firmware collects and transmits system statistics periodically:

### 10.1 Transmission Statistics
- Data sent and received
- Transmission errors
- Current and expected frame rate
- Wi-Fi queue usage

### 10.2 Camera Statistics
- Minimum and maximum frame sizes
- Camera overload count
- Sensor temperature

### 10.3 System Statistics
- Memory usage
- SD card status (if available)
- Recording status

## 11. Unidirectional Transmission Operation

The system is designed primarily as a unidirectional transmission from the air device to the base station:

1. **Video Transmission:**
   - JPEG frames are transmitted continuously
   - No reception confirmation is expected
   - Quality is dynamically adjusted according to conditions

2. **OSD Transmission:**
   - Status information is sent periodically
   - Includes system statistics and connection status

3. **Configuration Transmission:**
   - Current configuration is sent periodically
   - Allows the base station to maintain synchronization

4. **Command Reception:**
   - The base station can send configuration commands
   - Commands are processed and applied in real-time

## 12. Data Integrity Measures

### 12.1 CRC Verification

All packets include a CRC-8 code to verify data integrity:

1. Before sending, the packet CRC is calculated
2. Upon receipt, the CRC is verified against the content
3. Packets with invalid CRC are discarded

### 12.2 Packet Filtering

The system implements a packet filtering mechanism:

1. **Device ID Filtering:**
   - Only packets directed to the device are processed
   - Connections from specific devices can be accepted

2. **Channel Filtering:**
   - Only packets in the configured channel are processed
   - Packets from other channels are ignored

### 12.3 Forward Error Correction

The FEC system helps maintain data integrity in environments with interference:

1. **Encoding at the Transmitter:**
   - Data is divided into blocks
   - Additional parity packets are generated

2. **Decoding at the Receiver:**
   - Lost data can be recovered using parity packets
   - A minimum number of packets is required for recovery

## 13. Optimizations and Performance Considerations

### 13.1 Memory Usage

- Extensive use of PSRAM for transmission queues
- Optimization of data structures to minimize memory usage
- Use of circular buffers for efficient data handling

### 13.2 CPU Optimization

- Use of IRAM_ATTR functions for critical code
- Minimization of operations in interrupt callbacks
- Use of separate tasks for intensive processing

### 13.3 Wi-Fi Optimization

- Fixed rate and power configuration to maximize performance
- Use of HT20 mode for better stability
- Continuous monitoring of connection status

## 14. Diagnosis and Debugging

### 14.1 Debug Output

The firmware can generate debug output through UART0:

- System statistics every second
- Error and warning information
- Component status information

### 14.2 Performance Monitoring

- Measurement of CPU usage by task
- Monitoring of memory usage
- Detailed transmission statistics

## 15. Security Considerations

### 15.1 Device Identification

- Each device has a unique ID generated from its MAC address
- Packets are filtered according to source and destination IDs

### 15.2 Privacy

- No data encryption is implemented
- Transmissions can be intercepted if not protected at the network level

## 16. Possible Future Improvements

1. **Data Encryption:**
   - Implement AES encryption to protect transmission

2. **Bidirectional Control Protocol:**
   - Improve the control protocol for more robust communication

3. **Support for More Sensors:**
   - Expand support to more image sensor models

4. **OSD Improvements:**
   - Implement more OSD customization options

5. **FEC Optimization:**
   - Implement more efficient FEC algorithms

6. **Audio Support:**
   - Add audio transmission capability

## 17. Conclusion

The `air_firmware_esp32cam` firmware provides a complete solution for real-time video transmission using an ESP32-CAM. With its adaptive quality control system, FEC for robustness, and remote configuration capability, it is ideal for FPV (First Person View) applications and other uses where reliable video transmission is required in environments with possible interference.

The modular architecture of the firmware allows for easy maintenance and extension, while performance optimizations ensure smooth transmission even in challenging conditions.

# Auxiliary Unit Functionality (ESP32-AUX)

## Overview

The system includes a separate ESP32 auxiliary unit (`esp32_aux`) that acts as a support device for the main air unit (ESP32-CAM). This auxiliary unit handles multiple functions including environmental sensor readings (DHT11) and motor control, communicating with the air unit via I2C.

## Implementation Details

### Hardware Architecture

The system consists of two ESP32 devices:
1. **Air Unit (ESP32-CAM)**: Handles video capture, processing, and transmission
2. **Auxiliary Unit (ESP32)**: Dedicated to reading the DHT11 sensor, controlling motors, and communicating with the air unit via I2C

### Auxiliary Unit Implementation (`esp32_aux`)

The auxiliary unit firmware is located in the `esp32_aux` directory and includes:

1. **DHT11 Initialization**: The DHT11 sensor is initialized on GPIO4
2. **Motor Control**: Motor controller initialization with pins IN1-IN4 and ENA/ENB for speed control
3. **I2C Slave Communication**: The auxiliary unit acts as an I2C slave, receiving commands from the air unit
4. **Periodic Reading Task**: A FreeRTOS task (`dht11_task`) that reads sensor data every 2 seconds

### Code Structure (Auxiliary Unit)

```cpp
// Motor pins configuration
#define IN1_PIN GPIO_NUM_14
#define IN2_PIN GPIO_NUM_27
#define IN3_PIN GPIO_NUM_26
#define IN4_PIN GPIO_NUM_25
#define ENA_PIN GPIO_NUM_33
#define ENB_PIN GPIO_NUM_32

// DHT11 sensor pin
#define DHT11_PIN GPIO_NUM_4

// I2C Command Definitions
#define I2C_CMD_FORWARD 1
#define I2C_CMD_BACKWARD 2
#define I2C_CMD_RIGHT 3
#define I2C_CMD_LEFT 4
#define I2C_CMD_FLASH 5

// DHT11 sensor instance
dht11_sensor_t dht11_sensor;

// Function to send DHT11 data via I2C
void send_dht11_data(float humidity, float temperature) {
    // For now, just log the data
    ESP_LOGI(TAG, "DHT11 Data - Humidity: %.1f%%, Temperature: %.1f°C", humidity, temperature);
}

// DHT11 reading task
void dht11_task(void *pvParameter) {
    float humidity, temperature;
    while (1) {
        // Read data from the sensor
        esp_err_t read_result = dht11_read(&dht11_sensor, &humidity, &temperature);
        if (read_result == ESP_OK) {
            // Print the values
            dht11_print_values(humidity, temperature);
            // Send data via I2C
            send_dht11_data(humidity, temperature);
        } else {
            ESP_LOGE(TAG, "Failed to read data from DHT11 sensor");
        }
        // Wait for 2 seconds before next reading
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main() {
    // Initialize I2C Slave
    if (init_i2c_slave() != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing I2C Slave");
        return;
    }
    
    // Initialize motor controller
    Motor motor(IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN, ENA_PIN, ENB_PIN);
    
    // Initialize DHT11 sensor
    if (dht11_init(&dht11_sensor, DHT11_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DHT11 sensor");
    } else {
        ESP_LOGI(TAG, "DHT11 sensor initialized on GPIO %d", DHT11_PIN);
        // Create DHT11 reading task
        xTaskCreate(dht11_task, "dht11_task", 2048, NULL, 5, NULL);
    }
    
    ESP_LOGI(TAG, "System started - Waiting for I2C commands");
    
    // Process I2C commands
    while (1) {
        // Read data from master (write operation from master)
        int size = i2c_slave_read_buffer(I2C_SLAVE_PORT, receivedData, BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (size > 0) {
            ESP_LOGI(TAG, "Received data (%d bytes): ", size);
            
            // Process each received byte as a command
            for (int i = 0; i < size; i++) {
                ESP_LOGI(TAG, "Received command: 0x%02X", receivedData[i]);
                
                switch (receivedData[i]) {
                    case I2C_CMD_FORWARD:
                        motor.forward();
                        break;
                    case I2C_CMD_BACKWARD:
                        motor.backward();
                        break;
                    case I2C_CMD_RIGHT:
                        motor.right();
                        break;
                    case I2C_CMD_LEFT:
                        motor.left();
                        break;
                    case I2C_CMD_FLASH:
                        // Flash command - not implemented for motor control
                        ESP_LOGI(TAG, "FLASH command received - no action on motors");
                        break;
                    default:
                        // Unknown command - stop motors
                        motor.stop();
                        ESP_LOGW(TAG, "Unknown command: 0x%02X - stopping motors", receivedData[i]);
                        break;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

## Air Unit Integration

### I2C Master Implementation

The air unit acts as an I2C master, sending commands to the auxiliary unit and periodically requesting sensor data:

1. **Motor Control Commands**: The air unit can send movement commands (forward, backward, left, right) to the auxiliary unit
2. **DHT11 Task**: A dedicated task (`dht11_task`) in the air unit firmware handles communication with the auxiliary unit for sensor data
3. **Priority Handling**: The task checks if the camera is actively capturing frames to avoid interference
4. **Timeout Management**: Communication with the auxiliary unit includes timeout handling to prevent blocking

### Code Structure (Air Unit)

```cpp
// DHT11 data variables
static float s_dht11_humidity = 0.0;
static float s_dht11_temperature = 0.0;
static bool s_dht11_data_valid = false;

// Function to read DHT11 data from I2C slave with timeout and priority handling
esp_err_t read_dht11_data(float* humidity, float* temperature) {
    // Check if camera is actively capturing frames, if so, delay DHT11 reading
    if (s_video_frame_started) {
        // Implementation with timeout and priority handling
    }
    // I2C communication with auxiliary unit
    // Implementation details...
}

// DHT11 task to read sensor data when camera is not busy
void dht11_task(void* pvParameters) {
    while (1) {
        // Read DHT11 data with timeout to prevent blocking
        float humidity, temperature;
        esp_err_t result = read_dht11_data(&humidity, &temperature);
        if (result == ESP_OK) {
            // Update global variables
            s_dht11_humidity = humidity;
            s_dht11_temperature = temperature;
            s_dht11_data_valid = true;
            LOG("DHT11 data - Humidity: %.2f%%, Temperature: %.2f°C\n", humidity, temperature);
        } else {
            s_dht11_data_valid = false;
            LOG("Failed to read DHT11 data: %s\n", esp_err_to_name(result));
        }
        // Delay between readings
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
```

## Configuration

The auxiliary unit is configured with the following pin assignments:

### Motor Control Pins
- **IN1**: GPIO14
- **IN2**: GPIO27
- **IN3**: GPIO26
- **IN4**: GPIO25
- **ENA**: GPIO33 (Speed control for motor A)
- **ENB**: GPIO32 (Speed control for motor B)

### Sensor Pin
- **DHT11**: GPIO4

The DHT11 task in the air unit is created during firmware initialization with appropriate priority settings:

```cpp
// Create DHT11 task with lower priority to minimize interference
xTaskCreate(dht11_task, "dht11_task", 2048, NULL, 1, &s_dht11_task_handle);
if (s_dht11_task_handle == NULL) {
    LOG("Failed to create DHT11 task\n");
} else {
    LOG("DHT11 task created successfully\n");
}
```

## I2C Communication Protocol

The air unit communicates with the auxiliary unit using a simple command protocol:

- **0x01**: Move Forward
- **0x02**: Move Backward
- **0x03**: Turn Right
- **0x04**: Turn Left
- **0x05**: Flash (Reserved for future use)

The auxiliary unit continuously listens for these commands and executes the appropriate motor control actions.

# Report Packet Functionality

## Overview

A new packet type, `Air2Ground_Report_Packet`, has been added to the communication protocol to transmit sensor data (such as DHT11 readings from the auxiliary unit) from the air unit to the ground station.

## Packet Structure

The report packet is defined in `components/common/packets.h` and contains the following fields:

```cpp
struct Air2Ground_Report_Packet : Air2Ground_Header
{
    float temperature; // Temperature data (e.g., from DHT11 or other sensors)
    float humidity;    // Humidity data (e.g., from DHT11 or other sensors)
    uint8_t data_valid : 1; // Flag to indicate if the data is valid
    uint8_t reserved : 7;   // Reserved for future use
};
```

## Transmission Implementation

### Sending Report Packets

Report packets are sent periodically (every 5 seconds) from the air unit to the ground station:

```cpp
static int64_t s_last_report_packet_tp = -5000;

// Send DHT11 report packet every 5 seconds
dt = millis() - s_last_report_packet_tp;
if (dt > 5000) {
    // Only send report packet if camera is not actively capturing
    if (!s_video_frame_started && s_initialized && s_connected_gs_device_id != 0) {
        s_fec_encoder.lock();
        send_air2ground_report_packet();
        s_fec_encoder.unlock();
    }
    s_last_report_packet_tp = millis(); // Reset timer after sending
}
```

### Report Packet Creation

The `send_air2ground_report_packet()` function creates and sends the report packet with data from the auxiliary unit:

```cpp
IRAM_ATTR void send_air2ground_report_packet() {
    Air2Ground_Report_Packet& packet = *(Air2Ground_Report_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::Report; // Using Report type for report packets
    packet.size = sizeof(Air2Ground_Report_Packet);
    packet.pong = s_ground2air_config_packet.ping;
    
    // Fill in DHT11 data from auxiliary unit
    packet.temperature = s_dht11_temperature;
    packet.humidity = s_dht11_humidity;
    packet.data_valid = s_dht11_data_valid ? 1 : 0;
    
    packet.crc = crc8(0, &packet, sizeof(Air2Ground_Report_Packet));
    // Send packet through FEC encoder
}
```

## Ground Station Handling

### Receiving Report Packets

The ground station handles report packets in the main packet processing loop:

```cpp
else if (air2ground_header.type == Air2Ground_Header::Type::Report) {
    if (packet_size < sizeof(Air2Ground_Report_Packet)) {
        LOGE("Report frame: data too small: {} > {}", packet_size, sizeof(Air2Ground_Report_Packet));
        break;
    }
    
    Air2Ground_Report_Packet& air2ground_report_packet = *(Air2Ground_Report_Packet*)rx_data.data.data();
    
    // CRC verification
    uint8_t crc = air2ground_report_packet.crc;
    air2ground_report_packet.crc = 0;
    uint8_t computed_crc = crc8(0, rx_data.data.data(), sizeof(Air2Ground_Report_Packet));
    if (crc != computed_crc) {
        // Handle CRC error
        break;
    }
    
    // Log that we received a report packet
    LOGI("Received Report packet: Temperature={:.2f}°C, Humidity={:.2f}%, Valid={}",
         air2ground_report_packet.temperature,
         air2ground_report_packet.humidity,
         (int)air2ground_report_packet.data_valid);
    
    // Update DHT11 data variables
    s_dht11_temperature = air2ground_report_packet.temperature;
    s_dht11_humidity = air2ground_report_packet.humidity;
    s_dht11_data_valid = air2ground_report_packet.data_valid != 0;
    s_last_dht11_data_tp = Clock::now(); // Update timestamp when data is received
}
```

### Displaying Sensor Data

The ground station displays the received DHT11 data in the status bar, showing "N/A" if the data is not valid or hasn't been received within the last 10 seconds:

```cpp
// DHT11 data variables
float s_dht11_temperature = 0.0f;
float s_dht11_humidity = 0.0f;
bool s_dht11_data_valid = false;
Clock::time_point s_last_dht11_data_tp = Clock::now();

// Display DHT11 data in status bar
// Show data if valid and received within last 10 seconds, otherwise show "N/A"
char buf[64];
if (s_dht11_data_valid && (Clock::now() - s_last_dht11_data_tp < std::chrono::seconds(10))) {
    sprintf(buf, "%.1f°C %.1f%%", s_dht11_temperature, s_dht11_humidity);
} else {
    sprintf(buf, "N/A");
}
```

## System Architecture Benefits

1. **Separation of Concerns**: The auxiliary unit handles sensor reading, allowing the air unit to focus on video processing and transmission
2. **Non-Interfering Operation**: The report packet system operates independently of the main video transmission
3. **Periodic Updates**: Sensor data is sent every 5 seconds, providing regular updates without overwhelming the communication channel
4. **Error Handling**: CRC verification ensures data integrity, and timeout handling prevents blocking of critical operations
5. **Flexible Design**: The report packet structure can be extended to include additional sensor types in the future

### DHT11 Logic (`handle_dht11_logic` function)

The `handle_dht11_logic` function is responsible for managing the reading and sending of DHT11 sensor data from the air unit. It implements a two-pass mechanism to ensure that both the sensor reading (which involves I2C communication) and the packet transmission are handled efficiently without blocking critical video streaming operations.

**Functionality:**
- **Two-Pass Cycle:** The function operates in two distinct passes within a single 5-second interval:
    1.  **Read Pass:** In the first pass, it attempts to read data from the DHT11 sensor via I2C. If successful, it updates the global `s_dht11_temperature`, `s_dht11_humidity`, and `s_dht11_data_valid` variables.
    2.  **Send Pass:** In the second pass, it checks if the DHT11 data is valid and if the air unit is connected to a ground station. If both conditions are met, it calls `send_air2ground_report_packet()` to transmit the sensor data.
- **Flexible Timing:** A `s_last_report_packet_tp` timer ensures that a full read-and-send cycle is attempted only after a minimum of 5 seconds has passed. However, the two-pass approach allows each step (read and send) to execute in separate, short bursts within the `camera_data_available` callback, ensuring that the overall process is not strictly tied to a single 5-second tick but can adapt to the availability of processing time.

**Importance of Placement (`camera_data_available` callback):**
It is crucial that `handle_dht11_logic` is called from within the `camera_data_available` callback, specifically after `s_video_frame_started` is set to `false` (at the end of a video frame). This placement is vital for the following reasons:

1.  **Non-Blocking Operation:** The `camera_data_available` callback is a high-priority, IRAM-optimized function that runs frequently during active video streaming. By integrating the DHT11 logic here, the sensor reading and packet sending operations are executed in small, quick bursts, minimizing their impact on the real-time video processing and transmission.
2.  **Reliable Execution during Streaming:** Unlike the `app_main` loop, which might miss brief windows of opportunity due to its `vTaskDelay`, the `camera_data_available` callback provides a consistent and frequent execution context. The end of a video frame (when `s_video_frame_started` is `false`) is a natural synchronization point where the system is momentarily less burdened by video data, making it an ideal time to perform other periodic tasks.
3.  **Avoiding Interference:** Placing the I2C communication (reading DHT11) and Wi-Fi transmission (sending report packet) within this callback, and splitting them into two passes, helps prevent these operations from interfering with the continuous flow of video data. This ensures that the FPV system maintains low latency and high video quality while still providing essential telemetry.
4.  **Adaptive Scheduling:** The 5-second timer acts as a minimum interval, but the actual execution of the read and send passes will occur at the next available opportunity within the `camera_data_available` callback after the timer expires. This makes the system more robust to temporary CPU load spikes, as it doesn't strictly demand execution at an exact 5-second mark but rather "at or after" 5 seconds when resources are available.
