## Project Overview
The `hx-esp32-cam-fpv` project is an open-source digital FPV (First Person View) system based on ESP32 microcontrollers and camera modules . Its main goal is to provide a low-cost, compact, and low-power wireless video transmission solution for FPV applications, particularly for small inav-based planes  . The system enables real-time video streaming from an air unit (transmitter) to a ground station (receiver) with features like low latency, FEC encoding, and integrated OSD and telemetry . The target audience includes FPV enthusiasts, hobbyists, and developers interested in building or customizing digital FPV systems with ESP32 hardware .

## Architecture & Structure
The system comprises two main components: an Air Unit (VTX) and a Ground Station (VRX), communicating over a 2.4GHz WiFi link using packet injection and FEC encoding .

### High-level Architecture Overview
The Air Unit, typically an ESP32/ESP32S3 MCU with an OV2640/OV5640 camera, captures video, performs FEC encoding, and transmits it via WiFi . It can also record video to an SD card . The Ground Station, often a Single Board Computer (SBC) like a Radxa Zero 3W or Raspberry Pi, receives the WiFi stream using RTL8812AU cards in monitor mode, decodes the video, and displays it via SDL2/OpenGL .

### Key Directories and their Purposes
*   `air_firmware_esp32cam/`: Contains PlatformIO project files for the ESP32-CAM air unit firmware .
*   `air_firmware_esp32s3sense/`: Contains PlatformIO project files for the ESP32S3 Sense air unit firmware.
*   `air_firmware_esp32s3sense_ov5640/`: Contains PlatformIO project files for the ESP32S3 Sense with OV5640 camera air unit firmware .
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

## Development Setup

### Prerequisites and Dependencies
For the Ground Station, common dependencies include `libdrm-dev`, `libgbm-dev`, `libgles2-mesa-dev`, `libpcap-dev`, `libturbojpeg0-dev`, `libts-dev`, `libfreetype6-dev`, `build-essential`, `autoconf`, `automake`, `libtool`, `libasound2-dev`, `libudev-dev`, `libdbus-1-dev`, `libxext-dev`, `libsdl2-dev`, `dkms`, `git`, and `aircrack-ng`  . Specific hardware like RTL8812AU WiFi cards are recommended .

For the Air Unit firmware, PlatformIO is used, which requires Python and its dependencies .

### Installation Steps
#### Ground Station
1.  **Automated Installation (on RubyFPV images)**: Use the `install_on_ruby.sh` script. This script detects the platform (Radxa Zero 3W or Raspberry Pi), installs dependencies, compiles SDL2 from source (for Raspberry Pi), clones the repository, compiles the ground station software, and configures boot scripts .
    *   For Radxa Zero 3W: `wget https://raw.githubusercontent.com/RomanLut/hx-esp32-cam-fpv/refs/heads/release/scripts/install_on_ruby.sh` followed by `chmod +x install_on_ruby.sh` and `./install_on_ruby.sh` .
    *   For Raspberry Pi: Similar steps, but the script will compile SDL2 from source .
2.  **Manual Installation (on Raspberry Pi OS or Ubuntu)**:
    *   Install required packages using `sudo apt install --no-install-recommends -y ...`  .
    *   Clone the repository: `git clone -b release --recursive https://github.com/RomanLut/esp32-cam-fpv` .
    *   Navigate to the `gs` directory: `cd esp32-cam-fpv/gs` .
    *   Build the software: `make -j<jobs>` (where `<jobs>` is determined by available memory, e.g., `make -j4`)  .

#### Air Unit Firmware
1.  **Flashing Prebuilt Firmware**: Instructions are provided for ESP32-CAM and ESP32S3 Sense  .
2.  **Building and Flashing with PlatformIO**:
    *   Install PlatformIO .
    *   Clone the repository: `git clone -b release --recursive https://github.com/RomanLut/esp32-cam-fpv` .
    *   Open the project in PlatformIO (e.g., `esp32-cam-fpv\air_firmware_esp32cam\esp32-cam-fpv-esp32cam.code-workspace`) .
    *   Connect the ESP32 board and click `PlatformIO: Upload` .

### Environment Configuration
*   **Raspberry Pi/Radxa**: Configure display resolution (e.g., 1280x720x60Hz), enable serial port hardware, and disable screen blanking/compositor  .
*   **GPIO Keys**: Add `gpio=24,18,22,27,23,17,4=pd` to `/boot/config.txt` for GPIO key support .
*   **Air Unit OTA**: For Over-The-Air (OTA) updates, hold the `REC` button on power-up to enter OTA mode, connect to the `espvtx` access point, and navigate to `http://192.168.4.1/ota` to upload `firmware.bin` .

### How to Run the Project Locally
#### Ground Station
After building, run the ground station software using the `launch.sh` script: `sudo /home/pi/esp32-cam-fpv/gs/launch.sh` <cite repo="RomanLut/hx-esp32-cam-fp

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

---
Generated using [Sidekick Dev]({REPO_URL}), your coding agent sidekick.
