# GPIO Control Implementation Summary

## Overview
This implementation adds GPIO control functionality to the ESP32 firmware based on button presses in the Ground Station UI. When the GPIO toggle button is pressed in the Ground Station, it increments a counter that triggers the ESP32 to toggle a GPIO pin.

## Implementation Details

### 1. GPIO Pin Definition
- **Pin**: GPIO_NUM_15 (available on ESP32-CAM board)
- **Function**: Digital output for external device control
- **Initial State**: OFF (LOW)

### 2. Files Modified

#### `components/air/main.h`
- Added `#define GPIO_CONTROL_PIN GPIO_NUM_15` for GPIO control pin definition

#### `components/air/main.cpp`
- **State Variable**: `static bool s_gpio_control_state = false` tracks current GPIO state
- **Initialization Function**: `initialize_gpio_control_pin()` configures GPIO_NUM_15 as output
- **Control Function**: `set_gpio_control_pin(bool enabled)` sets GPIO high/low
- **Configuration Handler**: Added gpio_control_btn processing in `handle_ground2air_config_packetEx1()`

### 3. Logic Flow
1. **Ground Station**: Button press increments `config.dataChannel.gpio_control_btn`
2. **ESP32**: Receives configuration packet via `Ground2Air_Config_Packet`
3. **Handler**: `handle_ground2air_config_packetEx1()` detects counter change
4. **Action**: Toggles GPIO pin state and logs status

### 4. Communication Protocol
- **Packet Structure**: `Ground2Air_Config_Packet.dataChannel.gpio_control_btn`
- **Trigger Mechanism**: Counter increment detection
- **State Toggle**: Each button press toggles GPIO state (ON→OFF→ON...)



working pins:
GPIO_NUM_2
GPIO_NUM_4 - LED
GPIO_NUM_12
GPIO_NUM_13
GPIO_NUM_14
GPIO_NUM_15

GPIO_NUM_16 - NO USAR