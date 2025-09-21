#pragma once

#include <cassert>
#include <cstring>

// working pins:
// GPIO_NUM_2
// GPIO_NUM_4 - LED
// GPIO_NUM_12
// GPIO_NUM_13
// GPIO_NUM_14
// GPIO_NUM_15
// GPIO_NUM_16 - NO USE

//For esp32cam
#define GPIO_CONTROL_PIN GPIO_NUM_4

// I2C Pin Definitions
// #define I2C_SDA_PIN GPIO_NUM_2
// #define I2C_SCL_PIN GPIO_NUM_14

// I2C variables
// #define I2C_MASTER_NUM I2C_NUM_0   // I2C port number for master dev
// #define I2C_MASTER_FREQ_HZ 3400000  // I2C master clock frequency (400KHz for faster communication)
// #define I2C_MASTER_TX_BUF_DISABLE 0  // I2C master doesn't need buffer
// #define I2C_MASTER_RX_BUF_DISABLE 0  // I2C master doesn't need buffer
// #define I2C_SLAVE_ADDR 0x08        // I2C slave device address

// I2C Command Definitions
#define SPI_CMD_NONE 0
// #define I2C_CMD_FORWARD 1
// #define I2C_CMD_BACKWARD 2
// #define I2C_CMD_RIGHT 3
// #define I2C_CMD_LEFT 4
#define SPI_CMD_FLASH 5
#define SPI_CMD_JOYSTICK_MOVE 6

#define UART0_BAUDRATE 115200

extern bool isHQDVRMode();

extern bool SDError;
extern uint16_t SDTotalSpaceGB16;
extern uint16_t SDFreeSpaceGB16;
extern bool s_sd_initialized;

// void updateSDInfo();

#define MAX_SD_WRITE_SPEED_ESP32   (800*1024) //esp32 can hadle 1.9mb writes, but in this project it's 0.8mb max due to overal system load (otherwise we miss camera data callback)
#define MAX_SD_WRITE_SPEED_ESP32S3 (1800*1024) //can  write 1900 but we set to 1800 due to overal system load

//for development - enabled debug output on normal UART pins (1,3)
#define USBUART_DEBUG_OUTPUT

#define INIT_UART_0
#ifdef USBUART_DEBUG_OUTPUT
#define TXD0_PIN    1
#define RXD0_PIN    3
#endif

#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#else
#error "Camera model not selected"
#endif
