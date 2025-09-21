# SPI Communication Migration: Air Unit (Master) and Auxiliary Unit (Slave)

This document details the migration from I2C to SPI communication between the Air Unit (ESP32-CAM) and the Auxiliary Unit (ESP32). The goal is to replace the existing I2C protocol with SPI, maintaining a 7MHz clock speed for efficient data transfer.

## 1. Overview of Changes

The core communication mechanism for control commands (from Air Unit to Aux Unit) and sensor data (DHT11 readings from Aux Unit to Air Unit) will transition from I2C to SPI. This involves:
*   Creating new SPI master and slave libraries.
*   Updating the Air Unit firmware (`components/air/air_main.cpp`) to use the SPI master.
*   Updating the Auxiliary Unit firmware (`esp32_aux/src/aux_main.cpp`) to use the SPI slave.
*   Adjusting build configurations (`CMakeLists.txt`) to include the new SPI components and remove the old I2C ones.

## 2. SPI Pin Assignments

To ensure clear and dedicated communication, the following GPIO pins will be used for SPI:

### Air Unit (Master)
*   **SPI Host:** SPI3_HOST
*   **MOSI (Master Out Slave In):** GPIO13
*   **MISO (Master In Slave Out):** GPIO15
*   **SCLK (Serial Clock):** GPIO14
*   **CS (Chip Select):** GPIO2

### Auxiliary Unit (Slave)
*   **SPI Host:** SPI3_HOST
*   **MOSI (Master Out Slave In):** GPIO18
*   **MISO (Master In Slave Out):** GPIO19
*   **SCLK (Serial Clock):** GPIO22
*   **CS (Chip Select):** GPIO21

## 3. SPI Configuration Parameters

The SPI bus will be configured to operate at 7MHz, as specified, using SPI Mode 0. DMA will be enabled for efficient data transfers.

*   **Clock Speed:** 7 * 1000 * 1000 Hz (7 MHz)
*   **SPI Mode:** 0 (CPOL=0, CPHA=0)
*   **DMA Channel:** SPI_DMA_CH_AUTO
*   **Buffer Size:** 128 bytes (for commands and DHT11 data)
*   **CS Stabilization:** `cs_ena_pretrans = 1` and `cs_ena_posttrans = 1` will be used for the master to stabilize the Chip Select line.

## 4. Data Structures for SPI Communication

To maintain compatibility with the existing data flow, the following structures will be used for SPI transactions:

### `spi_command_t` (for control commands from Air Unit to Aux Unit)
```c++
typedef struct {
    uint8_t command; // e.g., I2C_CMD_FLASH, I2C_CMD_JOYSTICK_MOVE
    int8_t val1;     // e.g., joystick_x
    int8_t val2;     // e.g., joystick_y
} spi_command_t;
```

### `spi_dht11_data_t` (for DHT11 sensor data from Aux Unit to Air Unit)
```c++
typedef struct {
    float temperature;
    float humidity;
    uint8_t data_valid; // 1 if data is valid, 0 otherwise
    uint8_t reserved[5]; // Padding to ensure a consistent size, e.g., 10 bytes total
} spi_dht11_data_t;
```
The `reserved` field ensures the structure size is consistent, which is important for fixed-size SPI transfers.

## 5. Functional Logic Preservation

The existing communication logic will be preserved:

*   **Air Unit (Master):**
    *   Will periodically initiate SPI transactions to read DHT11 data from the Auxiliary Unit.
    *   When a control command is received (e.g., from the Ground Station via WiFi), it will immediately send this command to the Auxiliary Unit via SPI.
*   **Auxiliary Unit (Slave):**
    *   Will continuously listen for SPI transactions from the Master.
    *   Upon receiving a command, it will process it (e.g., control motors, toggle flash).
    *   In parallel, a dedicated task will periodically read DHT11 sensor data and update an internal buffer, making it available for the Master to read during its periodic requests.

## 6. New SPI Library Files

### `components/air/spi_master.h`
This header will define the SPI master configuration, `spi_command_t`, `spi_dht11_data_t`, and function prototypes for `spi_master_init()`, `spi_master_send_command()`, and `spi_master_read_dht11_data()`.

### `components/air/spi_master.cpp`
This file will contain the implementation of the SPI master functions, including bus initialization, device addition, and transaction handling using `spi_device_queue_trans` and `spi_device_get_trans_result`.

### `esp32_aux/src/spi_slave.h`
This header will define the SPI slave configuration, `spi_command_t`, `spi_dht11_data_t`, and function prototypes for `spi_slave_init()`, `spi_slave_task()`, `spi_command_queue_init()`, `spi_command_queue_send()`, and `update_spi_dht11_data()`.

### `esp32_aux/src/spi_slave.cpp`
This file will implement the SPI slave functions, including bus initialization, slave device configuration, and a dedicated task (`spi_slave_task`) to handle incoming SPI transactions and prepare responses.

## 7. Firmware Modifications

### `components/air/air_main.cpp`
*   Remove `#include "i2c.h"` and calls to `initialize_i2c()`.
*   Add `#include "spi_master.h"`.
*   Replace `initialize_i2c()` with `spi_master_init()`.
*   Update `handle_ground2air_control_packet` to use `spi_master_send_command`.
*   Update `handle_dht11_read_and_send` to use `spi_master_read_dht11_data`.

### `esp32_aux/src/aux_main.cpp`
*   Remove `#include "i2c_handler.h"` and calls to `init_i2c_slave()`.
*   Add `#include "spi_slave.h"`.
*   Replace `init_i2c_slave()` with `spi_slave_init()`.
*   Replace `i2c_command_queue_init()` with `spi_command_queue_init()`.
*   Modify `i2c_command_processing_task` to receive from `s_spi_command_queue`.
*   Replace the main loop's I2C slave read logic with the `spi_slave_task` to handle SPI communication.
*   Update `dht11_task` to call `update_spi_dht11_data`.

## 8. Build System Updates (`CMakeLists.txt`)

*   **`components/air/CMakeLists.txt`:**
    *   Remove `i2c.cpp` from the `SRCS` list.
    *   Add `spi_master.cpp` to the `SRCS` list.
*   **`esp32_aux/CMakeLists.txt`:**
    *   Remove `i2c_handler.cpp` from the `SRCS` list.
    *   Add `spi_slave.cpp` to the `SRCS` list.

This comprehensive plan ensures a smooth and well-documented transition to SPI communication, addressing all specified requirements.
