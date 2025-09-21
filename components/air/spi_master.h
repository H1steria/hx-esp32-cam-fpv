#ifndef SPI_MASTER_H
#define SPI_MASTER_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// SPI Master Configuration
#define SPI_MASTER_HOST SPI3_HOST
#define SPI_MASTER_PIN_MOSI GPIO_NUM_13
#define SPI_MASTER_PIN_MISO GPIO_NUM_15
#define SPI_MASTER_PIN_SCLK GPIO_NUM_14
#define SPI_MASTER_PIN_CS   GPIO_NUM_2
#define SPI_MASTER_CLOCK_SPEED_HZ (7 * 1000 * 1000) // 7 MHz
#define SPI_MASTER_BUFFER_SIZE 16 // Max size for commands (3 bytes) and DHT11 data (9 bytes)
#define MAX_SPI_MASTER_TRANSACTIONS_IN_FLIGHT 3 // Number of transactions we can queue

// Command structure for SPI (similar to i2c_command_t)
typedef struct {
    uint8_t command;
    int8_t val1;
    int8_t val2;
} air_spi_command_t;

// DHT11 data structure for SPI
typedef struct {
    float temperature;
    float humidity;
    uint8_t data_valid; // 1 if valid, 0 if not
} spi_dht11_data_t;

// Define a specific command for reading DHT11 data
#define SPI_CMD_READ_DHT11 0x07 // Using 0x07 as a new command for reading DHT11

// Extern declarations for global DHT11 data variables (defined in air_main.cpp)
extern float s_dht11_humidity;
extern float s_dht11_temperature;
extern bool s_dht11_data_valid;

// Structure to hold a transaction and its buffers
typedef struct {
    spi_transaction_t trans;
    uint8_t tx_buffer[SPI_MASTER_BUFFER_SIZE];
    uint8_t rx_buffer[SPI_MASTER_BUFFER_SIZE];
} spi_master_transaction_t;

// Extern declarations for spi_handle and return_transaction_to_pool
extern spi_device_handle_t spi_handle;
extern void return_transaction_to_pool(spi_master_transaction_t* trans_obj);

esp_err_t spi_master_init();
esp_err_t spi_master_send_control_command(const air_spi_command_t* cmd);
spi_master_transaction_t* spi_master_send_read_dht11_command();
void spi_master_task(void *pvParameter); // New task to handle completed transactions

#ifdef __cplusplus
}
#endif

#endif // SPI_MASTER_H
