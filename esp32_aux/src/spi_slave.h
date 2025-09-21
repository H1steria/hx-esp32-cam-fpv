#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include "esp_err.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" // For mutex if needed, though not strictly for this part

// SPI Slave Configuration
#define SPI_SLAVE_HOST SPI3_HOST
#define SPI_SLAVE_PIN_MOSI GPIO_NUM_18
#define SPI_SLAVE_PIN_MISO GPIO_NUM_19
#define SPI_SLAVE_PIN_SCLK GPIO_NUM_22
#define SPI_SLAVE_PIN_CS   GPIO_NUM_21
#define SPI_SLAVE_BUFFER_SIZE 16 // Must match master buffer size
#define MAX_SPI_SLAVE_TRANSACTIONS_IN_FLIGHT 3 // Number of transactions we can queue

// Command structure for SPI (same as master)
typedef struct {
    uint8_t command;
    int8_t val1;
    int8_t val2;
} aux_spi_command_t;

// DHT11 data structure for SPI (same as master)
typedef struct {
    float temperature;
    float humidity;
    uint8_t data_valid; // 1 if valid, 0 if not
} spi_dht11_data_t;

// Define a specific command for reading DHT11 data (same as master)
#define SPI_CMD_READ_DHT11 0x07

#ifdef __cplusplus
extern "C" {
#endif

// External queue for commands received via SPI
extern QueueHandle_t s_spi_command_queue;

// Global variable to store the latest DHT11 data to be sent to the master
extern spi_dht11_data_t s_dht11_data_to_send;
extern bool s_dht11_data_to_send_valid;

// Structure to hold a transaction and its buffers for the slave
typedef struct {
    spi_slave_transaction_t transaction;  // The actual ESP-IDF transaction structure
    uint8_t tx_buffer[SPI_SLAVE_BUFFER_SIZE];
    uint8_t rx_buffer[SPI_SLAVE_BUFFER_SIZE];
} custom_spi_slave_transaction_t;

esp_err_t spi_slave_init();
void spi_slave_task(void *pvParameter);
void spi_command_queue_init();
BaseType_t spi_command_queue_send(aux_spi_command_t command);
void update_spi_dht11_data(float humidity, float temperature, bool data_valid);

#ifdef __cplusplus
}
#endif

#endif // SPI_SLAVE_H
