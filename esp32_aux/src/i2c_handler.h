#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/i2c.h"

// Configuración I2C Slave
#define I2C_SLAVE_PORT I2C_NUM_0
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define SLAVE_ADDRESS 0x08
#define BUF_SIZE 128

// Buffer para almacenar datos recibidos
extern uint8_t receivedData[BUF_SIZE];
extern int dataIndex;

// DHT11 data buffer
extern uint8_t s_dht11_data_buffer[9]; // 4 bytes humidity + 4 bytes temperature + 1 byte valid flag

// Function declarations
esp_err_t init_i2c_slave();
void i2c_slave_task(void *pvParameter);
void update_dht11_data_buffer(float humidity, float temperature, bool data_valid);

#endif // I2C_HANDLER_H
