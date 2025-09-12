#ifndef I2C_H
#define I2C_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "main.h" // For I2C_SLAVE_ADDR, I2C_CMD_FORWARD, etc.

#ifdef __cplusplus
extern "C" {
#endif

extern int s_uart_verbose;
extern bool s_i2c_initialized;

esp_err_t i2c_master_init();
void initialize_i2c();
void send_i2c_command(uint8_t command, int8_t val1 = 0, int8_t val2 = 0);

#define LOG(...) do { if (s_uart_verbose > 0) SAFE_PRINTF(__VA_ARGS__); } while (false) 

#ifdef __cplusplus
}
#endif

#endif // I2C_H
