#ifndef DHT11_H
#define DHT11_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "dht.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DHT11 sensor class
 */
typedef struct {
    gpio_num_t pin;          /*!< GPIO pin connected to DHT11 data pin */
    dht_sensor_type_t type;  /*!< Type of DHT sensor (DHT_TYPE_DHT11) */
} dht11_sensor_t;

/**
 * @brief Initialize DHT11 sensor
 *
 * @param sensor Pointer to DHT11 sensor structure
 * @param pin GPIO pin number
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t dht11_init(dht11_sensor_t* sensor, gpio_num_t pin);

/**
 * @brief Read temperature and humidity from DHT11 sensor
 *
 * @param sensor Pointer to DHT11 sensor structure
 * @param humidity Pointer to store humidity value
 * @param temperature Pointer to store temperature value
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t dht11_read(dht11_sensor_t* sensor, float* humidity, float* temperature);

/**
 * @brief Print temperature and humidity values
 *
 * @param humidity Humidity value to print
 * @param temperature Temperature value to print
 */
void dht11_print_values(float humidity, float temperature);

#ifdef __cplusplus
}
#endif

#endif /* DHT11_H */
