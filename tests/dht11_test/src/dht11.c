#include "dht11.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DHT11";

esp_err_t dht11_init(dht11_sensor_t* sensor, gpio_num_t pin) {
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    sensor->pin = pin;
    sensor->type = DHT_TYPE_DHT11;
    
    return ESP_OK;
}

esp_err_t dht11_read(dht11_sensor_t* sensor, float* humidity, float* temperature) {
    if (sensor == NULL || humidity == NULL || temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int16_t humidity_int = 0;
    int16_t temperature_int = 0;
    
    esp_err_t result = dht_read_data(sensor->type, sensor->pin, &humidity_int, &temperature_int);
    
    if (result == ESP_OK) {
        *humidity = (float)humidity_int / 10.0;
        *temperature = (float)temperature_int / 10.0;
    } else {
        ESP_LOGE(TAG, "Failed to read data from DHT11 sensor");
    }
    
    return result;
}

void dht11_print_values(float humidity, float temperature) {
    printf("Humidity: %.1f%%\n", humidity);
    printf("Temperature: %.1f°C\n", temperature);
}
