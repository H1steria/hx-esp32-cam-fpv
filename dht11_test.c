#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht11.h"

#define DHT11_PIN GPIO_NUM_4  // Change this to the GPIO pin you're using

void app_main() {
    dht11_sensor_t sensor;
    
    // Initialize the DHT11 sensor
    esp_err_t init_result = dht11_init(&sensor, DHT11_PIN);
    if (init_result != ESP_OK) {
        printf("Failed to initialize DHT11 sensor\n");
        return;
    }
    
    printf("DHT11 sensor initialized on GPIO %d\n", DHT11_PIN);
    
    // Main loop to read sensor data
    while (1) {
        float humidity = 0.0;
        float temperature = 0.0;
        
        // Read data from the sensor
        esp_err_t read_result = dht11_read(&sensor, &humidity, &temperature);
        
        if (read_result == ESP_OK) {
            // Print the values
            dht11_print_values(humidity, temperature);
        } else {
            printf("Failed to read data from DHT11 sensor\n");
        }
        
        // Wait for 2 seconds before next reading
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
