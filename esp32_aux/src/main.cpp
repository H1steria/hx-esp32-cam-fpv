#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "i2c_handler.h"
#include "motor.h"
#include "dht11.h"

// I2C Command Definitions from components/air/main.h
#define I2C_CMD_FORWARD 1
#define I2C_CMD_BACKWARD 2
#define I2C_CMD_RIGHT 3
#define I2C_CMD_LEFT 4
#define I2C_CMD_FLASH 5

static const char *TAG = "MAIN";

// Motor pins configuration
#define IN1_PIN GPIO_NUM_14
#define IN2_PIN GPIO_NUM_27
#define IN3_PIN GPIO_NUM_26
#define IN4_PIN GPIO_NUM_25
#define ENA_PIN GPIO_NUM_33
#define ENB_PIN GPIO_NUM_32

// DHT11 sensor pin
#define DHT11_PIN GPIO_NUM_4

// DHT11 sensor instance
dht11_sensor_t dht11_sensor;

// Function to send DHT11 data via I2C
void send_dht11_data(float humidity, float temperature) {
    // For now, just log the data
    ESP_LOGI(TAG, "DHT11 Data - Humidity: %.1f%%, Temperature: %.1f°C", humidity, temperature);
    
    // TODO: Implement actual I2C sending to air unit
    // This would require implementing I2C master functionality
    // to send data to the air unit
}

// DHT11 reading task
void dht11_task(void *pvParameter) {
    float humidity, temperature;
    
    while (1) {
        // Read data from the sensor
        esp_err_t read_result = dht11_read(&dht11_sensor, &humidity, &temperature);
        
        if (read_result == ESP_OK) {
            // Print the values
            dht11_print_values(humidity, temperature);
            
            // Send data via I2C
            send_dht11_data(humidity, temperature);
        } else {
            ESP_LOGE(TAG, "Failed to read data from DHT11 sensor");
        }
        
        // Wait for 2 seconds before next reading
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main() {    
    // Inicializar I2C Slave
    if (init_i2c_slave() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando I2C Slave");
        return;
    }
    
    // Initialize motor controller
    Motor motor(IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN, ENA_PIN, ENB_PIN);
    
    // Initialize DHT11 sensor
    if (dht11_init(&dht11_sensor, DHT11_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DHT11 sensor");
    } else {
        ESP_LOGI(TAG, "DHT11 sensor initialized on GPIO %d", DHT11_PIN);
        
        // Create DHT11 reading task
        xTaskCreate(dht11_task, "dht11_task", 2048, NULL, 5, NULL);
    }
    
    ESP_LOGI(TAG, "Sistema iniciado - Esperando comandos I2C");
    
    // Process I2C commands
    while (1) {
        // Leer datos del master (operación de escritura del master)
        int size = i2c_slave_read_buffer(I2C_SLAVE_PORT, receivedData, BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (size > 0) {
            ESP_LOGI(TAG, "Datos recibidos (%d bytes): ", size);
            
            // Process each received byte as a command
            for (int i = 0; i < size; i++) {
                ESP_LOGI(TAG, "Comando recibido: 0x%02X", receivedData[i]);
                
                switch (receivedData[i]) {
                    case I2C_CMD_FORWARD:
                        motor.forward();
                        break;
                    case I2C_CMD_BACKWARD:
                        motor.backward();
                        break;
                    case I2C_CMD_RIGHT:
                        motor.right();
                        break;
                    case I2C_CMD_LEFT:
                        motor.left();
                        break;
                    case I2C_CMD_FLASH:
                        // Flash command - not implemented for motor control
                        ESP_LOGI(TAG, "Comando FLASH recibido - sin acción en motores");
                        break;
                    default:
                        // Unknown command - stop motors
                        motor.stop();
                        ESP_LOGW(TAG, "Comando desconocido: 0x%02X - deteniendo motores", receivedData[i]);
                        break;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
