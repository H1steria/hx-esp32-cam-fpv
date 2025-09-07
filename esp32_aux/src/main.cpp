#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "string.h"
#include "i2c_handler.h"
#include "motor.h"
#include "dht11.h"

// I2C Command Definitions from components/air/main.h
#define I2C_CMD_FORWARD 1
#define I2C_CMD_BACKWARD 2
#define I2C_CMD_RIGHT 3
#define I2C_CMD_LEFT 4
#define I2C_CMD_FLASH 5
#define I2C_CMD_GET_DHT11_DATA 6  // New command to request DHT11 data

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

// Global variables to store latest DHT11 readings
static float s_dht11_humidity = 0.0;
static float s_dht11_temperature = 0.0;
static bool s_dht11_data_valid = false;
static TickType_t s_last_dht11_read_time = 0;

// Function to send DHT11 data via I2C
void send_dht11_data(float humidity, float temperature) {
    // For now, just log the data
    ESP_LOGI(TAG, "DHT11 Data - Humidity: %.1f%%, Temperature: %.1f°C", humidity, temperature);
}

// DHT11 reading task
void dht11_task(void *pvParameter) {
    float humidity, temperature;
    
    ESP_LOGI(TAG, "DHT11 Task started");
    
    while (1) {
        TickType_t start_time = xTaskGetTickCount();
        ESP_LOGI(TAG, "DHT11 Task - Reading sensor data");
        
        // Read data from the sensor
        esp_err_t read_result = dht11_read(&dht11_sensor, &humidity, &temperature);
        
        if (read_result == ESP_OK) {
            // Print the values
            dht11_print_values(humidity, temperature);
            
            // Store data in global variables
            s_dht11_humidity = humidity;
            s_dht11_temperature = temperature;
            s_dht11_data_valid = true;
            s_last_dht11_read_time = xTaskGetTickCount();
            
            TickType_t end_time = xTaskGetTickCount();
            ESP_LOGI(TAG, "DHT11 Task - Read successful: H=%.1f%%, T=%.1f°C (took %d ms)", 
                     humidity, temperature, (int)((end_time - start_time) * portTICK_PERIOD_MS));
            
            // Send data via I2C
            send_dht11_data(humidity, temperature);
        } else {
            ESP_LOGE(TAG, "DHT11 Task - Failed to read data from DHT11 sensor: %s", esp_err_to_name(read_result));
            s_dht11_data_valid = false;
        }
        
        // Wait for 5 seconds before next reading (matching air unit's report interval)
        ESP_LOGI(TAG, "DHT11 Task - Waiting 5 seconds before next reading");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

// Function to send DHT11 data to I2C master
void send_dht11_data_to_master() {
    // Prepare data packet: [command_id, humidity_float_bytes(4), temperature_float_bytes(4), valid_flag]
    uint8_t data_packet[10];
    data_packet[0] = I2C_CMD_GET_DHT11_DATA;  // Command ID
    
    // Copy float values as bytes
    memcpy(&data_packet[1], &s_dht11_humidity, sizeof(float));
    memcpy(&data_packet[5], &s_dht11_temperature, sizeof(float));
    
    // Valid flag
    data_packet[9] = s_dht11_data_valid ? 1 : 0;
    
    // Send data packet
    // Note: In slave mode, we can't directly send data. The master will read from our buffer.
    // We'll store the data in the receivedData buffer for the master to read.
    memcpy(receivedData, data_packet, sizeof(data_packet));
    dataIndex = sizeof(data_packet);
    
    ESP_LOGI(TAG, "DHT11 I2C Response - Prepared data packet for master: H=%.1f%%, T=%.1f°C, Valid=%d", 
             s_dht11_humidity, s_dht11_temperature, s_dht11_data_valid);
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
                    case I2C_CMD_GET_DHT11_DATA:
                        // Send DHT11 data to master
                        send_dht11_data_to_master();
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
