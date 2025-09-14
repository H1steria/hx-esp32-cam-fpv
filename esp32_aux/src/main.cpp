#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "string.h"
#include "i2c_handler.h"
#include "motor.h"
#include "dht11.h"

// I2C Command Definitions from components/air/main.h
#define I2C_CMD_NONE 0
#define I2C_CMD_FORWARD 1
#define I2C_CMD_BACKWARD 2
#define I2C_CMD_RIGHT 3
#define I2C_CMD_LEFT 4
#define I2C_CMD_FLASH 5
#define I2C_CMD_JOYSTICK_MOVE 6
static const char *TAG = "MAIN";

// Forward declaration for the command processing task
void i2c_command_processing_task(void *pvParameter);

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

// DHT11 reading task
void dht11_task(void *pvParameter) {
    float humidity, temperature;
    bool data_valid;
    
    ESP_LOGI(TAG, "DHT11 Task started");
    
    while (1) {
        TickType_t start_time = xTaskGetTickCount();
        ESP_LOGI(TAG, "DHT11 Task - Reading sensor data");
        
        // Read data from the sensor
        esp_err_t read_result = dht11_read(&dht11_sensor, &humidity, &temperature);
        
        if (read_result == ESP_OK) {
            // Print the values
            dht11_print_values(humidity, temperature);
            data_valid = true;
            
            TickType_t end_time = xTaskGetTickCount();
            ESP_LOGI(TAG, "DHT11 Task - Read successful: H=%.1f%%, T=%.1f°C (took %d ms)", 
                     humidity, temperature, (int)((end_time - start_time) * portTICK_PERIOD_MS));
        } else {
            ESP_LOGE(TAG, "DHT11 Task - Failed to read data from DHT11 sensor: %s", esp_err_to_name(read_result));
            data_valid = false;
        }
        
        // Update the I2C transmit buffer with the latest DHT11 data
        update_dht11_data_buffer(humidity, temperature, data_valid);
        i2c_slave_write_buffer(I2C_SLAVE_PORT, s_dht11_data_buffer, sizeof(s_dht11_data_buffer), pdMS_TO_TICKS(100));
        
        // Wait for 5 seconds before next reading (matching air unit's report interval)
        ESP_LOGI(TAG, "DHT11 Task - Waiting 5 seconds before next reading");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}


extern "C" void app_main() {    
    // Initialize I2C Slave
    if (init_i2c_slave() != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing I2C Slave");
        return;
    }

    // Initialize I2C command queue
    i2c_command_queue_init();
    
    // Initialize motor controller
    Motor* motor = new Motor(IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN, ENA_PIN, ENB_PIN);
    
    // Initialize DHT11 sensor
    if (dht11_init(&dht11_sensor, DHT11_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DHT11 sensor");
    } else {
        ESP_LOGI(TAG, "DHT11 sensor initialized on GPIO %d", DHT11_PIN);
        
        // Create DHT11 reading task
        xTaskCreate(dht11_task, "dht11_task", 4096, NULL, 5, NULL);
    }

    // Create I2C command processing task
    xTaskCreate(i2c_command_processing_task, "i2c_cmd_proc_task", 4096, motor, 8, NULL);
    
    ESP_LOGI(TAG, "System started - Waiting for I2C commands");
    
    // Process I2C commands
    while (1) {
        // Read data from master (write operation from master)
        int size = i2c_slave_read_buffer(I2C_SLAVE_PORT, receivedData, BUF_SIZE, pdMS_TO_TICKS(10)); // Increased timeout to reduce CPU usage and prevent watchdog timeout
        
        if (size > 0) {
            ESP_LOGI(TAG, "Received data (%d bytes): ", size);
            
            i2c_command_t cmd;
            cmd.command = receivedData[0];
            cmd.val1 = 0;
            cmd.val2 = 0;

            if (cmd.command == I2C_CMD_JOYSTICK_MOVE && size >= 3) {
                cmd.val1 = receivedData[1];
                cmd.val2 = receivedData[2];
                ESP_LOGI(TAG, "Received Joystick move command: x=%d, y=%d", cmd.val1, cmd.val2);
            } else {
                ESP_LOGI(TAG, "Received command: 0x%02X", cmd.command);
            }
            
            // Send command to the queue
            i2c_command_queue_send(cmd);
        } else {
            taskYIELD(); // Yield CPU if no data received to prevent watchdog timeout
        }
    }
}

// Task to process I2C commands from the queue
void i2c_command_processing_task(void *pvParameter) {
    Motor* motor = (Motor*)pvParameter;
    i2c_command_t cmd;

    ESP_LOGI(TAG, "I2C Command Processing Task started");

    while (1) {
        if (xQueueReceive(s_i2c_command_queue, &cmd, portMAX_DELAY) == pdPASS) {
            // Process the command
            switch (cmd.command) {
                case I2C_CMD_JOYSTICK_MOVE:
                    ESP_LOGI(TAG, "Processing Joystick move command: x=%d, y=%d", cmd.val1, cmd.val2);
                    motor->set_movement(cmd.val1, cmd.val2);
                    break;
                case I2C_CMD_FLASH:
                    ESP_LOGI(TAG, "Processing FLASH command - no action on motors");
                    break;
                case I2C_CMD_NONE:
                    motor->stop();
                    ESP_LOGI(TAG, "Processing Stop command - stopping motors");
                    break;
                default:
                    motor->stop();
                    ESP_LOGW(TAG, "Processing Unknown command: 0x%02X - stopping motors", cmd.command);
                    break;
            }
        }
    }
}
