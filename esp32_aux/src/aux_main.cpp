#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "string.h"
#include "motor.h"
#include "dht11.h"
#include "spi_slave.h"

// SPI Command Definitions from components/air/spi_master.h
#define SPI_CMD_NONE 0
#define SPI_CMD_FORWARD 1
#define SPI_CMD_BACKWARD 2
#define SPI_CMD_RIGHT 3
#define SPI_CMD_LEFT 4
#define SPI_CMD_FLASH 5
#define SPI_CMD_JOYSTICK_MOVE 6
// SPI_CMD_READ_DHT11 is already defined in spi_slave.h as 0x07

static const char *TAG = "MAIN";

// Forward declaration for the command processing task
void spi_command_processing_task(void *pvParameter);

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
        
        // Update the SPI slave's DHT11 data buffer
        update_spi_dht11_data(humidity, temperature, data_valid);
        
        // Wait for 5 seconds before next reading (matching air unit's report interval)
        ESP_LOGI(TAG, "DHT11 Task - Waiting 5 seconds before next reading");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}


extern "C" void app_main() {
    // Initialize SPI Slave
    if (spi_slave_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing SPI Slave");
        return;
    }

    // Initialize SPI command queue
    spi_command_queue_init();
    
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

    // Create SPI command processing task
    xTaskCreate(spi_command_processing_task, "spi_cmd_proc_task", 4096, motor, 8, NULL);
    
    ESP_LOGI(TAG, "System started - Waiting for SPI commands");
    
    // Start the SPI slave task to handle communication
    xTaskCreate(spi_slave_task, "spi_slave_task", 4096, NULL, 10, NULL);

    // The main loop can now be simplified or removed if no other main thread tasks are needed
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Keep main task alive, or remove if not needed
    }
}

// Task to process SPI commands from the queue
void spi_command_processing_task(void *pvParameter) {
    Motor* motor = (Motor*)pvParameter;
    aux_spi_command_t cmd;

    ESP_LOGI(TAG, "SPI Command Processing Task started");

    while (1) {
        if (xQueueReceive(s_spi_command_queue, &cmd, portMAX_DELAY) == pdPASS) {
            // Process the command
            switch (cmd.command) {
                case SPI_CMD_JOYSTICK_MOVE:
                    ESP_LOGI(TAG, "Processing Joystick move command: x=%d, y=%d", cmd.val1, cmd.val2);
                    motor->set_movement(cmd.val1, cmd.val2);
                    break;
                case SPI_CMD_FLASH:
                    ESP_LOGI(TAG, "Processing FLASH command - no action on motors");
                    break;
                case SPI_CMD_NONE:
                    motor->stop();
                    ESP_LOGI(TAG, "Processing Stop command - stopping motors");
                    break;
                case SPI_CMD_READ_DHT11:
                    ESP_LOGI(TAG, "Processing READ_DHT11 command - data already updated by dht11_task");
                    break;
                default:
                    motor->stop();
                    uint8_t unknown_cmd_val = cmd.command; // Store command in a local variable
                    ESP_LOGW(TAG, "Processing Unknown command: 0x%02X - stopping motors", (unsigned int)unknown_cmd_val);
                    break;
            }
        }
    }
}
