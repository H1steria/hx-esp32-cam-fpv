#include "i2c_handler.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "string.h"
#include "freertos/queue.h"
#include "driver/i2c.h"

static const char *TAG = "I2C_SLAVE";

// Buffer para almacenar datos recibidos
uint8_t receivedData[BUF_SIZE];
int dataIndex = 0;

// FreeRTOS queue for I2C commands
QueueHandle_t s_i2c_command_queue;

// DHT11 data buffer
uint8_t s_dht11_data_buffer[9]; // 4 bytes humidity + 4 bytes temperature + 1 byte valid flag

esp_err_t init_i2c_slave() {
    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .addr_10bit_en = 0,
            .slave_addr = SLAVE_ADDRESS,
            .maximum_speed = 3400000
        },
        .clk_flags = 0
    };
    
    esp_err_t ret = i2c_param_config(I2C_SLAVE_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Install driver with a larger RX buffer for read operations to prevent overflow
    // The TX buffer can remain BUF_SIZE as the slave typically doesn't send large amounts of data.
    ret = i2c_driver_install(I2C_SLAVE_PORT, conf.mode, 512, BUF_SIZE, 0); // Corrected i2c_driver_install call (5 arguments)
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error instalando driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C Slave OK - Puerto:%d Addr:0x%02X SDA:%d SCL:%d", 
             I2C_SLAVE_PORT, SLAVE_ADDRESS, SDA_PIN, SCL_PIN);
    
    return ESP_OK;
}

// Function to initialize the I2C command queue
void i2c_command_queue_init() {
    s_i2c_command_queue = xQueueCreate(30, sizeof(i2c_command_t)); // Increased queue size to 30 commands
    if (s_i2c_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command queue");
    } else {
        ESP_LOGI(TAG, "I2C command queue created successfully");
    }
}

// Function to send a command to the queue
BaseType_t i2c_command_queue_send(i2c_command_t command) {
    if (s_i2c_command_queue == NULL) {
        ESP_LOGE(TAG, "I2C command queue not initialized");
        return pdFAIL;
    }
    BaseType_t ret = xQueueSend(s_i2c_command_queue, &command, 0); // Don't block if queue is full
    if (ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to send command to I2C queue (queue full?)");
    }
    return ret;
}

// Function to update DHT11 data in the buffer for transmission
void update_dht11_data_buffer(float humidity, float temperature, bool data_valid) {
    // Copy float values as bytes to the buffer
    memcpy(&s_dht11_data_buffer[0], &humidity, sizeof(float));
    memcpy(&s_dht11_data_buffer[4], &temperature, sizeof(float));
    s_dht11_data_buffer[8] = data_valid ? 1 : 0;
    
    ESP_LOGI(TAG, "DHT11 data buffer updated - H=%.1f%%, T=%.1f°C, Valid=%d", 
             humidity, temperature, data_valid);
}
