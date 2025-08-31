#include "i2c_handler.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "I2C_SLAVE";

// Buffer para almacenar datos recibidos
uint8_t receivedData[BUF_SIZE];
int dataIndex = 0;

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
            .maximum_speed = 400000
        },
        .clk_flags = 0
    };
    
    esp_err_t ret = i2c_param_config(I2C_SLAVE_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_SLAVE_PORT, conf.mode, BUF_SIZE, BUF_SIZE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error instalando driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C Slave OK - Puerto:%d Addr:0x%02X SDA:%d SCL:%d", 
             I2C_SLAVE_PORT, SLAVE_ADDRESS, SDA_PIN, SCL_PIN);
    
    return ESP_OK;
}

void i2c_slave_task(void *pvParameter) {
    ESP_LOGI(TAG, "Slave listo para recibir datos...");
    
    while (1) {
        // Leer datos del master (operación de escritura del master)
        int size = i2c_slave_read_buffer(I2C_SLAVE_PORT, receivedData, BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (size > 0) {
            ESP_LOGI(TAG, "Datos recibidos (%d bytes): ", size);
            
            // Imprimir los datos en formato hexadecimal
            for (int i = 0; i < size; i++) {
                ESP_LOGI(TAG, "0x%02X ", receivedData[i]);
            }
            
            ESP_LOGI(TAG, ""); // Nueva línea
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
