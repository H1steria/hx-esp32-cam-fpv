#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

// Configuración I2C Slave
#define I2C_SLAVE_PORT I2C_NUM_1
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define SLAVE_ADDRESS 0x08
#define BUF_SIZE 128

// LED indicador
#define LED_PIN GPIO_NUM_2

static const char *TAG = "I2C_SLAVE";

// Variables globales
uint8_t ultimoResultado = 0;

void blink_led(int times) {
    for (int i = 0; i < times; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_PIN, 0);
        if (i < times - 1) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

esp_err_t init_gpio() {
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&led_config);
    if (ret == ESP_OK) {
        gpio_set_level(LED_PIN, 0);
        ESP_LOGI(TAG, "GPIO configurado - LED: GPIO%d", LED_PIN);
    }
    
    return ret;
}

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
            .maximum_speed = 100000
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

void slave_task(void *pvParameter) {
    uint8_t data_rd[BUF_SIZE];
    uint8_t data_wr[BUF_SIZE] = {0};  // Initialize buffer to zero
    
    ESP_LOGI(TAG, "Slave listo para recibir datos...");
    
    while (1) {
        // Leer datos del master (operación de escritura del master)
        int size = i2c_slave_read_buffer(I2C_SLAVE_PORT, data_rd, BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (size > 0) {
            if (size >= 2) {
                uint8_t numA = data_rd[0];
                uint8_t numB = data_rd[1];
                ultimoResultado = numA + numB;
                
                ESP_LOGI(TAG, "Recibido: %d + %d = %d", numA, numB, ultimoResultado);
                
                // Parpadear LED para indicar cálculo
                blink_led(2);
                
                // Preparar respuesta
                data_wr[0] = ultimoResultado;
                
                // Escribir resultado al buffer de I2C para que esté disponible para el master
                i2c_slave_write_buffer(I2C_SLAVE_PORT, data_wr, 1, pdMS_TO_TICKS(10));
            } else {
                ESP_LOGW(TAG, "Datos insuficientes recibidos: %d bytes", size);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void heartbeat_task(void *pvParameter) {
    while (1) {
        blink_led(1);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Parpadeo cada 2 segundos para indicar que está vivo
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "    I2C SLAVE - CALCULADORA SIMPLE");
    ESP_LOGI(TAG, "    Dirección: 0x%02X", SLAVE_ADDRESS);
    ESP_LOGI(TAG, "==========================================");
    
    // Inicializar GPIO
    if (init_gpio() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando GPIO");
        return;
    }
    
    // Inicializar I2C Slave
    if (init_i2c_slave() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando I2C Slave");
        return;
    }
    
    // Secuencia de inicio
    ESP_LOGI(TAG, "Sistema iniciando...");
    for (int i = 0; i < 3; i++) {
        blink_led(1);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    
    ESP_LOGI(TAG, "Calculadora lista!");
    ESP_LOGI(TAG, "Protocolo: [numA, numB] -> [resultado]");
    
    // Crear tareas
    xTaskCreate(slave_task, "slave_task", 4096, NULL, 6, NULL);
    xTaskCreate(heartbeat_task, "heartbeat_task", 2048, NULL, 2, NULL);
}
