#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

// Configuración I2C Master
#define I2C_MASTER_PORT I2C_NUM_0
#define SDA_PIN GPIO_NUM_2
#define SCL_PIN GPIO_NUM_14
#define I2C_FREQ 100000
#define SLAVE_ADDRESS 0x08
#define I2C_TIMEOUT_MS 1000

static const char *TAG = "I2C_MASTER";

esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_FREQ
        },
        .clk_flags = 0
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error instalando driver I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C Master inicializado - SDA:%d SCL:%d Freq:%dHz", 
             SDA_PIN, SCL_PIN, I2C_FREQ);
    return ESP_OK;
}

esp_err_t i2c_test_connection() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

void i2c_scan_devices() {
    ESP_LOGI(TAG, "Escaneando dispositivos I2C...");
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Dispositivo encontrado en 0x%02X", addr);
        }
    }
}

esp_err_t i2c_send_sum_request(uint8_t numA, uint8_t numB) {
    // Enviar comando simple: [numA, numB]
    uint8_t data[2] = {numA, numB};
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 2, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Enviado: %d + %d", numA, numB);
    } else {
        ESP_LOGE(TAG, "✗ Error enviando suma: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t i2c_read_result(uint8_t* result) {
    // Intentar leer el resultado con reintentos para manejar el desfase de tiempo
    esp_err_t ret = ESP_FAIL;
    int retries = 3;
    
    for (int i = 0; i < retries; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, result, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Resultado leído: %d (intento %d)", *result, i+1);
            return ret;
        }
        
        // Esperar antes de reintentar
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGE(TAG, "✗ Error leyendo resultado después de %d intentos: %s", retries, esp_err_to_name(ret));
    return ret;
}

void master_task(void *pvParameter) {
    ESP_LOGI(TAG, "Tarea Master iniciada");
    
    // Escanear dispositivos
    i2c_scan_devices();
    
    // Esperar que el slave esté listo
    ESP_LOGI(TAG, "Buscando slave en dirección 0x%02X...", SLAVE_ADDRESS);
    bool slave_found = false;
    
    for (int i = 0; i < 20; i++) {
        if (i2c_test_connection() == ESP_OK) {
            ESP_LOGI(TAG, "✓ Slave encontrado en intento %d", i + 1);
            slave_found = true;
            break;
        }
        ESP_LOGW(TAG, "Intento %d: Slave no responde", i + 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    if (!slave_found) {
        ESP_LOGE(TAG, "No se pudo encontrar el slave. Verificar conexiones.");
        ESP_LOGI(TAG, "Continuando con intentos...");
    }
    
    uint16_t ciclo = 0;
    
    while (1) {
        ESP_LOGI(TAG, "\n=== Ciclo %d ===", ciclo);
        
        uint8_t numA = (ciclo % 10) + 1;
        uint8_t numB = ((ciclo + 3) % 15) + 1;
        uint8_t resultado = 0;
        
        // Verificar conexión
        if (i2c_test_connection() != ESP_OK) {
            ESP_LOGW(TAG, "Slave no disponible, esperando...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        
        ESP_LOGI(TAG, "Calculando: %d + %d", numA, numB);
        
        // Enviar números para sumar
        if (i2c_send_sum_request(numA, numB) == ESP_OK) {
            // Esperar procesamiento
            vTaskDelay(pdMS_TO_TICKS(200));
            
            // Leer resultado
            if (i2c_read_result(&resultado) == ESP_OK) {
                uint8_t esperado = numA + numB;
                
                if (resultado == esperado) {
                    ESP_LOGI(TAG, "✓ CORRECTO: %d + %d = %d", numA, numB, resultado);
                } else {
                    ESP_LOGW(TAG, "✗ ERROR: %d + %d = %d (esperaba %d)", 
                            numA, numB, resultado, esperado);
                }
            }
        }
        
        ciclo++;
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "    I2C MASTER - CALCULADORA SIMPLE");
    ESP_LOGI(TAG, "==========================================");
    
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error fatal inicializando I2C Master");
        return;
    }
    
    xTaskCreate(master_task, "master_task", 4096, NULL, 5, NULL);
}
