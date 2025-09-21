#include "spi_slave.h"
#include "esp_log.h"
#include <string.h> // For memcpy
#include "freertos/task.h" // For vTaskDelay (though it will be removed from the main loop)
#include "freertos/semphr.h" // For mutex

static const char* TAG = "SPI_SLAVE";

// Global queue for commands received via SPI
QueueHandle_t s_spi_command_queue;

// Global variables for DHT11 data to be sent to the master
spi_dht11_data_t s_dht11_data_to_send = { .temperature = 0.0f, .humidity = 0.0f, .data_valid = 0 };
bool s_dht11_data_to_send_valid = false;
SemaphoreHandle_t s_dht11_data_to_send_mutex; // Mutex to protect DHT11 data

// Transaction pool for slave
static custom_spi_slave_transaction_t s_slave_transaction_pool[MAX_SPI_SLAVE_TRANSACTIONS_IN_FLIGHT];
static QueueHandle_t s_slave_transaction_pool_queue; // Queue for available transactions

esp_err_t spi_slave_init() {
    esp_err_t ret;

    // Create mutex for DHT11 data to send
    s_dht11_data_to_send_mutex = xSemaphoreCreateMutex();
    if (s_dht11_data_to_send_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create DHT11 data to send mutex");
        return ESP_FAIL;
    }

    // Create queue for transaction pool
    s_slave_transaction_pool_queue = xQueueCreate(MAX_SPI_SLAVE_TRANSACTIONS_IN_FLIGHT, sizeof(custom_spi_slave_transaction_t*));
    if (s_slave_transaction_pool_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create slave transaction pool queue");
        return ESP_FAIL;
    }

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SPI_SLAVE_PIN_MOSI;
    buscfg.miso_io_num = SPI_SLAVE_PIN_MISO;
    buscfg.sclk_io_num = SPI_SLAVE_PIN_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = SPI_SLAVE_BUFFER_SIZE;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.spics_io_num = SPI_SLAVE_PIN_CS;
    slvcfg.flags = 0;
    slvcfg.queue_size = MAX_SPI_SLAVE_TRANSACTIONS_IN_FLIGHT;
    slvcfg.mode = 0; // SPI mode 0
    slvcfg.post_setup_cb = NULL;
    slvcfg.post_trans_cb = NULL;

    // Initialize the SPI bus in slave mode
    ret = spi_slave_initialize(SPI_SLAVE_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize transaction pool and queue initial transactions
    for (int i = 0; i < MAX_SPI_SLAVE_TRANSACTIONS_IN_FLIGHT; i++) {
        memset(&s_slave_transaction_pool[i], 0, sizeof(custom_spi_slave_transaction_t));
        s_slave_transaction_pool[i].transaction.length = SPI_SLAVE_BUFFER_SIZE * 8;
        s_slave_transaction_pool[i].transaction.tx_buffer = s_slave_transaction_pool[i].tx_buffer;
        s_slave_transaction_pool[i].transaction.rx_buffer = s_slave_transaction_pool[i].rx_buffer;
        s_slave_transaction_pool[i].transaction.user = (void*)&s_slave_transaction_pool[i]; // Store pointer to transaction in user field

        // Prepare the transmit buffer with initial DHT11 data
        if (xSemaphoreTake(s_dht11_data_to_send_mutex, portMAX_DELAY) == pdTRUE) {
            memcpy((void*)s_slave_transaction_pool[i].tx_buffer, &s_dht11_data_to_send, sizeof(spi_dht11_data_t));
            xSemaphoreGive(s_dht11_data_to_send_mutex);
        } else {
            ESP_LOGE(TAG, "Failed to take DHT11 data to send mutex during init");
        }
        
        // Queue the transaction to the SPI slave driver
        ret = spi_slave_queue_trans(SPI_SLAVE_HOST, &s_slave_transaction_pool[i].transaction, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to queue initial SPI slave transaction %d: %s", i, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGD(TAG, "Queued initial SPI slave transaction %d", i);
    }

    ESP_LOGI(TAG, "SPI Slave initialized successfully");
    return ESP_OK;
}

void spi_command_queue_init() {
    s_spi_command_queue = xQueueCreate(10, sizeof(aux_spi_command_t));
    if (s_spi_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create SPI command queue");
    } else {
        ESP_LOGI(TAG, "SPI command queue created");
    }
}

BaseType_t spi_command_queue_send(aux_spi_command_t command) {
    if (s_spi_command_queue == NULL) {
        ESP_LOGE(TAG, "SPI command queue not initialized");
        return pdFAIL;
    }
    return xQueueSend(s_spi_command_queue, &command, pdMS_TO_TICKS(10));
}

void update_spi_dht11_data(float humidity, float temperature, bool data_valid) {
    if (xSemaphoreTake(s_dht11_data_to_send_mutex, portMAX_DELAY) == pdTRUE) {
        s_dht11_data_to_send.humidity = humidity;
        s_dht11_data_to_send.temperature = temperature;
        s_dht11_data_to_send.data_valid = data_valid ? 1 : 0;
        s_dht11_data_to_send_valid = data_valid;
        ESP_LOGD(TAG, "DHT11 data updated for SPI: Temp=%.1f, Hum=%.1f, Valid=%d",
                 s_dht11_data_to_send.temperature, s_dht11_data_to_send.humidity, s_dht11_data_to_send.data_valid);
        xSemaphoreGive(s_dht11_data_to_send_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take DHT11 data to send mutex for update");
    }
}

void spi_slave_task(void *pvParameter) {
    ESP_LOGI(TAG, "SPI Slave Task started");
    custom_spi_slave_transaction_t* trans_obj;
    esp_err_t ret;

    while (1) {
        // Wait for a completed transaction
        spi_slave_transaction_t* completed_trans;
        ret = spi_slave_get_trans_result(SPI_SLAVE_HOST, &completed_trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get completed SPI slave transaction: %s", esp_err_to_name(ret));
            continue;
        }

        // Find our custom transaction object from the completed transaction
        // We stored a pointer to our custom object in the user field
        trans_obj = (custom_spi_slave_transaction_t*)completed_trans->user;

        // Process received command from master
        aux_spi_command_t received_cmd;
        memcpy(&received_cmd, trans_obj->rx_buffer, sizeof(aux_spi_command_t));

        if (received_cmd.command != 0x00) { // Assuming 0x00 is a no-op or empty command
            uint8_t cmd_val = received_cmd.command;
            int8_t val1_val = received_cmd.val1;
            int8_t val2_val = received_cmd.val2;
            ESP_LOGI(TAG, "Received SPI command: cmd=0x%02X, val1=%d, val2=%d", cmd_val, (int)val1_val, (int)val2_val);
            spi_command_queue_send(received_cmd);
        } else {
            ESP_LOGD(TAG, "Received no-op SPI command (0x00)");
        }

        // Prepare the transmit buffer for the next transaction with the latest DHT11 data
        if (xSemaphoreTake(s_dht11_data_to_send_mutex, portMAX_DELAY) == pdTRUE) {
            memcpy((void*)trans_obj->tx_buffer, &s_dht11_data_to_send, sizeof(spi_dht11_data_t));
            // Fill remaining with zeros if needed
            if (sizeof(spi_dht11_data_t) < SPI_SLAVE_BUFFER_SIZE) {
                memset((void*)(trans_obj->tx_buffer + sizeof(spi_dht11_data_t)), 0, SPI_SLAVE_BUFFER_SIZE - sizeof(spi_dht11_data_t));
            }
            xSemaphoreGive(s_dht11_data_to_send_mutex);
        } else {
            ESP_LOGE(TAG, "Failed to take DHT11 data to send mutex in task");
        }

        // Re-queue the transaction to the SPI slave driver to prepare for the next master transaction
        ret = spi_slave_queue_trans(SPI_SLAVE_HOST, &trans_obj->transaction, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to re-queue SPI slave transaction: %s", esp_err_to_name(ret));
            // If re-queueing fails, we might need to handle this more robustly,
            // e.g., by trying to re-initialize or log a critical error.
        }
    }
}
