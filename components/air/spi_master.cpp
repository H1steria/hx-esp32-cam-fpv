#include "spi_master.h"
#include "esp_log.h"
#include <string.h> // For memcpy
#include "freertos/queue.h"

static const char* TAG = "SPI_MASTER";
spi_device_handle_t spi_handle; // Removed static to make it accessible externally

// Transaction pool
static spi_master_transaction_t s_transaction_pool[MAX_SPI_MASTER_TRANSACTIONS_IN_FLIGHT];
static QueueHandle_t s_transaction_pool_queue; // Queue for available transactions

esp_err_t spi_master_init() {
    esp_err_t ret;

    // Create queues for transaction management
    s_transaction_pool_queue = xQueueCreate(MAX_SPI_MASTER_TRANSACTIONS_IN_FLIGHT, sizeof(spi_master_transaction_t*));

    if (s_transaction_pool_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create transaction pool queue");
        return ESP_FAIL;
    }

    // Initialize transaction pool
    for (int i = 0; i < MAX_SPI_MASTER_TRANSACTIONS_IN_FLIGHT; i++) {
        memset(&s_transaction_pool[i], 0, sizeof(spi_master_transaction_t));
        s_transaction_pool[i].trans.rx_buffer = s_transaction_pool[i].rx_buffer;
        s_transaction_pool[i].trans.tx_buffer = s_transaction_pool[i].tx_buffer;
        s_transaction_pool[i].trans.user = (void*)&s_transaction_pool[i]; // Store pointer to transaction in user field
        spi_master_transaction_t* trans_ptr_to_queue = &s_transaction_pool[i]; // Get the actual pointer to the transaction object
        xQueueSend(s_transaction_pool_queue, &trans_ptr_to_queue, portMAX_DELAY); // Send the address of the variable holding the pointer value
    }

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SPI_MASTER_PIN_MOSI;
    buscfg.miso_io_num = SPI_MASTER_PIN_MISO;
    buscfg.sclk_io_num = SPI_MASTER_PIN_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = SPI_MASTER_BUFFER_SIZE;

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = SPI_MASTER_CLOCK_SPEED_HZ;
    devcfg.mode = 0; // SPI mode 0
    devcfg.spics_io_num = SPI_MASTER_PIN_CS;
    devcfg.cs_ena_pretrans = 1; // Add 1 clock cycle before transaction to stabilize CS
    devcfg.cs_ena_posttrans = 1; // Add 1 clock cycle after transaction to stabilize CS
    devcfg.queue_size = MAX_SPI_MASTER_TRANSACTIONS_IN_FLIGHT;

    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI_MASTER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add the SPI device
    ret = spi_bus_add_device(SPI_MASTER_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create a task to handle completed SPI transactions
    if (xTaskCreate(spi_master_task, "spi_master_task", 8192, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SPI master task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "SPI Master initialized successfully at %d Hz", SPI_MASTER_CLOCK_SPEED_HZ);
    return ESP_OK;
}

// Helper function to get a transaction from the pool
static spi_master_transaction_t* get_transaction_from_pool() {
    spi_master_transaction_t* trans_obj;
    if (xQueueReceive(s_transaction_pool_queue, &trans_obj, portMAX_DELAY) == pdTRUE) {
        ESP_LOGD(TAG, "get_transaction_from_pool: Raw received trans_obj pointer: %p", trans_obj);
        if (trans_obj == NULL) {
            ESP_LOGE(TAG, "get_transaction_from_pool: Received NULL trans_obj from queue, but xQueueReceive returned pdTRUE!");
            return NULL;
        }
        memset(&trans_obj->trans, 0, sizeof(spi_transaction_t));
        memset(trans_obj->tx_buffer, 0, SPI_MASTER_BUFFER_SIZE);
        memset(trans_obj->rx_buffer, 0, SPI_MASTER_BUFFER_SIZE);
        trans_obj->trans.rx_buffer = trans_obj->rx_buffer;
        trans_obj->trans.tx_buffer = trans_obj->tx_buffer;
        trans_obj->trans.user = (void*)trans_obj;
        return trans_obj;
    }
    return NULL;
}

// Helper function to return a transaction to the pool
void return_transaction_to_pool(spi_master_transaction_t* trans_obj) { // Removed static to make it accessible externally
    if (trans_obj == NULL) {
        ESP_LOGE(TAG, "Attempted to return NULL transaction to pool!");
        return;
    }
    ESP_LOGD(TAG, "return_transaction_to_pool: Returning trans_obj pointer: %p", trans_obj); // Added log for returned pointer
    xQueueSend(s_transaction_pool_queue, &trans_obj, portMAX_DELAY); // Send the address of the pointer to the queue
}

IRAM_ATTR esp_err_t spi_master_send_control_command(const air_spi_command_t* cmd) {
    if (cmd == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_master_transaction_t* trans_obj = get_transaction_from_pool();
    if (trans_obj == NULL) {
        ESP_LOGE(TAG, "Failed to get transaction from pool for control command");
        return ESP_FAIL;
    }

    memcpy(trans_obj->tx_buffer, cmd, sizeof(air_spi_command_t));
    trans_obj->trans.length = SPI_MASTER_BUFFER_SIZE * 8; // Always send/receive full buffer
    
    ESP_LOGD(TAG, "Queuing control command: cmd=0x%02X, val1=%d, val2=%d", cmd->command, cmd->val1, cmd->val2);
    esp_err_t ret = spi_device_queue_trans(spi_handle, &trans_obj->trans, pdMS_TO_TICKS(100)); // Use a timeout
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to queue SPI control command (timeout/error): %s", esp_err_to_name(ret));
        return_transaction_to_pool(trans_obj);
    }
    return ret;
}

IRAM_ATTR spi_master_transaction_t* spi_master_send_read_dht11_command() {
    spi_master_transaction_t* trans_obj = get_transaction_from_pool();
    if (trans_obj == NULL) {
        ESP_LOGE(TAG, "Failed to get transaction from pool for DHT11 read command");
        return NULL; // Return NULL to indicate failure
    }

    air_spi_command_t read_dht_cmd = { .command = SPI_CMD_READ_DHT11, .val1 = 0, .val2 = 0 };
    memcpy(trans_obj->tx_buffer, &read_dht_cmd, sizeof(air_spi_command_t));
    trans_obj->trans.length = SPI_MASTER_BUFFER_SIZE * 8; // Always send/receive full buffer

    ESP_LOGD(TAG, "Queuing read DHT11 command (0x%02X)", SPI_CMD_READ_DHT11);
    esp_err_t ret = spi_device_queue_trans(spi_handle, &trans_obj->trans, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to queue SPI DHT11 read command (timeout/error): %s", esp_err_to_name(ret));
        return_transaction_to_pool(trans_obj);
        return NULL; // Return NULL to indicate failure
    }
    return trans_obj; // Return the transaction object for later retrieval of results
}

// spi_master_get_latest_dht11_data is no longer needed

void spi_master_task(void *pvParameter) {
    spi_transaction_t* r_trans;
    spi_master_transaction_t* trans_obj;
    esp_err_t ret;

    while (1) {
        ret = spi_device_get_trans_result(spi_handle, &r_trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get completed SPI transaction: %s", esp_err_to_name(ret));
            continue;
        }

        trans_obj = (spi_master_transaction_t*)r_trans->user;

        if (trans_obj == NULL) {
            ESP_LOGE(TAG, "spi_master_task: Received NULL trans_obj from completed transaction!");
            continue; // Skip processing this invalid transaction
        }
        
        // Check if this was a DHT11 read command
        air_spi_command_t cmd;
        memcpy(&cmd, trans_obj->tx_buffer, sizeof(air_spi_command_t));

        if (cmd.command == SPI_CMD_READ_DHT11) {
            spi_dht11_data_t dht11_data;
            bool data_valid = false;

            if (trans_obj->trans.rx_buffer != nullptr && trans_obj->trans.length > 0) {
                memcpy(&dht11_data, trans_obj->rx_buffer, sizeof(spi_dht11_data_t));
                data_valid = (dht11_data.data_valid == 1);
            }

            // Update global DHT11 data variables
            s_dht11_humidity = dht11_data.humidity;
            s_dht11_temperature = dht11_data.temperature;
            s_dht11_data_valid = data_valid;

            ESP_LOGI(TAG, "DHT11 data updated - Humidity: %.1f%%, Temperature: %.1f°C, Valid: %d", 
                     s_dht11_humidity, s_dht11_temperature, (int)s_dht11_data_valid);
        }
        
        return_transaction_to_pool(trans_obj);
    }
}
