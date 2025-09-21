#include <iostream>
#include <string>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

// --- Configuración de Hardware ---
#define SPI_HOST SPI3_HOST
#define PIN_NUM_MOSI 18
#define PIN_NUM_MISO 19
#define PIN_NUM_SCLK 22
#define PIN_NUM_CS   21

// --- Configuración de la Simulación ---
#define BUFFER_SIZE 1024 // Debe coincidir con el FRAME_BUFFER_SIZE del maestro

static const char* TAG = "SpiSlaveTask";

// Pool de búferes para evitar corrupción durante las transacciones
#define NUM_BUFFERS 3

typedef struct {
    char* tx_buffer;
    char* rx_buffer;
    bool in_use;
} buffer_pair_t;

static buffer_pair_t buffer_pool[NUM_BUFFERS];

// Función para obtener un par de búferes disponibles
static buffer_pair_t* get_available_buffer() {
    for (int i = 0; i < NUM_BUFFERS; i++) {
        if (!buffer_pool[i].in_use) {
            buffer_pool[i].in_use = true;
            return &buffer_pool[i];
        }
    }
    return NULL; // No hay búferes disponibles
}

// Función para liberar un par de búferes
static void release_buffer(buffer_pair_t* buffer) {
    if (buffer) {
        buffer->in_use = false;
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando bus SPI en modo esclavo...");

    // Inicializar el pool de búferes
    for (int i = 0; i < NUM_BUFFERS; i++) {
        buffer_pool[i].tx_buffer = (char*)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);
        buffer_pool[i].rx_buffer = (char*)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);
        buffer_pool[i].in_use = false;
        
        if (!buffer_pool[i].tx_buffer || !buffer_pool[i].rx_buffer) {
            ESP_LOGE(TAG, "Fallo al asignar memoria para los búferes!");
            return;
        }
        // Inicializar los búferes con datos válidos
        memset(buffer_pool[i].tx_buffer, 0, BUFFER_SIZE);
        memset(buffer_pool[i].rx_buffer, 0, BUFFER_SIZE);
        snprintf(buffer_pool[i].tx_buffer, BUFFER_SIZE, "ACK: Esperando datos...");
    }

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.sclk_io_num = PIN_NUM_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = BUFFER_SIZE;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.spics_io_num = PIN_NUM_CS;
    slvcfg.flags = 0;
    slvcfg.queue_size = NUM_BUFFERS;
    slvcfg.mode = 0;
    slvcfg.post_setup_cb = NULL;
    slvcfg.post_trans_cb = NULL;

    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));

    // Prepara una respuesta inicial en el primer búfer de transmisión
    buffer_pair_t* current_buffer = get_available_buffer();
    if (!current_buffer) {
        ESP_LOGE(TAG, "No hay búferes disponibles!");
        return;
    }
    // Asegurarse de que el búfer tiene datos válidos desde el principio
    if (current_buffer->tx_buffer[0] == '\0') {
        snprintf(current_buffer->tx_buffer, BUFFER_SIZE, "ACK: Esperando el primer fotograma...");
    }

    while (1) {
        // Verificar que el búfer actual es válido
        if (!current_buffer) {
            ESP_LOGE(TAG, "Búfer actual inválido!");
            // Intentar obtener un nuevo búfer
            current_buffer = get_available_buffer();
            if (!current_buffer) {
                ESP_LOGE(TAG, "No hay búferes disponibles!");
                vTaskDelay(pdMS_TO_TICKS(100)); // Esperar antes de reintentar
                break;
            }
        }
        
        // Preparar la respuesta PARA LA PRÓXIMA transacción (antes de que ocurra)
        // Esto es crucial para evitar corrupción del primer byte
        // Primero guardamos los datos recibidos en esta transacción
        char received_data[BUFFER_SIZE];
        strncpy(received_data, current_buffer->rx_buffer, BUFFER_SIZE - 1);
        received_data[BUFFER_SIZE - 1] = '\0';
        
        // Limpiar el búfer de recepción para la próxima transacción
        memset(current_buffer->rx_buffer, 0, BUFFER_SIZE);
        
        // Preparar la respuesta basada en los datos recibidos
        if (strlen(received_data) > 0 && received_data[0] != '\0') {
            snprintf(current_buffer->tx_buffer, BUFFER_SIZE, "ACK para el fotograma recibido: '%.60s...'", received_data);
        } else {
            // Si no hay datos, mantener una respuesta válida
            if (current_buffer->tx_buffer[0] == '\0') {
                snprintf(current_buffer->tx_buffer, BUFFER_SIZE, "ACK: Esperando datos...");
            }
        }

        // Prepara la transacción. El esclavo siempre tiene una transacción lista.
        spi_slave_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.length = BUFFER_SIZE * 8;
        trans.tx_buffer = current_buffer->tx_buffer;
        trans.rx_buffer = current_buffer->rx_buffer;

        // Espera a que el maestro inicie la comunicación.
        esp_err_t ret = spi_slave_transmit(SPI_HOST, &trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error en la transmisión del esclavo: %s", esp_err_to_name(ret));
            release_buffer(current_buffer);
            // Obtener un nuevo búfer para la próxima transacción
            current_buffer = get_available_buffer();
            continue;
        }

        // La transacción ha ocurrido.
        // Verificar la integridad de los datos recibidos
        char* rx_data = current_buffer->rx_buffer;
        size_t rx_len = strlen(rx_data);
        
        // Contar caracteres no imprimibles como indicador de corrupción
        int non_printable = 0;
        for (size_t i = 0; i < rx_len && i < 50; i++) {
            if (rx_data[i] < 32 || rx_data[i] > 126) {
                non_printable++;
            }
        }
        
        if (non_printable > 5) {
            ESP_LOGW(TAG, "Datos posiblemente corruptos recibidos (%d caracteres no imprimibles)", non_printable);
        }
        
        // Verificar que los datos recibidos son válidos antes de procesarlos
        if (current_buffer->rx_buffer[0] != '\0') {
            ESP_LOGI(TAG, "Transacción completada. Recibido: '%.80s...'", current_buffer->rx_buffer);
        } else {
            ESP_LOGI(TAG, "Transacción completada. Recibido: (datos vacíos)");
        }

        // Liberar el búfer actual
        release_buffer(current_buffer);

        // Obtener un nuevo búfer para la próxima transacción
        current_buffer = get_available_buffer();
        if (!current_buffer) {
            ESP_LOGE(TAG, "No hay búferes disponibles!");
            continue;
        }
    }
}
