#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"

// --- Configuración de Hardware ---
#define SPI_HOST SPI3_HOST
#define PIN_NUM_MOSI 13
#define PIN_NUM_MISO 15
#define PIN_NUM_SCLK 14
#define PIN_NUM_CS   2

// --- Configuración de la Simulación ---
#define FRAME_BUFFER_SIZE 1024 // Simula un fragmento de fotograma de 1KB
#define FRAME_RATE_MS 33       // ~30 FPS
#define MAX_SPI_TRANSACTIONS_IN_FLIGHT 5 // Número de transacciones SPI que podemos poner en cola a la vez

static const char* CAM_TAG = "CameraTask";
static const char* SPI_TAG = "SpiMasterTask";

static QueueHandle_t frame_data_queue;

// ==============================================================================
// TAREA 1: Simula la cámara
// ==============================================================================
void camera_simulator_task(void* pvParameters) {
    ESP_LOGI(CAM_TAG, "Tarea del simulador de cámara iniciada.");
    uint32_t frame_counter = 0;

    while (1) {
        char* frame_buffer = (char*)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_DMA);
        if (frame_buffer == nullptr) {
            ESP_LOGE(CAM_TAG, "Fallo al asignar memoria para el fotograma!");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        memset(frame_buffer, 0, FRAME_BUFFER_SIZE); // Ensure buffer is null-terminated
        snprintf(frame_buffer, FRAME_BUFFER_SIZE, "Este es el fotograma #%lu de la camara.", frame_counter++);
        
        if (xQueueSend(frame_data_queue, &frame_buffer, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGW(CAM_TAG, "La cola de SPI está llena. Descartando fotograma.");
            free(frame_buffer);
        } else {
             ESP_LOGI(CAM_TAG, "Fotograma #%lu enviado a la cola de SPI.", frame_counter - 1);
        }

        vTaskDelay(pdMS_TO_TICKS(FRAME_RATE_MS));
    }
}

// ==============================================================================
// TAREA 2: Gestiona la comunicación SPI
// ==============================================================================
void spi_master_task(void* pvParameters) {
    spi_device_handle_t spi_handle = (spi_device_handle_t)pvParameters;
    esp_err_t ret;

    // 1. Crear un POOL de descriptores de transacción y búferes de recepción.
    // Cada transacción "en vuelo" tendrá su propio descriptor y su propio búfer de recepción.
    std::vector<spi_transaction_t> transactions(MAX_SPI_TRANSACTIONS_IN_FLIGHT);
    std::vector<char*> rx_buffers(MAX_SPI_TRANSACTIONS_IN_FLIGHT);

    for (int i = 0; i < MAX_SPI_TRANSACTIONS_IN_FLIGHT; i++) {
        // Asignar memoria para cada búfer de recepción
        rx_buffers[i] = (char*)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_DMA);
        if (!rx_buffers[i]) {
            ESP_LOGE(SPI_TAG, "Fallo al asignar búfer de recepción!");
            return;
        }
        // Pre-configurar las partes constantes de cada descriptor
        memset(&transactions[i], 0, sizeof(spi_transaction_t));
        transactions[i].length = FRAME_BUFFER_SIZE * 8;
        transactions[i].rx_buffer = rx_buffers[i];
    }

    int transactions_posted = 0; // Contador de transacciones enviadas pero no completadas.
    int next_trans_idx = 0;      // Índice para el siguiente recurso del pool a usar.

    // =================== FIN DE LAS CORRECCIONES ===================

    ESP_LOGI(SPI_TAG, "Tarea SPI iniciada. Esperando datos de la cámara...");

    while (1) {
        // --- FASE 1: Recoger resultados de transacciones completadas ---
        // Recogemos todos los que estén listos para liberar recursos.
        while (transactions_posted > 0) {
            spi_transaction_t* completed_trans;
            ret = spi_device_get_trans_result(spi_handle, &completed_trans, 0); // Timeout de 0 = no bloquear

            if (ret == ESP_OK) {
                // ¡Transacción completada!
                transactions_posted--; // Un recurso menos en vuelo
                
                // Verificar la integridad de los datos recibidos
                char* rx_data = (char*)completed_trans->rx_buffer;
                size_t rx_len = strlen(rx_data);
                
                // Contar caracteres no imprimibles como indicador de corrupción
                int non_printable = 0;
                for (size_t i = 0; i < rx_len && i < 50; i++) {
                    if (rx_data[i] < 32 || rx_data[i] > 126) {
                        non_printable++;
                    }
                }
                
                if (non_printable > 5) {
                    ESP_LOGW(SPI_TAG, "Datos posiblemente corruptos recibidos (%d caracteres no imprimibles)", non_printable);
                }
                
                // Verificar si el primer byte está corrupto (indicador común del problema)
                if (rx_len > 0 && (rx_data[0] < 32 || rx_data[0] > 126) && rx_data[0] != 0) {
                    ESP_LOGW(SPI_TAG, "Primer byte posiblemente corrupto: 0x%02X", (unsigned char)rx_data[0]);
                }
                
                ESP_LOGI(SPI_TAG, "Transacción completada. Recibido: '%.50s...'", rx_data);
                
                // IMPORTANTE: Liberamos la memoria del búfer de transmisión que fue
                // asignado por la tarea de la cámara.
                free((void*)completed_trans->tx_buffer);
                vTaskDelay(pdMS_TO_TICKS(1)); // Ceder el paso después de procesar una transacción

            } else if (ret == ESP_ERR_TIMEOUT) {
                // Normal, significa que no hay más transacciones completadas esperando en la cola.
                break;
            } else {
                ESP_LOGE(SPI_TAG, "Error al obtener resultado de la transacción: %s", esp_err_to_name(ret));
                // En caso de error, decrementamos el contador para evitar bloqueo
                transactions_posted--;
            }
        }

        // --- FASE 2: Poner en cola nuevas transacciones si hay datos y recursos ---
        // Solo intentamos enviar si tenemos un recurso libre en nuestro pool.
        if (transactions_posted < MAX_SPI_TRANSACTIONS_IN_FLIGHT) {
            char* frame_to_send = nullptr;
            // Esperamos brevemente por nuevos datos de la cámara
            if (xQueueReceive(frame_data_queue, &frame_to_send, pdMS_TO_TICKS(1))) {
                // Verificar que el buffer es válido antes de usarlo
                if (frame_to_send == nullptr) {
                    ESP_LOGW(SPI_TAG, "Recibido buffer nulo de la cola de SPI. Descartando.");
                    continue;
                }
                
                // Verificar que el buffer contiene datos válidos
                if (strlen(frame_to_send) == 0) {
                    ESP_LOGW(SPI_TAG, "Recibido buffer vacío de la cola de SPI. Descartando.");
                    free((void*)frame_to_send);
                    continue;
                }
                
                // Obtenemos el siguiente descriptor y búfer del pool
                spi_transaction_t* trans = &transactions[next_trans_idx];
                
                // Configuramos la parte variable del descriptor: el búfer de transmisión
                trans->tx_buffer = frame_to_send;

                ESP_LOGI(SPI_TAG, "Poniendo en cola la transmisión para: '%.50s...'", frame_to_send);
                
                // Usar un timeout razonable en lugar de portMAX_DELAY para evitar bloqueos indefinidos
                ret = spi_device_queue_trans(spi_handle, trans, pdMS_TO_TICKS(100));
                if (ret == ESP_ERR_TIMEOUT) {
                    ESP_LOGW(SPI_TAG, "Timeout al poner en cola la transacción SPI");
                    // Liberar el buffer ya que no se pudo encolar
                    free((void*)frame_to_send);
                } else if (ret != ESP_OK) {
                    ESP_LOGE(SPI_TAG, "Error al poner en cola la transacción SPI: %s", esp_err_to_name(ret));
                    // Liberar el buffer ya que no se pudo encolar
                    free((void*)frame_to_send);
                } else {
                    // Actualizamos nuestros contadores para la siguiente iteración solo si tuvo éxito
                    transactions_posted++;
                    next_trans_idx = (next_trans_idx + 1) % MAX_SPI_TRANSACTIONS_IN_FLIGHT;
                }
            }
        }
        
        // Ceder el paso si no hay nada que hacer
        // Reducir el delay para ceder control más frecuentemente
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


extern "C" void app_main(void) {
    // Desactivar el Task Watchdog Timer (TWDT) completamente
    esp_task_wdt_deinit();

    frame_data_queue = xQueueCreate(MAX_SPI_TRANSACTIONS_IN_FLIGHT, sizeof(char*));

    ESP_LOGI(SPI_TAG, "Inicializando bus SPI...");
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.sclk_io_num = PIN_NUM_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = FRAME_BUFFER_SIZE;

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 7 * 1000 * 1000; // 7 MHz is the maximum stable frecuency
    devcfg.mode = 0;
    devcfg.spics_io_num = PIN_NUM_CS;
    devcfg.cs_ena_pretrans = 1; // Add 1 clock cycle before transaction to stabilize CS
    devcfg.cs_ena_posttrans = 1; // Add 1 clock cycle after transaction to stabilize CS
    // devcfg.dummy_bits = 8; // Reverted: Dummy bits caused queue to fill up
    // devcfg.flags = SPI_DEVICE_NO_DUMMY; // Removed flag to allow default timing/dummy cycles
    // El tamaño de la cola del driver debe coincidir con nuestro pool
    devcfg.queue_size = MAX_SPI_TRANSACTIONS_IN_FLIGHT;
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_handle_t spi_handle;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi_handle));

    xTaskCreate(camera_simulator_task, "camera_task", 4096, NULL, 10, NULL);
    xTaskCreate(spi_master_task, "spi_master_task", 4096, (void*)spi_handle, 5, NULL);
}
