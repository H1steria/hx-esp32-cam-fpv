// Explicitly include FreeRTOS headers first to ensure proper type definitions
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "report.h"
#include "esp_log.h"
#include "string.h"
#include "crc.h"
#include "esp_wifi.h" // For esp_wifi_80211_tx
#include "esp_private/wifi.h" // For esp_wifi_80211_tx (private API)
#include "wifi.h" // For ESP_WIFI_IF
#include "safe_printf.h" // This includes FreeRTOS headers in correct order
#include "structures.h" // For Packet_Header and WLAN_MAX_PAYLOAD_SIZE

extern int s_uart_verbose;

#define LOG(...) do { if (s_uart_verbose > 0) SAFE_PRINTF(__VA_ARGS__); } while (false) 

// DHT11 data (these are extern in spi_master.h and defined in air_main.cpp)
extern float s_dht11_humidity;
extern float s_dht11_temperature;
extern bool s_dht11_data_valid;

// Queue for report packets (now will contain FEC-ready payloads)
QueueHandle_t s_report_packet_queue;

extern Ground2Air_Config_Packet s_ground2air_config_packet;
extern uint16_t s_air_device_id;
extern uint16_t s_connected_gs_device_id;

// Extern declaration for the main outgoing WLAN queue
extern QueueHandle_t s_wlan_outgoing_queue;

void send_air2ground_report_packet()
{
    // The report packet will now be a payload for the FEC system.
    // It needs to be preceded by a Packet_Header.
    // The total size will be sizeof(Packet_Header) + sizeof(Air2Ground_Report_Packet)
    uint8_t fec_payload_buffer[sizeof(Packet_Header) + sizeof(Air2Ground_Report_Packet)];
    
    // Populate the Packet_Header (dummy values for now, will be filled by FEC encoder)
    Packet_Header fec_header;
    fec_header.block_index = 0; // Will be set by FEC encoder
    fec_header.packet_index = 0; // Will be set by FEC encoder
    fec_header.size = sizeof(Air2Ground_Report_Packet); // Size of the actual report packet payload

    // Copy the FEC header to the buffer
    memcpy(fec_payload_buffer, &fec_header, sizeof(Packet_Header));

    // Populate the Report Packet payload
    Air2Ground_Report_Packet report_packet;
    report_packet.type = Air2Ground_Header::Type::Report;
    report_packet.size = sizeof(Air2Ground_Report_Packet);
    report_packet.pong = s_ground2air_config_packet.ping;
    report_packet.version = PACKET_VERSION;
    report_packet.airDeviceId = s_air_device_id;
    report_packet.gsDeviceId = s_connected_gs_device_id;
    report_packet.crc = 0;

    // Fill in DHT11 data
    report_packet.temperature = s_dht11_temperature;
    report_packet.humidity = s_dht11_humidity;
    report_packet.data_valid = s_dht11_data_valid ? 1 : 0;

    report_packet.crc = crc8(0, &report_packet, sizeof(Air2Ground_Report_Packet));

    // Copy the Report Packet into the FEC payload buffer after the Packet_Header
    memcpy(fec_payload_buffer + sizeof(Packet_Header), &report_packet, sizeof(Air2Ground_Report_Packet));

    // Send to the main outgoing WLAN queue for FEC encoding and transmission
    if (s_wlan_outgoing_queue != NULL) {
        // We need to send the entire FEC payload (Packet_Header + Air2Ground_Report_Packet)
        // as a single item to the outgoing queue.
        // The queue expects Wlan_Outgoing_Packet, so we need to create one.
        Wlan_Outgoing_Packet outgoing_wlan_packet;
        // Allocate memory for the packet data (802.11 header + FEC payload)
        // The 802.11 header will be added by the wifi_tx_task
        outgoing_wlan_packet.ptr = (uint8_t*)heap_caps_malloc(WLAN_MAX_PAYLOAD_SIZE, MALLOC_CAP_DMA);
        if (outgoing_wlan_packet.ptr == NULL) {
            LOG("Failed to allocate memory for outgoing report packet\n");
            return;
        }
        // The payload_ptr points to where the FEC payload starts (after 802.11 header)
        outgoing_wlan_packet.payload_ptr = outgoing_wlan_packet.ptr; // No 802.11 header here yet
        outgoing_wlan_packet.size = sizeof(Packet_Header) + sizeof(Air2Ground_Report_Packet);
        outgoing_wlan_packet.offset = 0; // Not used for this type of packet

        // Copy the FEC payload into the allocated buffer
        memcpy(outgoing_wlan_packet.payload_ptr, fec_payload_buffer, outgoing_wlan_packet.size);

        if (xQueueSend(s_wlan_outgoing_queue, &outgoing_wlan_packet, pdMS_TO_TICKS(10)) != pdPASS) {
            LOG("Failed to enqueue FEC-ready report packet\n");
            heap_caps_free(outgoing_wlan_packet.ptr); // Free memory if enqueue fails
        } else {
            LOG("FEC-ready report packet enqueued. AirID: %d, GsID: %d, Temp: %.2f, Hum: %.2f, Valid: %d\n",
                report_packet.airDeviceId, report_packet.gsDeviceId, report_packet.temperature,
                report_packet.humidity, (int)report_packet.data_valid);
        }
    } else {
        LOG("WLAN outgoing queue not initialized\n");
    }
}

// The report_packet_send_task is no longer needed as report packets will go through the main WLAN outgoing queue.
// This task will be removed.
void report_packet_send_task(void* pvParameters) {
    // This task is now obsolete.
    vTaskDelete(NULL);
}
