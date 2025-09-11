#include "report.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "i2c.h"
#include "safe_printf.h"
#include "fec_codec.h"
#include "crc.h"

// DHT11 data
static float s_dht11_humidity = 0.0;
static float s_dht11_temperature = 0.0;
static bool s_dht11_data_valid = false;

static int64_t s_last_report_packet_tp = -5000;

extern Ground2Air_Config_Packet s_ground2air_config_packet;
extern uint16_t s_air_device_id;
extern uint16_t s_connected_gs_device_id;

// Function to read DHT11 data from I2C slave
esp_err_t read_dht11_data(float* humidity, float* temperature) {
    TickType_t start_time = xTaskGetTickCount();
    
    if (!s_i2c_initialized) {
        LOG("DHT11 Read - I2C not initialized, initializing\n");
        initialize_i2c();
        if (!s_i2c_initialized) {
            LOG("DHT11 Read - Failed to initialize I2C\n");
            return ESP_FAIL;
        }
    }

    uint8_t data_buffer[9]; // 4 bytes for humidity + 4 bytes for temperature + 1 byte for valid flag

    // Read data from slave
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data_buffer, sizeof(data_buffer), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        // Log raw data for debugging
        LOG("DHT11 Read - Raw data received: ");
        for (int j = 0; j < sizeof(data_buffer); j++) {
            LOG("%02X ", data_buffer[j]);
        }
        LOG("\n");

        // Check data validity flag (last byte)
        if (data_buffer[8] == 0) {
            LOG("DHT11 Read - Data not valid from auxiliary unit\n");
            LOG("%d-%d\n",data_buffer[7], data_buffer[8]);
            return ESP_ERR_INVALID_STATE;
        }

        // Convert bytes to float values
        memcpy(humidity, &data_buffer[0], sizeof(float));
        memcpy(temperature, &data_buffer[4], sizeof(float));
        
        TickType_t end_time = xTaskGetTickCount();
        LOG("DHT11 Read - Success: H=%d%%, T=%d°C (took %d ms)\n", 
            (int)(*humidity), (int)(*temperature), (int)((end_time - start_time) * portTICK_PERIOD_MS));

        return ESP_OK;
    }
    
    LOG("DHT11 Read - Failed to read data, error: %s\n", esp_err_to_name(ret));
    s_i2c_initialized = false;
    return ret;
}

IRAM_ATTR void send_air2ground_report_packet()
{
    uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);
    if( !packet_data )
    {
        LOG("no data buf!\n");
        return;
    }

    Air2Ground_Report_Packet& packet = *(Air2Ground_Report_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::Report; // Using Report type for report packets
    packet.size = sizeof(Air2Ground_Report_Packet);
    packet.pong = s_ground2air_config_packet.ping;
    packet.version = PACKET_VERSION;
    packet.airDeviceId = s_air_device_id;
    packet.gsDeviceId = s_connected_gs_device_id;
    packet.crc = 0;

    // Fill in DHT11 data
    packet.temperature = s_dht11_temperature;
    packet.humidity = s_dht11_humidity;
    packet.data_valid = s_dht11_data_valid ? 1 : 0;

    packet.crc = crc8(0, &packet, sizeof(Air2Ground_Report_Packet));

    if (!s_fec_encoder.flush_encode_packet(true))
    {
        LOG("Fec codec busy\n");
        // s_stats.wlan_error_count++; // s_stats is in main.cpp, not accessible here
    }
}

IRAM_ATTR void handle_dht11_read_and_send()
{
    // Only proceed if 5 seconds have passed since the last report packet was sent
    if (millis() - s_last_report_packet_tp > 5000) {
        static float dht11_humidity_local = 0.0;
        static float dht11_temperature_local = 0.0;
        
        esp_err_t result = read_dht11_data(&dht11_humidity_local, &dht11_temperature_local);
        if (result == ESP_OK) {
            s_dht11_humidity = dht11_humidity_local;
            s_dht11_temperature = dht11_temperature_local;
            s_dht11_data_valid = true;
            LOG("DHT11 data - Humidity: %d%%, Temperature: %d°C\n", (int)dht11_humidity_local, (int)dht11_temperature_local);
        } else {
            s_dht11_data_valid = false;
            LOG("Failed to read DHT11 data: %s\n", esp_err_to_name(result));
        }

        // Always attempt to send the report packet if connected, regardless of read success
        // The data_valid flag in the packet will indicate if the data is fresh/valid
        if (s_connected_gs_device_id != 0) {
            send_air2ground_report_packet();
        }
        s_last_report_packet_tp = millis(); // Reset timer after a full cycle (read attempt + send attempt)
    }
}
