#ifndef REPORT_H
#define REPORT_H

#include "esp_err.h"
#include "packets.h" // For Air2Ground_Report_Packet

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t read_dht11_data(float* humidity, float* temperature);
void send_air2ground_report_packet();
void handle_dht11_read_and_send();

extern uint64_t millis();

#ifdef __cplusplus
}
#endif

#endif
