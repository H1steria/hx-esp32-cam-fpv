#ifndef REPORT_H
#define REPORT_H

#include "esp_err.h"
#include "packets.h" // For Air2Ground_Report_Packet
#include "freertos/queue.h" // Include FreeRTOS queue header

#ifdef __cplusplus
extern "C" {

// Declare the new queue for report packets
extern QueueHandle_t s_report_packet_queue;
extern void report_packet_send_task(void* pvParameters); // Declare the new task

#endif

void send_air2ground_report_packet();

extern uint64_t millis();

#ifdef __cplusplus
}
#endif

#endif
