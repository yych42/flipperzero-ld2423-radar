#pragma once

#include <furi.h>
#include <furi_hal.h>
#include "ld2423_protocol.h"

typedef struct LD2423Uart LD2423Uart;

typedef void (*LD2423ReportCallback)(const LD2423Report* report, void* context);
typedef void (*LD2423AckCallback)(const LD2423Ack* ack, void* context);

LD2423Uart* ld2423_uart_alloc(void);
void ld2423_uart_free(LD2423Uart* uart);

void ld2423_uart_set_callbacks(
    LD2423Uart* uart,
    LD2423ReportCallback report_cb,
    LD2423AckCallback ack_cb,
    void* context);

bool ld2423_uart_start(LD2423Uart* uart, uint32_t baudrate);
void ld2423_uart_stop(LD2423Uart* uart);

bool ld2423_uart_set_baudrate(LD2423Uart* uart, uint32_t baudrate);
bool ld2423_uart_send(LD2423Uart* uart, const uint8_t* data, size_t len);

uint32_t ld2423_uart_get_rx_bytes(LD2423Uart* uart);
uint32_t ld2423_uart_get_tx_bytes(LD2423Uart* uart);
uint32_t ld2423_uart_get_bad_frames(LD2423Uart* uart);
uint32_t ld2423_uart_get_baudrate(LD2423Uart* uart);
