#include "ld2423_uart.h"

#include <furi_hal_serial.h>
#include <furi_hal_serial_control.h>
#include <string.h>

#define TAG         "LD2423Uart"
#define UART_CH     FuriHalSerialIdUsart
#define RX_BUF_SIZE 1024

struct LD2423Uart {
    FuriThread* thread;
    FuriStreamBuffer* rx_stream;
    FuriMutex* tx_mutex;
    FuriHalSerialHandle* serial_handle;
    LD2423ReportCallback report_cb;
    LD2423AckCallback ack_cb;
    void* cb_context;
    LD2423Parser parser;
    volatile bool worker_running;
    volatile bool reset_parser_req;
    volatile uint32_t total_rx_bytes;
    volatile uint32_t total_tx_bytes;
    volatile uint32_t current_baudrate;
};

static void ld2423_uart_on_irq_rx(
    FuriHalSerialHandle* handle,
    FuriHalSerialRxEvent event,
    void* context) {
    LD2423Uart* uart = (LD2423Uart*)context;

    if(event & FuriHalSerialRxEventData) {
        uint8_t data = furi_hal_serial_async_rx(handle);
        uart->total_rx_bytes++;
        furi_stream_buffer_send(uart->rx_stream, &data, 1, 0);
    }
}

static int32_t ld2423_uart_worker(void* context) {
    LD2423Uart* uart = (LD2423Uart*)context;
    uint8_t data_buf[64];

    while(uart->worker_running) {
        if(uart->reset_parser_req) {
            ld2423_parser_reset(&uart->parser);
            uart->reset_parser_req = false;
        }

        size_t received =
            furi_stream_buffer_receive(uart->rx_stream, data_buf, sizeof(data_buf), 100);

        if(received > 0) {
            for(size_t i = 0; i < received; i++) {
                LD2423Report report;
                LD2423Ack ack;
                LD2423ParseResult res =
                    ld2423_parser_push_byte(&uart->parser, data_buf[i], &report, &ack);

                if(res == LD2423ParseReport && uart->report_cb) {
                    uart->report_cb(&report, uart->cb_context);
                } else if(res == LD2423ParseAck && uart->ack_cb) {
                    uart->ack_cb(&ack, uart->cb_context);
                }
            }
        }
    }

    return 0;
}

LD2423Uart* ld2423_uart_alloc(void) {
    LD2423Uart* uart = malloc(sizeof(LD2423Uart));
    furi_check(uart);
    memset(uart, 0, sizeof(LD2423Uart));

    uart->rx_stream = furi_stream_buffer_alloc(RX_BUF_SIZE, 1);
    furi_check(uart->rx_stream);

    uart->tx_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    furi_check(uart->tx_mutex);

    uart->thread = furi_thread_alloc();
    furi_check(uart->thread);

    furi_thread_set_name(uart->thread, "LD2423UartWorker");
    furi_thread_set_stack_size(uart->thread, 2048);
    furi_thread_set_callback(uart->thread, ld2423_uart_worker);
    furi_thread_set_context(uart->thread, uart);

    ld2423_parser_init(&uart->parser);
    uart->current_baudrate = LD2423_DEFAULT_BAUDRATE;

    return uart;
}

void ld2423_uart_free(LD2423Uart* uart) {
    furi_assert(uart);
    ld2423_uart_stop(uart);
    furi_thread_free(uart->thread);
    furi_mutex_free(uart->tx_mutex);
    furi_stream_buffer_free(uart->rx_stream);
    free(uart);
}

void ld2423_uart_set_callbacks(
    LD2423Uart* uart,
    LD2423ReportCallback report_cb,
    LD2423AckCallback ack_cb,
    void* context) {
    furi_assert(uart);
    uart->report_cb = report_cb;
    uart->ack_cb = ack_cb;
    uart->cb_context = context;
}

bool ld2423_uart_start(LD2423Uart* uart, uint32_t baudrate) {
    furi_assert(uart);
    if(uart->worker_running) return true;

    uart->worker_running = true;
    uart->current_baudrate = baudrate;
    ld2423_parser_init(&uart->parser);

    int attempts = 20;
    while(attempts > 0) {
        uart->serial_handle = furi_hal_serial_control_acquire(UART_CH);
        if(uart->serial_handle) break;
        furi_delay_ms(10);
        attempts--;
    }

    if(!uart->serial_handle) {
        FURI_LOG_E(TAG, "Failed to acquire USART handle");
        uart->worker_running = false;
        return false;
    }

    furi_hal_serial_init(uart->serial_handle, baudrate);
    furi_hal_serial_async_rx_start(uart->serial_handle, ld2423_uart_on_irq_rx, uart, false);

    furi_thread_start(uart->thread);
    return true;
}

void ld2423_uart_stop(LD2423Uart* uart) {
    furi_assert(uart);

    if(uart->worker_running) {
        uart->worker_running = false;

        if(uart->serial_handle) {
            furi_hal_serial_async_rx_stop(uart->serial_handle);
        }

        furi_thread_join(uart->thread);

        if(uart->serial_handle) {
            furi_hal_serial_deinit(uart->serial_handle);
            furi_hal_serial_control_release(uart->serial_handle);
            uart->serial_handle = NULL;
        }
    }
}

bool ld2423_uart_set_baudrate(LD2423Uart* uart, uint32_t baudrate) {
    furi_assert(uart);
    if(!uart->serial_handle) return false;

    furi_mutex_acquire(uart->tx_mutex, FuriWaitForever);
    furi_hal_serial_set_br(uart->serial_handle, baudrate);
    uart->current_baudrate = baudrate;
    uart->reset_parser_req = true;
    furi_mutex_release(uart->tx_mutex);
    return true;
}

bool ld2423_uart_send(LD2423Uart* uart, const uint8_t* data, size_t len) {
    furi_assert(uart);
    if(!uart->serial_handle || !data || len == 0) return false;

    furi_mutex_acquire(uart->tx_mutex, FuriWaitForever);
    furi_hal_serial_tx(uart->serial_handle, data, len);
    furi_hal_serial_tx_wait_complete(uart->serial_handle);
    uart->total_tx_bytes += (uint32_t)len;
    furi_mutex_release(uart->tx_mutex);
    return true;
}

uint32_t ld2423_uart_get_rx_bytes(LD2423Uart* uart) {
    return uart ? uart->total_rx_bytes : 0;
}

uint32_t ld2423_uart_get_tx_bytes(LD2423Uart* uart) {
    return uart ? uart->total_tx_bytes : 0;
}

uint32_t ld2423_uart_get_bad_frames(LD2423Uart* uart) {
    return uart ? uart->parser.bad_frames : 0;
}

uint32_t ld2423_uart_get_baudrate(LD2423Uart* uart) {
    return uart ? uart->current_baudrate : 0;
}
