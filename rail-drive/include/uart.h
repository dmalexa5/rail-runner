#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stddef.h>

/** Initializes USART2 at 115200 baud with interrupt-driven RX and TX. */
bool uart_init(void);

/** Returns at most one complete LF- or CRLF-terminated request. */
bool uart_read_line(char *line, size_t line_size);

/** Queues one complete response without blocking or partial writes. */
bool uart_write(const char *text);

/** Starts transmission of queued response bytes when USART2 is idle. */
void uart_service(void);

/** Returns and clears the receive-ring overflow flag. */
bool uart_take_rx_overflow(void);

#endif
