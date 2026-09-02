#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stddef.h>

/** Initializes USART2 at 115200 baud with interrupt-driven transmission. */
bool uart_init(void);

/**
 * Polls received bytes and returns at most one complete, newline-terminated
 * line. Input longer than the internal line buffer is discarded as an empty
 * line so the protocol rejects it.
 */
bool uart_read_line(char *line, size_t line_size);

/**
 * Appends a string to the bounded asynchronous transmit queue.
 *
 * Returns false without enqueueing a partial string when insufficient queue
 * space is available. This function never waits for UART hardware.
 */
bool uart_write(const char *text);

/** Starts queued UART transmission when the peripheral is idle. */
void uart_service(void);

#endif
