#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stddef.h>

bool uart_init(void);
bool uart_read_line(char *line, size_t line_size);
void uart_write(const char *text);

#endif
