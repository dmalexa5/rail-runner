#ifndef TEST_REGISTERS_H
#define TEST_REGISTERS_H
#include <stdint.h>
#define _BV(bit) (1U << (bit))
#define PB0 0
#define PB1 1
#define PB4 4
#define PD7 7
#define UDRIE0 5
#define ISR(vector) void vector(void)
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(mode) for (int atomic_once = 1; atomic_once; atomic_once = 0)
static uint8_t PORTB, PORTD, PINB, UCSR0B, UDR0;
static uint16_t ADC;
#endif
