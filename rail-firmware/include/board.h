#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#define BOARD_LED_GPIO_PORT GPIOA
#define BOARD_LED_PIN GPIO_PIN_5

#define BOARD_CAN1_RX_GPIO_PORT GPIOB
#define BOARD_CAN1_RX_PIN GPIO_PIN_8
#define BOARD_CAN1_TX_GPIO_PORT GPIOB
#define BOARD_CAN1_TX_PIN GPIO_PIN_9
#define BOARD_CAN1_GPIO_AF GPIO_AF9_CAN1

#define BOARD_UART_GPIO_PORT GPIOA
#define BOARD_UART_TX_PIN GPIO_PIN_2
#define BOARD_UART_RX_PIN GPIO_PIN_3
#define BOARD_UART_GPIO_AF GPIO_AF7_USART2

/*
 * The safety switches are normally closed to ground. A low input means the
 * switch and its wiring are healthy; a high input means pressed or open and
 * must inhibit motor torque.
 */
#define BOARD_SAFETY_GPIO_PORT GPIOA
#define BOARD_SAFETY_1_PIN GPIO_PIN_0
#define BOARD_SAFETY_2_PIN GPIO_PIN_1

/** Initializes the HAL and all board-level GPIO pin assignments. */
void board_init(void);

/** Returns true only while both normally-closed safety inputs read low. */
bool board_safety_switches_ok(void);

#endif
