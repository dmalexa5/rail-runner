#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#define BOARD_LED_GPIO_PORT GPIOA
#define BOARD_LED_PIN GPIO_PIN_5

#define BOARD_CAN1_RX_GPIO_PORT GPIOB
#define BOARD_CAN1_RX_PIN GPIO_PIN_8
#define BOARD_CAN1_TX_PIN GPIO_PIN_9
#define BOARD_CAN1_GPIO_AF GPIO_AF9_CAN1

#define BOARD_UART_GPIO_PORT GPIOA
#define BOARD_UART_TX_PIN GPIO_PIN_2
#define BOARD_UART_RX_PIN GPIO_PIN_3
#define BOARD_UART_GPIO_AF GPIO_AF7_USART2

#ifdef BOARD_DRIVE
#define BOARD_HARD_LIMIT_GPIO_PORT GPIOA
#define BOARD_HARD_LIMIT_PIN GPIO_PIN_0
#define BOARD_ESTOP_GPIO_PORT GPIOA
#define BOARD_ESTOP_PIN GPIO_PIN_1
#define BOARD_OPTICAL_MIN_GPIO_PORT GPIOC
#define BOARD_OPTICAL_MIN_PIN GPIO_PIN_0
#else
#define BOARD_STEP_GPIO_PORT GPIOA
#define BOARD_STEP_PIN GPIO_PIN_0
#define BOARD_STEP_GPIO_AF GPIO_AF2_TIM5
#define BOARD_DIR_GPIO_PORT GPIOB
#define BOARD_DIR_PIN GPIO_PIN_0
#define BOARD_ENABLE_GPIO_PORT GPIOB
#define BOARD_ENABLE_PIN GPIO_PIN_1
#define BOARD_LIMIT_GPIO_PORT GPIOC
#define BOARD_LIMIT_PIN GPIO_PIN_0
#define BOARD_JOYSTICK_GPIO_PORT GPIOC
#define BOARD_JOYSTICK_PIN GPIO_PIN_1
#endif

/** Initializes the HAL and selected board's GPIO assignments. */
void board_init(void);

#ifdef BOARD_DRIVE
/** Returns true while the normally-closed hard-limit chain is healthy. */
bool board_hard_limits_ok(void);

/** Returns true while the normally-closed estop chain is healthy. */
bool board_estop_ok(void);

/** Returns true while the normally-closed MIN optical switch is asserted. */
bool board_optical_min_active(void);
#else
/** Returns true when the normally-closed MIN switch is open. */
bool board_limit_active(void);

/** Returns and clears the MIN-switch assertion recorded by EXTI. */
bool board_take_limit_event(void);
#endif

#endif
