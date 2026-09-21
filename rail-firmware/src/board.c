#include "board.h"

void board_init(void)
{
    HAL_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BOARD_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_GPIO_PORT, &gpio);

    gpio.Pin = BOARD_CAN1_RX_PIN | BOARD_CAN1_TX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = BOARD_CAN1_GPIO_AF;
    HAL_GPIO_Init(BOARD_CAN1_RX_GPIO_PORT, &gpio);

    gpio.Pin = BOARD_UART_TX_PIN | BOARD_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = BOARD_UART_GPIO_AF;
    HAL_GPIO_Init(BOARD_UART_GPIO_PORT, &gpio);

    gpio.Pin = BOARD_HARD_LIMIT_PIN | BOARD_ESTOP_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = BOARD_OPTICAL_MIN_PIN;
    HAL_GPIO_Init(BOARD_OPTICAL_MIN_GPIO_PORT, &gpio);
}

bool board_hard_limits_ok(void)
{
    return HAL_GPIO_ReadPin(BOARD_HARD_LIMIT_GPIO_PORT,
                            BOARD_HARD_LIMIT_PIN) == GPIO_PIN_RESET;
}

bool board_estop_ok(void)
{
    return HAL_GPIO_ReadPin(BOARD_ESTOP_GPIO_PORT,
                            BOARD_ESTOP_PIN) == GPIO_PIN_RESET;
}

bool board_optical_min_active(void)
{
    return HAL_GPIO_ReadPin(BOARD_OPTICAL_MIN_GPIO_PORT,
                            BOARD_OPTICAL_MIN_PIN) == GPIO_PIN_SET;
}
