#include "board.h"

#include "stepper.h"

static volatile bool limit_event;

void board_init(void)
{
    HAL_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    HAL_GPIO_WritePin(BOARD_LED_GPIO_PORT, BOARD_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_DIR_GPIO_PORT, BOARD_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_ENABLE_GPIO_PORT, BOARD_ENABLE_PIN,
                      GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BOARD_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_GPIO_PORT, &gpio);

    gpio.Pin = BOARD_DIR_PIN | BOARD_ENABLE_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = BOARD_STEP_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = BOARD_STEP_GPIO_AF;
    HAL_GPIO_Init(BOARD_STEP_GPIO_PORT, &gpio);

    gpio.Pin = BOARD_UART_TX_PIN | BOARD_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = BOARD_UART_GPIO_AF;
    HAL_GPIO_Init(BOARD_UART_GPIO_PORT, &gpio);

    gpio.Pin = BOARD_LIMIT_PIN;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0;
    HAL_GPIO_Init(BOARD_LIMIT_GPIO_PORT, &gpio);

    gpio.Pin = BOARD_JOYSTICK_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOARD_JOYSTICK_GPIO_PORT, &gpio);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

bool board_limit_active(void)
{
    return HAL_GPIO_ReadPin(BOARD_LIMIT_GPIO_PORT,
                            BOARD_LIMIT_PIN) == GPIO_PIN_SET;
}

bool board_take_limit_event(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool event = limit_event;
    limit_event = false;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return event;
}

void EXTI0_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(BOARD_LIMIT_PIN) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(BOARD_LIMIT_PIN);
        limit_event = true;
        stepper_emergency_stop_isr();
    }
}
