#include "board.h"
#include "can.h"
#include "control.h"
#include "control_timer.h"
#include "uart.h"

static void fatal_error(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_PORT, BOARD_LED_PIN);
        HAL_Delay(100U);
    }
}

int main(void)
{
    uint32_t last_led_ms = 0U;

    board_init();
    if (!uart_init())
    {
        while (1)
        {
        }
    }
    if (!can_init())
    {
        fatal_error();
    }

    control_init();
    if (!control_timer_init() || !control_timer_start())
    {
        fatal_error();
    }

    while (1)
    {
        if (control_run_pending())
        {
            continue;
        }

        uart_service();

        uint32_t now = HAL_GetTick();
        if (now - last_led_ms >= 500U)
        {
            HAL_GPIO_TogglePin(BOARD_LED_GPIO_PORT, BOARD_LED_PIN);
            last_led_ms = now;
        }
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
