#include "board.h"
#include "can.h"
#include "control.h"
#include "control_timer.h"
#include "protocol.h"
#include "uart.h"

static void fatal_error(const char *message)
{
    uart_write(message);
    while (1)
    {
        uart_service();
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_PORT, BOARD_LED_PIN);
        HAL_Delay(100U);
    }
}

int main(void)
{
    char line[32];
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
        fatal_error("ERR can_init\r\n");
    }

    control_init();
    if (!control_timer_init() || !control_timer_start())
    {
        fatal_error("ERR timer_init\r\n");
    }

    uart_write("READY rail-firmware at 1khz\r\n");

    while (1)
    {
        if (control_run_pending())
        {
            continue;
        }

        if (uart_read_line(line, sizeof(line)))
        {
            protocol_handle_line(line);
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
