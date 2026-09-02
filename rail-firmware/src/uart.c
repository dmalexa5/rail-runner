#include "uart.h"

#include <string.h>

#include "board.h"

static UART_HandleTypeDef huart2;

bool uart_init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    return HAL_UART_Init(&huart2) == HAL_OK;
}

bool uart_read_line(char *line, size_t line_size)
{
    static char buffer[32];
    static size_t used = 0;
    uint8_t ch = 0;

    if (line == 0 || line_size == 0)
    {
        return false;
    }

    while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
    {
        if (ch == '\r' || ch == '\n')
        {
            if (used == 0)
            {
                continue;
            }

            if (used >= line_size)
            {
                used = line_size - 1;
            }
            memcpy(line, buffer, used);
            line[used] = '\0';
            used = 0;
            return true;
        }

        if (used + 1 < sizeof(buffer))
        {
            buffer[used++] = (char)ch;
        }
    }

    return false;
}

void uart_write(const char *text)
{
    if (text == 0)
    {
        return;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}
