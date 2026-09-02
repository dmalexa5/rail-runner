#include "uart.h"

#include <string.h>

#include "board.h"

static UART_HandleTypeDef huart2;

#define UART_TX_QUEUE_SIZE 512U

static uint8_t tx_queue[UART_TX_QUEUE_SIZE];
static volatile size_t tx_head;
static volatile size_t tx_tail;
static volatile size_t tx_in_flight;
static volatile bool tx_active;

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

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    return true;
}

bool uart_read_line(char *line, size_t line_size)
{
    static char buffer[32];
    static size_t used = 0;
    static bool overflow = false;
    uint8_t ch = 0;

    if (line == 0 || line_size == 0)
    {
        return false;
    }

    while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
    {
        if (ch == '\r' || ch == '\n')
        {
            if (overflow)
            {
                line[0] = '\0';
                used = 0;
                overflow = false;
                return true;
            }
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
        else
        {
            overflow = true;
        }
    }

    return false;
}

bool uart_write(const char *text)
{
    if (text == 0)
    {
        return false;
    }

    size_t length = strlen(text);
    size_t head = tx_head;
    size_t tail = tx_tail;
    size_t used = head >= tail ? head - tail : UART_TX_QUEUE_SIZE - tail + head;
    size_t free = UART_TX_QUEUE_SIZE - used - 1U;

    if (length > free)
    {
        return false;
    }

    for (size_t i = 0; i < length; i++)
    {
        tx_queue[head] = (uint8_t)text[i];
        head = (head + 1U) % UART_TX_QUEUE_SIZE;
    }

    __DMB();
    tx_head = head;
    return true;
}

void uart_service(void)
{
    if (tx_active || tx_tail == tx_head)
    {
        return;
    }

    size_t tail = tx_tail;
    size_t head = tx_head;
    size_t length = head > tail ? head - tail : UART_TX_QUEUE_SIZE - tail;

    tx_in_flight = length;
    tx_active = true;
    if (HAL_UART_Transmit_IT(&huart2, &tx_queue[tail], (uint16_t)length) != HAL_OK)
    {
        tx_active = false;
        tx_in_flight = 0;
    }
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    tx_tail = (tx_tail + tx_in_flight) % UART_TX_QUEUE_SIZE;
    tx_in_flight = 0;
    tx_active = false;
}
