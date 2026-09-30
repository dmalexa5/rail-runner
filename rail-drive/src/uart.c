#include "uart.h"

#include <string.h>

#include "board.h"

#define UART_RX_QUEUE_SIZE 64U
#define UART_TX_QUEUE_SIZE 128U

static UART_HandleTypeDef huart2;
static uint8_t rx_byte;
static uint8_t rx_queue[UART_RX_QUEUE_SIZE];
static volatile size_t rx_head;
static volatile size_t rx_tail;
static volatile bool rx_overflow;
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

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    return HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) == HAL_OK;
}

bool uart_read_line(char *line, size_t line_size)
{
    static char buffer[24];
    static size_t used;
    static bool line_overflow;

    if (line == 0 || line_size == 0U)
    {
        return false;
    }

    while (rx_tail != rx_head)
    {
        uint8_t ch = rx_queue[rx_tail];
        rx_tail = (rx_tail + 1U) % UART_RX_QUEUE_SIZE;

        if (ch == '\r' || ch == '\n')
        {
            if (used == 0U && !line_overflow)
            {
                continue;
            }
            if (line_overflow || used >= line_size)
            {
                line[0] = '\0';
            }
            else
            {
                memcpy(line, buffer, used);
                line[used] = '\0';
            }
            used = 0U;
            line_overflow = false;
            return true;
        }

        if (used + 1U < sizeof(buffer))
        {
            buffer[used++] = (char)ch;
        }
        else
        {
            line_overflow = true;
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
    if (length > UART_TX_QUEUE_SIZE - used - 1U)
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
    if (HAL_UART_Transmit_IT(&huart2, &tx_queue[tail],
                             (uint16_t)length) != HAL_OK)
    {
        tx_active = false;
        tx_in_flight = 0U;
    }
}

bool uart_take_rx_overflow(void)
{
    bool overflow = rx_overflow;
    rx_overflow = false;
    return overflow;
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    size_t next = (rx_head + 1U) % UART_RX_QUEUE_SIZE;
    if (next == rx_tail)
    {
        rx_overflow = true;
    }
    else
    {
        rx_queue[rx_head] = rx_byte;
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) != HAL_OK)
    {
        rx_overflow = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    rx_overflow = true;
    if (huart2.RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }
    tx_tail = (tx_tail + tx_in_flight) % UART_TX_QUEUE_SIZE;
    tx_in_flight = 0U;
    tx_active = false;
}
