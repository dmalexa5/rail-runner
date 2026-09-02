#include "can.h"

#include <string.h>

#include "board.h"

static CAN_HandleTypeDef hcan1;
static volatile uint32_t tx_ok_count;
static volatile uint32_t tx_fail_count;
static can_frame_t latest_frame;
static volatile uint32_t latest_sequence;

bool can_init(void)
{
    __HAL_RCC_CAN1_CLK_ENABLE();

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 1;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        return false;
    }

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        return false;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    return HAL_CAN_ActivateNotification(&hcan1,
                                         CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
}

bool can_send(uint32_t id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox = 0;
    uint8_t payload[8];

    memcpy(payload, data, sizeof(payload));

    header.ExtId = id;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &header, payload, &mailbox) == HAL_OK)
    {
        tx_ok_count++;
        return true;
    }

    tx_fail_count++;
    return false;
}

bool can_read_latest(can_frame_t *frame, uint32_t *sequence)
{
    if (frame == 0 || sequence == 0 || latest_sequence == 0)
    {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *frame = latest_frame;
    *sequence = latest_sequence;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return true;
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef header = {0};
        can_frame_t frame = {0};

        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, frame.data) != HAL_OK)
        {
            return;
        }

        if (header.IDE != CAN_ID_EXT || header.RTR != CAN_RTR_DATA)
        {
            continue;
        }

        frame.id = header.ExtId;
        frame.len = (uint8_t)header.DLC;
        latest_frame = frame;
        latest_sequence++;
    }
}

void can_get_status(can_status_t *status)
{
    if (status == 0)
    {
        return;
    }

    status->error = HAL_CAN_GetError(&hcan1);
    status->state = (uint32_t)HAL_CAN_GetState(&hcan1);
    status->tx_mailboxes_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    status->rx_fifo0_pending = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
    status->tx_error_count = (CAN1->ESR & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
    status->rx_error_count = (CAN1->ESR & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
    status->last_tx_ok = tx_ok_count;
    status->last_tx_fail = tx_fail_count;
}
