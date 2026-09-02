#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
} can_frame_t;

typedef struct
{
    uint32_t error;
    uint32_t state;
    uint32_t tx_mailboxes_free;
    uint32_t rx_fifo0_pending;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t last_tx_ok;
    uint32_t last_tx_fail;
} can_status_t;

/** Initializes CAN1 at 1 Mbit/s and enables RX FIFO0 interrupts. */
bool can_init(void);

/** Queues one eight-byte extended-ID CAN data frame for transmission. */
bool can_send(uint32_t id, const uint8_t data[8]);

/**
 * Copies the newest frame captured by the CAN RX interrupt.
 *
 * Returns false until a frame has been received or when either argument is
 * null. The monotonically increasing sequence identifies a new snapshot.
 */
bool can_read_latest(can_frame_t *frame, uint32_t *sequence);

/** Copies CAN peripheral and driver diagnostic counters. */
void can_get_status(can_status_t *status);

#endif
