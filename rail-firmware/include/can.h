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

bool can_init(void);
bool can_send(uint32_t id, const uint8_t data[8]);
bool can_recv(can_frame_t *frame);
void can_get_status(can_status_t *status);

#endif
