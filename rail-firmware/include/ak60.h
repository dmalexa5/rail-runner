#ifndef AK60_H
#define AK60_H

#include <stdbool.h>
#include <stdint.h>

#include "can.h"

#define AK60_DEFAULT_CAN_ID 2U

typedef struct
{
    uint8_t id;
    uint32_t can_id;
    uint8_t raw[8];
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint8_t temperature_c;
    uint8_t error;
    bool valid;
} ak60_state_t;

void ak60_set_id(uint32_t id);
uint32_t ak60_get_id(void);
bool ak60_power_on(void);
bool ak60_power_off(void);

/** Sends one MIT-mode torque command, clamped to the AK60 protocol range. */
bool ak60_send_torque_nm(float torque_nm);

/** Decodes feedback addressed to the configured motor ID. */
bool ak60_parse_feedback(const can_frame_t *frame, ak60_state_t *state);

#endif
