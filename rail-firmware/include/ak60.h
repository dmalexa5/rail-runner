#ifndef AK60_H
#define AK60_H

#include <stdbool.h>
#include <stdint.h>

#include "can.h"

#define AK60_CAN_ID 2U
#define AK60_POLE_PAIRS 14.0f
#define AK60_REDUCTION 6.0f
#define AK60_RATED_OUTPUT_RPM 490.0f
#define AK60_DEFAULT_KD 0.1f

typedef struct
{
    int16_t position_counts;
    int16_t velocity_erpm;
    int16_t current_centiamps;
    int8_t temperature_c;
    uint8_t error;
} ak60_state_t;

/** Sends an MIT-mode output-shaft velocity command with derivative gain. */
bool ak60_send_velocity_rad_s(float velocity_rad_s, float gain_kd);

/** Sends an MIT-mode command with all five logical fields set to zero. */
bool ak60_send_zero_torque(void);

/** Sets the current motor position as the temporary origin. */
bool ak60_set_temporary_origin(void);

/** Decodes one periodic feedback frame for the configured motor ID. */
bool ak60_parse_feedback(const can_frame_t *frame, ak60_state_t *state);

#endif
