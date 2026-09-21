#include "ak60.h"

#include <limits.h>

#define AK60_PACKET_CURRENT 1U
#define AK60_PACKET_VELOCITY 3U
#define AK60_PACKET_ORIGIN 5U
#define AK60_PACKET_FEEDBACK 0x29U

static void pack_i32(uint8_t data[4], int32_t value)
{
    data[0] = (uint8_t)((uint32_t)value >> 24);
    data[1] = (uint8_t)((uint32_t)value >> 16);
    data[2] = (uint8_t)((uint32_t)value >> 8);
    data[3] = (uint8_t)value;
}

bool ak60_send_velocity_erpm(float velocity_erpm)
{
    uint8_t data[4];
    if (velocity_erpm > (float)INT32_MAX || velocity_erpm < (float)INT32_MIN)
    {
        return false;
    }

    int32_t command = (int32_t)(velocity_erpm >= 0.0f ?
                                velocity_erpm + 0.5f : velocity_erpm - 0.5f);
    pack_i32(data, command);
    return can_send((AK60_PACKET_VELOCITY << 8) | AK60_CAN_ID,
                    data, sizeof(data));
}

bool ak60_send_zero_current(void)
{
    uint8_t data[4] = {0};
    return can_send((AK60_PACKET_CURRENT << 8) | AK60_CAN_ID,
                    data, sizeof(data));
}

bool ak60_set_temporary_origin(void)
{
    uint8_t mode = 0U;
    return can_send((AK60_PACKET_ORIGIN << 8) | AK60_CAN_ID, &mode, 1U);
}

bool ak60_parse_feedback(const can_frame_t *frame, ak60_state_t *state)
{
    if (frame == 0 || state == 0 || frame->len != 8U ||
        frame->id != ((AK60_PACKET_FEEDBACK << 8) | AK60_CAN_ID))
    {
        return false;
    }

    state->position_counts = (int16_t)(((uint16_t)frame->data[0] << 8) |
                                        frame->data[1]);
    state->velocity_erpm = (int16_t)(((uint16_t)frame->data[2] << 8) |
                                      frame->data[3]);
    state->current_centiamps = (int16_t)(((uint16_t)frame->data[4] << 8) |
                                         frame->data[5]);
    state->temperature_c = (int8_t)frame->data[6];
    state->error = frame->data[7];
    return true;
}
