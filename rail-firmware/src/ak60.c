#include "ak60.h"

#define AK60_PACKET_MIT 8U
#define AK60_PACKET_ORIGIN 5U
#define AK60_PACKET_FEEDBACK 0x29U
#define AK60_P_MIN (-12.56f)
#define AK60_P_MAX 12.56f
#define AK60_V_MIN (-60.0f)
#define AK60_V_MAX 60.0f
#define AK60_T_MIN (-12.0f)
#define AK60_T_MAX 12.0f
#define AK60_KP_MIN 0.0f
#define AK60_KP_MAX 500.0f
#define AK60_KD_MIN 0.0f
#define AK60_KD_MAX 5.0f

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint16_t float_to_uint(float value, float minimum, float maximum,
                              uint8_t bits)
{
    float span = maximum - minimum;
    float offset = clampf(value, minimum, maximum) - minimum;
    return (uint16_t)(offset * (float)((1U << bits) - 1U) / span);
}

bool ak60_send_velocity_rad_s(float velocity_rad_s, float gain_kd)
{
    uint16_t kp = float_to_uint(0.0f, AK60_KP_MIN, AK60_KP_MAX, 12U);
    uint16_t kd = float_to_uint(gain_kd, AK60_KD_MIN, AK60_KD_MAX, 12U);
    uint16_t position = float_to_uint(0.0f, AK60_P_MIN, AK60_P_MAX, 16U);
    uint16_t velocity = float_to_uint(velocity_rad_s, AK60_V_MIN,
                                      AK60_V_MAX, 12U);
    uint16_t torque = float_to_uint(0.0f, AK60_T_MIN, AK60_T_MAX, 12U);
    uint8_t data[8] = {
        (uint8_t)(kp >> 4),
        (uint8_t)(((kp & 0x0FU) << 4) | (kd >> 8)),
        (uint8_t)kd,
        (uint8_t)(position >> 8),
        (uint8_t)position,
        (uint8_t)(velocity >> 4),
        (uint8_t)(((velocity & 0x0FU) << 4) | (torque >> 8)),
        (uint8_t)torque,
    };

    return can_send((AK60_PACKET_MIT << 8) | AK60_CAN_ID,
                    data, sizeof(data));
}

bool ak60_send_zero_torque(void)
{
    return ak60_send_velocity_rad_s(0.0f, 0.0f);
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
