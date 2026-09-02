#include "ak60.h"

#define AK60_PACKET_MIT 8U
#define AK60_P_MIN (-12.56f)
#define AK60_P_MAX (12.56f)
#define AK60_V_MIN (-60.0f)
#define AK60_V_MAX (60.0f)
#define AK60_T_MIN (-12.0f)
#define AK60_T_MAX (12.0f)
#define AK60_KP_MIN (0.0f)
#define AK60_KP_MAX (500.0f)
#define AK60_KD_MIN (0.0f)
#define AK60_KD_MAX (5.0f)

static uint32_t motor_id = AK60_DEFAULT_CAN_ID;

void ak60_set_id(uint32_t id)
{
    motor_id = id;
}

uint32_t ak60_get_id(void)
{
    return motor_id;
}

static float clampf(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

static uint16_t float_to_uint(float value, float min, float max, uint8_t bits)
{
    float span = max - min;
    float offset = clampf(value, min, max) - min;
    return (uint16_t)((offset * (float)((1U << bits) - 1U)) / span);
}

bool ak60_power_on(void)
{
    return ak60_send_torque_nm(0.0f);
}

bool ak60_power_off(void)
{
    return ak60_send_torque_nm(0.0f);
}

bool ak60_send_torque_nm(float torque_nm)
{
    uint16_t p = float_to_uint(0.0f, AK60_P_MIN, AK60_P_MAX, 16);
    uint16_t v = float_to_uint(0.0f, AK60_V_MIN, AK60_V_MAX, 12);
    uint16_t kp = float_to_uint(0.0f, AK60_KP_MIN, AK60_KP_MAX, 12);
    uint16_t kd = float_to_uint(0.0f, AK60_KD_MIN, AK60_KD_MAX, 12);
    uint16_t t = float_to_uint(clampf(torque_nm,
                                      -AK60_DEBUG_TORQUE_LIMIT_NM,
                                      AK60_DEBUG_TORQUE_LIMIT_NM),
                               AK60_T_MIN,
                               AK60_T_MAX,
                               12);

    uint8_t data[8] = {
        (uint8_t)(kp >> 4),
        (uint8_t)(((kp & 0x0F) << 4) | (kd >> 8)),
        (uint8_t)(kd & 0xFF),
        (uint8_t)(p >> 8),
        (uint8_t)(p & 0xFF),
        (uint8_t)(v >> 4),
        (uint8_t)(((v & 0x0F) << 4) | (t >> 8)),
        (uint8_t)(t & 0xFF),
    };

    return can_send((AK60_PACKET_MIT << 8) | motor_id, data);
}

bool ak60_parse_feedback(const can_frame_t *frame, ak60_state_t *state)
{
    if (frame == 0 || state == 0 || frame->len < 6)
    {
        return false;
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        state->raw[i] = frame->data[i];
    }

    state->can_id = frame->id;

    int16_t p = (int16_t)(((uint16_t)frame->data[0] << 8) | frame->data[1]);
    int16_t v = (int16_t)(((uint16_t)frame->data[2] << 8) | frame->data[3]);
    int16_t t = (int16_t)(((uint16_t)frame->data[4] << 8) | frame->data[5]);

    state->id = (uint8_t)(frame->id & 0xFFU);
    state->position_rad = (float)p * 0.1f * 0.01745329252f;
    state->velocity_rad_s = (float)v * 10.0f;
    state->torque_nm = (float)t * 0.01f;
    state->temperature_c = frame->len >= 7 ? frame->data[6] : 0;
    state->error = frame->len >= 8 ? frame->data[7] : 0;
    state->valid = (frame->id & 0xFFU) == motor_id;

    return state->valid;
}
