#ifndef TELEOP_PROTOCOL_H
#define TELEOP_PROTOCOL_H

#include <stdbool.h>

typedef enum
{
    TELEOP_REQUEST_NONE = 0,
    TELEOP_REQUEST_CALIBRATE,
    TELEOP_REQUEST_DISARM,
    TELEOP_REQUEST_STATE,
    TELEOP_REQUEST_INVALID
} teleop_request_t;

typedef enum
{
    TELEOP_RESPONSE_NONE = 0,
    TELEOP_RESPONSE_CALIBRATING,
    TELEOP_RESPONSE_DISARMED,
    TELEOP_RESPONSE_ACK,
    TELEOP_RESPONSE_ERR_CAL,
    TELEOP_RESPONSE_ERR_DIS,
    TELEOP_RESPONSE_ERR_HRD,
    TELEOP_RESPONSE_ERR_COM,
    TELEOP_RESPONSE_ERR_JOY,
    TELEOP_RESPONSE_ERR_CMD,
    TELEOP_RESPONSE_ERR_SYS
} teleop_response_type_t;

typedef struct
{
    teleop_response_type_t type;
    float position_mm;
    float velocity_mm_s;
} teleop_response_t;

bool teleop_protocol_read_request(teleop_request_t *request);
bool teleop_protocol_write_response(const teleop_response_t *response);

#endif
