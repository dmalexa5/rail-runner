#include "teleop_protocol.h"

#include <stdio.h>
#include <string.h>

#include "uart.h"

bool teleop_protocol_read_request(teleop_request_t *request)
{
    char line[24];
    if (request == 0 || !uart_read_line(line, sizeof(line)))
    {
        return false;
    }

    if (strcmp(line, "cal") == 0)
    {
        *request = TELEOP_REQUEST_CALIBRATE;
    }
    else if (strcmp(line, "dis") == 0)
    {
        *request = TELEOP_REQUEST_DISARM;
    }
    else if (strcmp(line, "req") == 0)
    {
        *request = TELEOP_REQUEST_STATE;
    }
    else
    {
        *request = TELEOP_REQUEST_INVALID;
    }
    return true;
}

static void format_tenth(char *buffer, size_t size, float value)
{
    long tenths = (long)(value >= 0.0f ? value * 10.0f + 0.5f :
                                      value * 10.0f - 0.5f);
    if (tenths == 0L)
    {
        snprintf(buffer, size, "0.0");
        return;
    }

    unsigned long magnitude = tenths < 0L ?
                              (unsigned long)(-tenths) : (unsigned long)tenths;
    snprintf(buffer, size, "%s%lu.%lu", tenths < 0L ? "-" : "",
             magnitude / 10UL, magnitude % 10UL);
}

bool teleop_protocol_write_response(const teleop_response_t *response)
{
    static const char *const fixed[] = {
        [TELEOP_RESPONSE_CALIBRATING] = "cal\n",
        [TELEOP_RESPONSE_DISARMED] = "dis\n",
        [TELEOP_RESPONSE_ERR_CAL] = "err cal\n",
        [TELEOP_RESPONSE_ERR_DIS] = "err dis\n",
        [TELEOP_RESPONSE_ERR_HRD] = "err hrd\n",
        [TELEOP_RESPONSE_ERR_COM] = "err com\n",
        [TELEOP_RESPONSE_ERR_JOY] = "err joy\n",
        [TELEOP_RESPONSE_ERR_CMD] = "err cmd\n",
        [TELEOP_RESPONSE_ERR_SYS] = "err sys\n",
    };
    char line[48];
    char position[16];
    char velocity[16];

    if (response == 0 || response->type == TELEOP_RESPONSE_NONE)
    {
        return true;
    }
    if (response->type == TELEOP_RESPONSE_ACK)
    {
        format_tenth(position, sizeof(position), response->position_mm);
        format_tenth(velocity, sizeof(velocity), response->velocity_mm_s);
        snprintf(line, sizeof(line), "ack %s %s\n", position, velocity);
        return uart_write(line);
    }
    if ((unsigned int)response->type >=
        sizeof(fixed) / sizeof(fixed[0]) || fixed[response->type] == 0)
    {
        return false;
    }
    return uart_write(fixed[response->type]);
}
