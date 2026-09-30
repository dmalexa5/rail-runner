#include "protocol.h"

#include <stdio.h>
#include <string.h>

#include "uart.h"

static bool parse_setpoint(const char *text, float *value)
{
    const char *cursor = text;
    int sign = 1;
    int whole = 0;
    int fraction = 0;
    int digits = 0;

    if (*cursor == '+' || *cursor == '-')
    {
        sign = *cursor++ == '-' ? -1 : 1;
    }
    while (*cursor >= '0' && *cursor <= '9')
    {
        if (whole > 1000)
        {
            return false;
        }
        whole = whole * 10 + (*cursor++ - '0');
        digits++;
    }
    if (digits == 0)
    {
        return false;
    }
    if (*cursor == '.')
    {
        cursor++;
        if (*cursor < '0' || *cursor > '9')
        {
            return false;
        }
        fraction = *cursor++ - '0';
    }
    if (*cursor != '\0')
    {
        return false;
    }

    *value = (float)(sign * (whole * 10 + fraction)) / 10.0f;
    return true;
}

static bool parse_gain_kd(const char *text, float *value)
{
    const char *cursor = text;
    unsigned int whole = 0U;
    unsigned int fraction = 0U;
    unsigned int scale = 1U;
    unsigned int fraction_digits = 0U;

    if (*cursor < '0' || *cursor > '9')
    {
        return false;
    }
    while (*cursor >= '0' && *cursor <= '9')
    {
        whole = whole * 10U + (unsigned int)(*cursor++ - '0');
        if (whole > 1U)
        {
            return false;
        }
    }
    if (*cursor == '.')
    {
        cursor++;
        while (*cursor >= '0' && *cursor <= '9' && fraction_digits < 3U)
        {
            fraction = fraction * 10U + (unsigned int)(*cursor++ - '0');
            scale *= 10U;
            fraction_digits++;
        }
        if (fraction_digits == 0U)
        {
            return false;
        }
    }
    if (*cursor != '\0' || (whole == 1U && fraction != 0U))
    {
        return false;
    }

    *value = (float)whole + (float)fraction / (float)scale;
    return true;
}

bool protocol_read_request(protocol_request_t *request)
{
    char line[24];
    if (request == 0 || !uart_read_line(line, sizeof(line)))
    {
        return false;
    }

    request->value_mm_s = 0.0f;
    request->gain_kd = 0.0f;
    if (strcmp(line, "cal") == 0)
    {
        request->type = PROTOCOL_REQUEST_CALIBRATE;
    }
    else if (strcmp(line, "dis") == 0)
    {
        request->type = PROTOCOL_REQUEST_DISARM;
    }
    else if (strncmp(line, "sp ", 3) == 0 &&
             parse_setpoint(line + 3, &request->value_mm_s))
    {
        request->type = PROTOCOL_REQUEST_SETPOINT;
    }
    else if (strncmp(line, "kd ", 3) == 0 &&
             parse_gain_kd(line + 3, &request->gain_kd))
    {
        request->type = PROTOCOL_REQUEST_SET_KD;
    }
    else if (strcmp(line, "kd") == 0 || strncmp(line, "kd ", 3) == 0)
    {
        request->type = PROTOCOL_REQUEST_INVALID_KD;
    }
    else
    {
        request->type = PROTOCOL_REQUEST_INVALID;
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

bool protocol_write_response(const protocol_response_t *response)
{
    static const char *const fixed[] = {
        [PROTOCOL_RESPONSE_CALIBRATING] = "cal\n",
        [PROTOCOL_RESPONSE_DISARMED] = "dis\n",
        [PROTOCOL_RESPONSE_ERR_CAL] = "err cal\n",
        [PROTOCOL_RESPONSE_ERR_DIS] = "err dis\n",
        [PROTOCOL_RESPONSE_ERR_LIM] = "err lim\n",
        [PROTOCOL_RESPONSE_ERR_HRD] = "err hrd\n",
        [PROTOCOL_RESPONSE_ERR_EST] = "err est\n",
        [PROTOCOL_RESPONSE_ERR_POS] = "err pos\n",
        [PROTOCOL_RESPONSE_ERR_COM] = "err com\n",
        [PROTOCOL_RESPONSE_ERR_MOT] = "err mot\n",
        [PROTOCOL_RESPONSE_ERR_CAN] = "err can\n",
        [PROTOCOL_RESPONSE_ERR_CMD] = "err cmd\n",
        [PROTOCOL_RESPONSE_ERR_KD] = "err kd\n",
        [PROTOCOL_RESPONSE_ERR_SYS] = "err sys\n",
    };
    char line[64];
    char position[16];
    char velocity[16];
    char acceleration[16];

    if (response == 0 || response->type == PROTOCOL_RESPONSE_NONE)
    {
        return true;
    }
    if (response->type == PROTOCOL_RESPONSE_ACK)
    {
        format_tenth(position, sizeof(position), response->position_mm);
        format_tenth(velocity, sizeof(velocity), response->velocity_mm_s);
        format_tenth(acceleration, sizeof(acceleration),
                     response->acceleration_mm_s2);
        snprintf(line, sizeof(line), "ack %s %s %s\n",
                 position, velocity, acceleration);
        return uart_write(line);
    }
    if (response->type == PROTOCOL_RESPONSE_KD)
    {
        unsigned long thousandths =
            (unsigned long)(response->gain_kd * 1000.0f + 0.5f);
        snprintf(line, sizeof(line), "kd %lu.%03lu\n",
                 thousandths / 1000UL, thousandths % 1000UL);
        return uart_write(line);
    }
    if ((unsigned int)response->type >=
        sizeof(fixed) / sizeof(fixed[0]) || fixed[response->type] == 0)
    {
        return false;
    }
    return uart_write(fixed[response->type]);
}
