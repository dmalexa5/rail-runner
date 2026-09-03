#include "protocol.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can.h"
#include "control.h"
#include "uart.h"

static void trim_line(char *line)
{
    size_t length = strlen(line);
    while (length > 0U && isspace((unsigned char)line[length - 1U]))
    {
        line[--length] = '\0';
    }
}

static void write_status(void)
{
    control_status_t control;
    can_status_t can;
    char line[384];

    control_get_status(&control);
    can_get_status(&can);
    snprintf(line, sizeof(line),
             "s,%u,%u,%u,%ld,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%ld,%ld,%ld,%u,%u,%lx,%lu,%lu\n",
             control.armed ? 1U : 0U,
             control.safety_switches_ok ? 1U : 0U,
             (unsigned int)control.fault,
             (long)(control.torque_command_nm * 1000.0f),
             (unsigned long)control.cycle_count,
             (unsigned long)control.overrun_count,
             (unsigned long)control.max_execution_cycles,
             (unsigned long)control.arm_lease_age_ms,
             (unsigned long)control.feedback_age_ms,
             (unsigned long)control.feedback_sequence,
             control.motor.valid ? 1U : 0U,
             control.motor.id,
             (long)(control.motor.position_rad * 1000.0f),
             (long)(control.motor.velocity_rad_s * 1000.0f),
             (long)(control.motor.torque_nm * 1000.0f),
             control.motor.temperature_c,
             control.motor.error,
             (unsigned long)can.error,
             (unsigned long)can.last_tx_ok,
             (unsigned long)can.last_tx_fail);
    uart_write(line);
}

static void write_feedback(void)
{
    control_status_t control;
    char line[64];

    control_get_status(&control);
    snprintf(line, sizeof(line),
             "f,%ld,%ld,%ld\n",
             (long)(control.motor.position_rad * 1000.0f),
             (long)(control.motor.velocity_rad_s * 1000.0f),
             (long)(control.motor.torque_nm * 1000.0f));
    uart_write(line);
}

void protocol_handle_line(char *line)
{
    if (line == 0)
    {
        return;
    }

    trim_line(line);
    if (strcmp(line, "a") == 0)
    {
        uart_write(control_arm() ? "k,a\n" : "e,a\n");
    }
    else if (strcmp(line, "d") == 0)
    {
        control_disarm();
        uart_write("k,d\n");
    }
    else if (strncmp(line, "t,", 2) == 0)
    {
        char *end = 0;
        float torque_nm = strtof(line + 2, &end);
        if (end == line + 2 || *end != '\0' || !isfinite(torque_nm))
        {
            uart_write("e,i\n");
        }
        else if (!control_set_torque(torque_nm))
        {
            uart_write("e,r\n");
        }
        else
        {
            uart_write("k,t\n");
        }
    }
    else if (strcmp(line, "s") == 0)
    {
        write_status();
    }
    else if (strcmp(line, "f") == 0)
    {
        write_feedback();
    }
    else
    {
        uart_write("e,c\n");
    }
}
