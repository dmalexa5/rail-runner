#include "board.h"
#include "ak60.h"
#include "can.h"
#include "uart.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

static bool streq(const char *a, const char *b)
{
    return strcmp(a, b) == 0;
}

static void trim_line(char *line)
{
    size_t len = strlen(line);

    while (len > 0 && isspace((unsigned char)line[len - 1]))
    {
        line[--len] = '\0';
    }
}

static void print_help(void)
{
    uart_write("\r\nCommands:\r\n");
    uart_write("  en     enable motor\r\n");
    uart_write("  dis    disable motor\r\n");
    uart_write("  s      print latest status\r\n");
    uart_write("  c      print CAN status\r\n");
    uart_write("  id <n> set motor CAN id\r\n");
    uart_write("  h      print help\r\n");
    uart_write("  <Nm>   set torque, clipped to +/-1.0 Nm\r\n\r\n");
}

static void print_can_status(void)
{
    can_status_t status;
    char line[160];

    can_get_status(&status);
    snprintf(line, sizeof(line),
             "can err=0x%08lx state=%lu tx_free=%lu rx_pending=%lu tec=%lu rec=%lu tx_ok=%lu tx_fail=%lu\r\n",
             (unsigned long)status.error,
             (unsigned long)status.state,
             (unsigned long)status.tx_mailboxes_free,
             (unsigned long)status.rx_fifo0_pending,
             (unsigned long)status.tx_error_count,
             (unsigned long)status.rx_error_count,
             (unsigned long)status.last_tx_ok,
             (unsigned long)status.last_tx_fail);
    uart_write(line);
}

static void print_status(bool enabled, float torque, const ak60_state_t *state)
{
    char line[220];

    snprintf(line, sizeof(line),
             "enabled=%u motor_id=%lu cmd_mNm=%ld feedback=%u fb_id=%u fb_can=0x%lx pos_mrad=%ld vel_mrad_s=%ld torque_mNm=%ld temp=%u err=%u raw=%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
             enabled ? 1U : 0U,
             (unsigned long)ak60_get_id(),
             (long)(torque * 1000.0f),
             state->valid ? 1U : 0U,
             state->id,
             (unsigned long)state->can_id,
             (long)(state->position_rad * 1000.0f),
             (long)(state->velocity_rad_s * 1000.0f),
             (long)(state->torque_nm * 1000.0f),
             state->temperature_c,
             state->error,
             state->raw[0],
             state->raw[1],
             state->raw[2],
             state->raw[3],
             state->raw[4],
             state->raw[5],
             state->raw[6],
             state->raw[7]);
    uart_write(line);
}

static float clamp_torque(float torque)
{
    if (torque < -AK60_DEBUG_TORQUE_LIMIT_NM)
    {
        return -AK60_DEBUG_TORQUE_LIMIT_NM;
    }
    if (torque > AK60_DEBUG_TORQUE_LIMIT_NM)
    {
        return AK60_DEBUG_TORQUE_LIMIT_NM;
    }
    return torque;
}

int main(void)
{
    bool enabled = false;
    float torque_nm = 0.0f;
    uint32_t last_command_ms = 0;
    uint32_t last_led_ms = 0;
    ak60_state_t state = {0};
    char line[32];
    can_frame_t frame;

    board_init();
    uart_init();

    if (!can_init())
    {
        uart_write("CAN init failed\r\n");
        while (1)
        {
            HAL_GPIO_TogglePin(BOARD_LED_GPIO_PORT, BOARD_LED_PIN);
            HAL_Delay(100);
        }
    }

    uart_write("\r\nAK60-6 MIT debug console ready. Motor disabled.\r\n");
    print_help();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        while (can_recv(&frame))
        {
            ak60_parse_feedback(&frame, &state);
        }

        if (uart_read_line(line, sizeof(line)))
        {
            trim_line(line);

            if (streq(line, "en"))
            {
                torque_nm = 0.0f;
                enabled = ak60_power_on();
                last_command_ms = 0;
                uart_write(enabled ? "enabled\r\n" : "enable failed\r\n");
            }
            else if (streq(line, "dis"))
            {
                torque_nm = 0.0f;
                for (uint8_t i = 0; i < 5; i++)
                {
                    ak60_send_torque_nm(0.0f);
                    HAL_Delay(AK60_COMMAND_PERIOD_MS);
                }
                ak60_power_off();
                enabled = false;
                uart_write("disabled\r\n");
            }
            else if (streq(line, "s"))
            {
                print_status(enabled, torque_nm, &state);
            }
            else if (streq(line, "c"))
            {
                print_can_status();
            }
            else if (strncmp(line, "id ", 3) == 0)
            {
                char *end = 0;
                unsigned long id = strtoul(line + 3, &end, 0);

                if (end != line + 3 && *end == '\0' && id <= 0x7FFUL)
                {
                    torque_nm = 0.0f;
                    enabled = false;
                    ak60_set_id((uint32_t)id);
                    uart_write("motor id updated; re-enable motor\r\n");
                }
                else
                {
                    uart_write("invalid id\r\n");
                }
            }
            else if (streq(line, "h"))
            {
                print_help();
            }
            else
            {
                char *end = 0;
                float requested = strtof(line, &end);

                if (end != line && *end == '\0')
                {
                    if (!enabled)
                    {
                        uart_write("enable before torque\r\n");
                    }
                    else
                    {
                        torque_nm = clamp_torque(requested);
                        uart_write("torque updated\r\n");
                    }
                }
                else
                {
                    uart_write("unknown command\r\n");
                }
            }
        }

        if (enabled && (now - last_command_ms) >= AK60_COMMAND_PERIOD_MS)
        {
            if (!ak60_send_torque_nm(torque_nm))
            {
                uart_write("torque tx failed\r\n");
                print_can_status();
                enabled = false;
                uart_write("command loop stopped\r\n");
            }
            last_command_ms = now;
        }

        if ((now - last_led_ms) >= (enabled ? 100U : 500U))
        {
            HAL_GPIO_TogglePin(BOARD_LED_GPIO_PORT, BOARD_LED_PIN);
            last_led_ms = now;
        }
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
