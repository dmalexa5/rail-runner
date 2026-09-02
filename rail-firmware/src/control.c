#include "control.h"

#include <limits.h>

#include "board.h"
#include "can.h"

static volatile bool cycle_pending;
static volatile bool cycle_active;
static volatile bool overrun_fault_pending;
static volatile uint32_t timer_ticks;
static volatile uint32_t overrun_count;

static volatile bool armed;
static bool safety_switches_ok;
static bool feedback_seen;
static volatile float torque_command_nm;
static volatile control_fault_t fault;
static uint32_t cycle_count;
static uint32_t max_execution_cycles;
static uint32_t last_arm_tick;
static uint32_t last_feedback_tick;
static uint32_t feedback_watchdog_tick;
static uint32_t feedback_sequence;
static ak60_state_t motor_state;

static void disarm_with_fault(control_fault_t reason)
{
    armed = false;
    torque_command_nm = 0.0f;
    fault = reason;
}

void control_init(void)
{
    safety_switches_ok = board_safety_switches_ok();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

bool control_arm(void)
{
    bool was_armed = armed;
    safety_switches_ok = board_safety_switches_ok();
    if (overrun_fault_pending)
    {
        disarm_with_fault(CONTROL_FAULT_OVERRUN);
        return false;
    }
    if (!safety_switches_ok)
    {
        disarm_with_fault(CONTROL_FAULT_SAFETY_SWITCH);
        return false;
    }

    armed = true;
    fault = CONTROL_FAULT_NONE;
    last_arm_tick = timer_ticks;
    if (!was_armed)
    {
        feedback_watchdog_tick = timer_ticks;
    }
    return true;
}

void control_disarm(void)
{
    armed = false;
    torque_command_nm = 0.0f;
    fault = CONTROL_FAULT_NONE;
}

bool control_set_torque(float torque_nm)
{
    if (!armed ||
        !(torque_nm >= -CONTROL_TORQUE_LIMIT_NM &&
          torque_nm <= CONTROL_TORQUE_LIMIT_NM))
    {
        return false;
    }

    torque_command_nm = torque_nm;
    return true;
}

void control_get_status(control_status_t *status)
{
    if (status == 0)
    {
        return;
    }

    uint32_t now = timer_ticks;
    status->armed = armed;
    status->safety_switches_ok = safety_switches_ok;
    status->torque_command_nm = torque_command_nm;
    status->fault = fault;
    status->cycle_count = cycle_count;
    status->overrun_count = overrun_count;
    status->max_execution_cycles = max_execution_cycles;
    status->arm_lease_age_ms = now - last_arm_tick;
    status->feedback_age_ms = feedback_seen ? now - last_feedback_tick : UINT32_MAX;
    status->feedback_sequence = feedback_sequence;
    status->motor = motor_state;
}

static void consume_latest_feedback(uint32_t now)
{
    can_frame_t frame;
    uint32_t sequence;

    if (!can_read_latest(&frame, &sequence) || sequence == feedback_sequence)
    {
        return;
    }

    feedback_sequence = sequence;
    if (ak60_parse_feedback(&frame, &motor_state))
    {
        feedback_seen = true;
        last_feedback_tick = now;
        feedback_watchdog_tick = now;
    }
}

static void run_control_cycle(void)
{
    uint32_t now = timer_ticks;
    safety_switches_ok = board_safety_switches_ok();
    consume_latest_feedback(now);

    if (overrun_fault_pending)
    {
        overrun_fault_pending = false;
        disarm_with_fault(CONTROL_FAULT_OVERRUN);
    }
    else if (!safety_switches_ok)
    {
        disarm_with_fault(CONTROL_FAULT_SAFETY_SWITCH);
    }
    else if (armed && now - last_arm_tick >= CONTROL_ARM_LEASE_MS)
    {
        disarm_with_fault(CONTROL_FAULT_ARM_LEASE);
    }
    else if (armed && now - feedback_watchdog_tick >= CONTROL_FEEDBACK_TIMEOUT_MS)
    {
        disarm_with_fault(CONTROL_FAULT_FEEDBACK_TIMEOUT);
    }

    float command_nm = armed ? torque_command_nm : 0.0f;
    if (!ak60_send_torque_nm(command_nm))
    {
        disarm_with_fault(CONTROL_FAULT_CAN_TX);
    }
}

bool control_run_pending(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!cycle_pending)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return false;
    }

    cycle_pending = false;
    cycle_active = true;
    if (primask == 0U)
    {
        __enable_irq();
    }

    uint32_t started = DWT->CYCCNT;
    run_control_cycle();
    uint32_t elapsed = DWT->CYCCNT - started;

    cycle_count++;
    if (elapsed > max_execution_cycles)
    {
        max_execution_cycles = elapsed;
    }
    cycle_active = false;
    return true;
}

void control_timer_tick_isr(void)
{
    timer_ticks++;
    if (cycle_pending || cycle_active)
    {
        overrun_count++;
        overrun_fault_pending = true;
        armed = false;
        torque_command_nm = 0.0f;
        fault = CONTROL_FAULT_OVERRUN;
    }
    cycle_pending = true;
}
