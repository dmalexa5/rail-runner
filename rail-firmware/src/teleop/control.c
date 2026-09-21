#include "control.h"

#include <math.h>

#include "board.h"
#include "joystick.h"
#include "stepper.h"
#include "teleop_protocol.h"
#include "uart.h"

#define HOST_TIMEOUT_MS 20U
#define JOYSTICK_TIMEOUT_MS 20U
#define LIMIT_CLEAR_CYCLES 5U
#define JOYSTICK_CENTER 2047.5f
#define JOYSTICK_MAX 4095.0f
#define JOYSTICK_DEADBAND 0.05f
#define SCALE 0.5f
#define MM_PER_REV 20.0f
#define PULSES_PER_REV 1600.0f
#define PULSES_PER_VIRTUAL_MM (SCALE * PULSES_PER_REV / MM_PER_REV)
#define VIRTUAL_MM_PER_PULSE (1.0f / PULSES_PER_VIRTUAL_MM)
#define CAL_SEEK_LIMIT_MM 500.0f
#define CAL_BACKOFF_LIMIT_MM 10.0f

typedef enum
{
    STATE_DEACTIVATED = 0,
    STATE_CALIBRATING,
    STATE_ACTIVE,
    STATE_STOPPING,
    STATE_FAULT
} teleop_state_t;

typedef enum
{
    CAL_SEEK_MIN = 0,
    CAL_BACK_OFF,
    CAL_BRAKE
} calibration_phase_t;

typedef enum
{
    FAULT_NONE = 0,
    FAULT_HARD_LIMIT,
    FAULT_SYSTEM,
    FAULT_JOYSTICK,
    FAULT_COMMUNICATION,
    FAULT_CALIBRATION
} teleop_fault_t;

static volatile bool cycle_pending;
static volatile bool cycle_active;
static volatile bool overrun_pending;
static volatile uint32_t timer_ticks;

static teleop_state_t state;
static calibration_phase_t calibration_phase;
static teleop_fault_t fault;
static teleop_fault_t stop_fault;
static teleop_response_type_t pending_response;
static bool dis_reply_pending;
static float requested_velocity_mm_s;
static float command_velocity_mm_s;
static float command_acceleration_mm_s2;
static int32_t origin_steps;
static int32_t calibration_start_steps;
static int32_t calibration_phase_steps;
static int32_t clear_edge_steps;
static uint8_t clear_count;
static bool calibration_complete_pending;
static bool operator_armed;
static bool direction_positive = true;
static uint8_t direction_wait_cycles;
static uint16_t joystick_raw;
static uint32_t joystick_sequence;
static uint32_t last_joystick_tick;
static uint32_t last_host_tick;

static teleop_response_type_t fault_response(teleop_fault_t reason)
{
    switch (reason)
    {
        case FAULT_HARD_LIMIT: return TELEOP_RESPONSE_ERR_HRD;
        case FAULT_SYSTEM: return TELEOP_RESPONSE_ERR_SYS;
        case FAULT_JOYSTICK: return TELEOP_RESPONSE_ERR_JOY;
        case FAULT_COMMUNICATION: return TELEOP_RESPONSE_ERR_COM;
        case FAULT_CALIBRATION: return TELEOP_RESPONSE_ERR_CAL;
        default: return TELEOP_RESPONSE_NONE;
    }
}

static void set_fault(teleop_fault_t reason)
{
    if (reason == FAULT_NONE)
    {
        return;
    }
    if (fault == FAULT_NONE || reason < fault)
    {
        fault = reason;
    }
    state = STATE_FAULT;
    stop_fault = FAULT_NONE;
    requested_velocity_mm_s = 0.0f;
    command_velocity_mm_s = 0.0f;
    command_acceleration_mm_s2 = 0.0f;
    calibration_complete_pending = false;
    operator_armed = false;
    stepper_stop();
    stepper_enable(false);
    if (dis_reply_pending)
    {
        pending_response = fault_response(fault);
        dis_reply_pending = false;
    }
}

static void begin_stop(teleop_fault_t reason, bool reply_when_stopped)
{
    state = STATE_STOPPING;
    stop_fault = reason;
    requested_velocity_mm_s = 0.0f;
    calibration_complete_pending = false;
    operator_armed = false;
    if (reply_when_stopped)
    {
        dis_reply_pending = true;
    }
}

static bool profile_stopped(void)
{
    return command_velocity_mm_s == 0.0f &&
           command_acceleration_mm_s2 == 0.0f;
}

static float position_mm(void)
{
    return (float)(stepper_position_steps() - origin_steps) *
           VIRTUAL_MM_PER_PULSE;
}

static float travel_from(int32_t start_steps)
{
    int32_t delta = stepper_position_steps() - start_steps;
    return fabsf((float)delta * VIRTUAL_MM_PER_PULSE);
}

static float joystick_velocity(void)
{
    float normalized;
    if ((float)joystick_raw >= JOYSTICK_CENTER)
    {
        normalized = ((float)joystick_raw - JOYSTICK_CENTER) /
                     (JOYSTICK_MAX - JOYSTICK_CENTER);
    }
    else
    {
        normalized = ((float)joystick_raw - JOYSTICK_CENTER) /
                     JOYSTICK_CENTER;
    }

    float magnitude = fabsf(normalized);
    if (magnitude <= JOYSTICK_DEADBAND)
    {
        return 0.0f;
    }
    magnitude = (magnitude - JOYSTICK_DEADBAND) /
                (1.0f - JOYSTICK_DEADBAND);
    if (magnitude > 1.0f)
    {
        magnitude = 1.0f;
    }
    return normalized < 0.0f ? -magnitude * MAX_VEL : magnitude * MAX_VEL;
}

static bool joystick_centered(void)
{
    float centered = ((float)joystick_raw - JOYSTICK_CENTER) /
                     JOYSTICK_CENTER;
    return fabsf(centered) <= JOYSTICK_DEADBAND;
}

static void update_joystick(uint32_t now)
{
    uint16_t sample;
    uint32_t sequence;

    if (!joystick_ready())
    {
        set_fault(FAULT_JOYSTICK);
        return;
    }
    if (joystick_read_latest(&sample, &sequence) &&
        sequence != joystick_sequence)
    {
        joystick_raw = sample;
        joystick_sequence = sequence;
        last_joystick_tick = now;
    }
    if (!joystick_start_sample())
    {
        set_fault(FAULT_JOYSTICK);
        return;
    }
    if ((joystick_sequence == 0U && now >= JOYSTICK_TIMEOUT_MS) ||
        (joystick_sequence != 0U &&
         now - last_joystick_tick >= JOYSTICK_TIMEOUT_MS))
    {
        set_fault(FAULT_JOYSTICK);
    }
}

static float position_safe_target(float target)
{
    float candidate_velocity = command_velocity_mm_s;
    float candidate_acceleration = command_acceleration_mm_s2;
    motion_profile_step(target, &candidate_velocity, &candidate_acceleration);

    if (candidate_velocity > 0.0f)
    {
        float remaining = MAX_POS - POSITION_MARGIN - position_mm();
        float next_motion = 0.5f * (command_velocity_mm_s +
                                    candidate_velocity) * CONTROL_DT_S;
        if (next_motion + motion_stopping_distance(candidate_velocity,
                                                    candidate_acceleration) >= remaining)
        {
            return 0.0f;
        }
    }
    else if (candidate_velocity < 0.0f)
    {
        float remaining = position_mm() - (MIN_POS + POSITION_MARGIN);
        float next_motion = -0.5f * (command_velocity_mm_s +
                                     candidate_velocity) * CONTROL_DT_S;
        if (next_motion + motion_stopping_distance(-candidate_velocity,
                                                    -candidate_acceleration) >= remaining)
        {
            return 0.0f;
        }
    }
    return target;
}

static void reset_motion_after_limit(void)
{
    command_velocity_mm_s = 0.0f;
    command_acceleration_mm_s2 = 0.0f;
    direction_wait_cycles = 0U;
    stepper_clear_emergency();
    stepper_enable(true);
}

static void begin_calibration(uint32_t now)
{
    state = STATE_CALIBRATING;
    calibration_complete_pending = false;
    operator_armed = false;
    clear_count = 0U;
    calibration_start_steps = stepper_position_steps();
    calibration_phase_steps = calibration_start_steps;
    command_velocity_mm_s = 0.0f;
    command_acceleration_mm_s2 = 0.0f;
    last_host_tick = now;
    stepper_clear_emergency();
    stepper_enable(true);

    if (board_limit_active())
    {
        calibration_phase = CAL_BACK_OFF;
        requested_velocity_mm_s = -CAL_VEL;
    }
    else
    {
        calibration_phase = CAL_SEEK_MIN;
        requested_velocity_mm_s = CAL_VEL;
    }
}

static void handle_limit_event(void)
{
    if (state == STATE_DEACTIVATED)
    {
        return;
    }
    if (state == STATE_CALIBRATING &&
        (calibration_phase == CAL_SEEK_MIN ||
         calibration_phase == CAL_BACK_OFF))
    {
        reset_motion_after_limit();
        if (calibration_phase == CAL_SEEK_MIN)
        {
            calibration_phase = CAL_BACK_OFF;
            calibration_phase_steps = stepper_position_steps();
        }
        clear_count = 0U;
        requested_velocity_mm_s = -CAL_VEL;
        return;
    }
    if (state == STATE_CALIBRATING)
    {
        set_fault(FAULT_CALIBRATION);
        return;
    }
    set_fault(FAULT_HARD_LIMIT);
}

static void update_calibration(void)
{
    if (state != STATE_CALIBRATING)
    {
        return;
    }

    if (calibration_phase == CAL_SEEK_MIN)
    {
        if (board_limit_active())
        {
            handle_limit_event();
        }
        else if (travel_from(calibration_start_steps) >= CAL_SEEK_LIMIT_MM)
        {
            set_fault(FAULT_CALIBRATION);
        }
        return;
    }

    if (calibration_phase == CAL_BACK_OFF)
    {
        if (travel_from(calibration_phase_steps) >= CAL_BACKOFF_LIMIT_MM)
        {
            set_fault(FAULT_CALIBRATION);
            return;
        }
        if (board_limit_active())
        {
            clear_count = 0U;
            return;
        }
        if (clear_count == 0U)
        {
            clear_edge_steps = stepper_position_steps();
        }
        if (++clear_count >= LIMIT_CLEAR_CYCLES)
        {
            origin_steps = clear_edge_steps;
            calibration_phase = CAL_BRAKE;
            requested_velocity_mm_s = 0.0f;
        }
        return;
    }

    if (profile_stopped())
    {
        state = STATE_ACTIVE;
        calibration_complete_pending = true;
        requested_velocity_mm_s = 0.0f;
    }
}

static bool fault_can_clear(void)
{
    if (fault == FAULT_HARD_LIMIT && board_limit_active())
    {
        return false;
    }
    if (fault == FAULT_JOYSTICK &&
        (!joystick_ready() ||
         timer_ticks - last_joystick_tick >= JOYSTICK_TIMEOUT_MS))
    {
        return false;
    }
    if (fault == FAULT_SYSTEM && !stepper_ready())
    {
        return false;
    }
    return true;
}

static void handle_request(teleop_request_t request, uint32_t now,
                           teleop_response_t *response)
{
    if (fault != FAULT_NONE)
    {
        if (request == TELEOP_REQUEST_DISARM && fault_can_clear())
        {
            fault = FAULT_NONE;
            state = STATE_DEACTIVATED;
            stop_fault = FAULT_NONE;
            pending_response = TELEOP_RESPONSE_NONE;
            dis_reply_pending = false;
            stepper_clear_emergency();
            stepper_stop();
            stepper_enable(false);
            response->type = TELEOP_RESPONSE_DISARMED;
        }
        else
        {
            response->type = fault_response(fault);
        }
        return;
    }

    switch (state)
    {
        case STATE_DEACTIVATED:
            if (request == TELEOP_REQUEST_DISARM)
            {
                response->type = TELEOP_RESPONSE_DISARMED;
            }
            else if (request == TELEOP_REQUEST_CALIBRATE)
            {
                begin_calibration(now);
                response->type = TELEOP_RESPONSE_CALIBRATING;
            }
            else
            {
                response->type = TELEOP_RESPONSE_ERR_DIS;
            }
            break;

        case STATE_CALIBRATING:
            if (request == TELEOP_REQUEST_CALIBRATE)
            {
                last_host_tick = now;
                response->type = TELEOP_RESPONSE_CALIBRATING;
            }
            else if (request == TELEOP_REQUEST_DISARM)
            {
                begin_stop(FAULT_NONE, true);
            }
            else
            {
                begin_stop(FAULT_NONE, false);
                response->type = TELEOP_RESPONSE_ERR_CAL;
            }
            break;

        case STATE_ACTIVE:
            if (calibration_complete_pending &&
                request == TELEOP_REQUEST_CALIBRATE)
            {
                calibration_complete_pending = false;
                last_host_tick = now;
                response->type = TELEOP_RESPONSE_ACK;
            }
            else if (request == TELEOP_REQUEST_DISARM)
            {
                begin_stop(FAULT_NONE, true);
            }
            else if (request == TELEOP_REQUEST_STATE)
            {
                calibration_complete_pending = false;
                last_host_tick = now;
                if (!operator_armed && joystick_centered())
                {
                    operator_armed = true;
                }
                response->type = TELEOP_RESPONSE_ACK;
            }
            else if (request == TELEOP_REQUEST_CALIBRATE)
            {
                response->type = TELEOP_RESPONSE_ERR_CAL;
            }
            else
            {
                response->type = TELEOP_RESPONSE_ERR_CMD;
            }
            break;

        case STATE_STOPPING:
            if (request != TELEOP_REQUEST_DISARM)
            {
                response->type = TELEOP_RESPONSE_ERR_DIS;
            }
            break;

        case STATE_FAULT:
            response->type = fault_response(fault);
            break;
    }
}

static float reversal_safe_target(float target)
{
    if ((command_velocity_mm_s > 0.0f && target < 0.0f) ||
        (command_velocity_mm_s < 0.0f && target > 0.0f))
    {
        return 0.0f;
    }
    if (!profile_stopped())
    {
        return target;
    }

    bool target_positive = target > 0.0f;
    if (target != 0.0f && target_positive != direction_positive)
    {
        stepper_stop();
        if (!stepper_set_direction(target_positive))
        {
            set_fault(FAULT_SYSTEM);
            return 0.0f;
        }
        direction_positive = target_positive;
        direction_wait_cycles = 1U;
        return 0.0f;
    }
    if (direction_wait_cycles > 0U)
    {
        direction_wait_cycles--;
        return 0.0f;
    }
    return target;
}

static void command_stepper(void)
{
    if (command_velocity_mm_s == 0.0f)
    {
        stepper_stop();
        return;
    }
    bool positive = command_velocity_mm_s > 0.0f;
    if (positive != direction_positive ||
        !stepper_set_rate(fabsf(command_velocity_mm_s) *
                          PULSES_PER_VIRTUAL_MM))
    {
        set_fault(FAULT_SYSTEM);
    }
}

static void run_control_cycle(void)
{
    uint32_t now = timer_ticks;
    teleop_request_t request = TELEOP_REQUEST_NONE;
    teleop_response_t response = {0};
    bool has_request = teleop_protocol_read_request(&request);

    if (uart_take_rx_overflow() || overrun_pending)
    {
        overrun_pending = false;
        set_fault(FAULT_SYSTEM);
    }
    update_joystick(now);
    if (!stepper_ready())
    {
        set_fault(FAULT_SYSTEM);
    }
    if (board_take_limit_event())
    {
        handle_limit_event();
    }
    if ((state == STATE_ACTIVE || state == STATE_STOPPING) &&
        board_limit_active())
    {
        set_fault(FAULT_HARD_LIMIT);
    }

    if (has_request)
    {
        handle_request(request, now, &response);
    }

    if ((state == STATE_CALIBRATING || state == STATE_ACTIVE) &&
        now - last_host_tick >= HOST_TIMEOUT_MS)
    {
        begin_stop(FAULT_COMMUNICATION, false);
    }

    update_calibration();

    if (state == STATE_ACTIVE && operator_armed)
    {
        requested_velocity_mm_s = joystick_velocity();
    }
    else if (state == STATE_ACTIVE)
    {
        requested_velocity_mm_s = 0.0f;
    }

    if (state == STATE_CALIBRATING || state == STATE_ACTIVE ||
        state == STATE_STOPPING)
    {
        float target = requested_velocity_mm_s;
        if (state == STATE_ACTIVE)
        {
            target = position_safe_target(target);
        }
        target = reversal_safe_target(target);
        motion_profile_step(target, &command_velocity_mm_s,
                            &command_acceleration_mm_s2);
        command_stepper();
    }

    update_calibration();

    if (state == STATE_STOPPING && profile_stopped())
    {
        stepper_stop();
        stepper_enable(false);
        if (stop_fault == FAULT_NONE)
        {
            state = STATE_DEACTIVATED;
            if (dis_reply_pending)
            {
                pending_response = TELEOP_RESPONSE_DISARMED;
                dis_reply_pending = false;
            }
        }
        else
        {
            teleop_fault_t completed_fault = stop_fault;
            stop_fault = FAULT_NONE;
            set_fault(completed_fault);
        }
    }

    bool deferred = response.type == TELEOP_RESPONSE_NONE &&
                    pending_response != TELEOP_RESPONSE_NONE;
    if (deferred)
    {
        response.type = pending_response;
    }
    if (response.type != TELEOP_RESPONSE_NONE)
    {
        if (fault != FAULT_NONE &&
            response.type != TELEOP_RESPONSE_ERR_CAL)
        {
            response.type = fault_response(fault);
        }
        response.position_mm = position_mm();
        response.velocity_mm_s = command_velocity_mm_s;
        if (!teleop_protocol_write_response(&response))
        {
            set_fault(FAULT_SYSTEM);
        }
        else if (deferred)
        {
            pending_response = TELEOP_RESPONSE_NONE;
        }
    }
}

void control_init(void)
{
    state = STATE_DEACTIVATED;
    stepper_stop();
    stepper_enable(false);
    if (!stepper_ready())
    {
        set_fault(FAULT_SYSTEM);
    }
    if (!joystick_ready() || !joystick_start_sample())
    {
        set_fault(FAULT_JOYSTICK);
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

    run_control_cycle();
    cycle_active = false;
    return true;
}

void control_timer_tick_isr(void)
{
    timer_ticks++;
    if (cycle_pending || cycle_active)
    {
        overrun_pending = true;
    }
    cycle_pending = true;
}
