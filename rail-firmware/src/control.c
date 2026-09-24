#include "control.h"

#include <math.h>

#include "ak60.h"
#include "board.h"
#include "protocol.h"
#include "uart.h"

#define HOST_TIMEOUT_MS 50U
#define CALIBRATION_TIMEOUT_MS 30000U
#define FEEDBACK_TIMEOUT_MS 10U
#define OPTICAL_DEBOUNCE_CYCLES 5U
#define TWO_PI 6.28318530718f

typedef enum
{
    STATE_DEACTIVATED = 0,
    STATE_CALIBRATING,
    STATE_ACTIVE,
    STATE_STOPPING,
    STATE_FAULT
} drive_state_t;

typedef enum
{
    CAL_SEEK_MIN = 0,
    CAL_BACK_OFF,
    CAL_BRAKE
} calibration_phase_t;

typedef enum
{
    FAULT_NONE = 0,
    FAULT_ESTOP,
    FAULT_HARD_LIMIT,
    FAULT_SYSTEM,
    FAULT_MOTOR,
    FAULT_CAN,
    FAULT_VELOCITY_LIMIT,
    FAULT_COMMUNICATION
} drive_fault_t;

static volatile bool cycle_pending;
static volatile bool cycle_active;
static volatile bool overrun_pending;
static volatile uint32_t timer_ticks;

static drive_state_t state;
static calibration_phase_t calibration_phase;
static drive_fault_t fault;
static drive_fault_t stop_fault;
static float requested_velocity_mm_s;
static float command_velocity_mm_s;
static float command_acceleration_mm_s2;
static float position_mm;
static float velocity_mm_s;
static float velocity_gain_kd;
static bool position_calibrated;
static bool calibration_complete_pending;
static bool calibration_error_pending;
static bool feedback_seen;
static bool last_motor_tx_ok;
static uint32_t feedback_sequence;
static uint32_t last_feedback_tick;
static uint32_t last_host_tick;
static uint32_t calibration_start_tick;
static int16_t previous_position_counts;
static ak60_state_t motor;

static bool optical_candidate;
static bool optical_stable;
static float optical_clear_position_mm;
static uint8_t optical_count;

static protocol_response_type_t fault_response(drive_fault_t reason)
{
    switch (reason)
    {
        case FAULT_ESTOP: return PROTOCOL_RESPONSE_ERR_EST;
        case FAULT_HARD_LIMIT: return PROTOCOL_RESPONSE_ERR_HRD;
        case FAULT_SYSTEM: return PROTOCOL_RESPONSE_ERR_SYS;
        case FAULT_MOTOR: return PROTOCOL_RESPONSE_ERR_MOT;
        case FAULT_CAN: return PROTOCOL_RESPONSE_ERR_CAN;
        case FAULT_VELOCITY_LIMIT: return PROTOCOL_RESPONSE_ERR_LIM;
        case FAULT_COMMUNICATION: return PROTOCOL_RESPONSE_ERR_COM;
        default: return PROTOCOL_RESPONSE_NONE;
    }
}

static void set_fault(drive_fault_t reason)
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
    position_calibrated = false;
    calibration_complete_pending = false;
    calibration_error_pending = false;
}

static void begin_stop(drive_fault_t reason)
{
    state = STATE_STOPPING;
    stop_fault = reason;
    requested_velocity_mm_s = 0.0f;
    position_calibrated = false;
    calibration_complete_pending = false;
    calibration_error_pending = false;
}

static void update_optical_switch(void)
{
    bool sample = board_optical_min_active();
    if (sample == optical_candidate)
    {
        if (optical_count < OPTICAL_DEBOUNCE_CYCLES)
        {
            optical_count++;
        }
    }
    else
    {
        optical_candidate = sample;
        optical_count = 1U;
        if (!sample)
        {
            /* Keep the edge position if this clear interval is confirmed. */
            optical_clear_position_mm = position_mm;
        }
    }
    if (optical_count >= OPTICAL_DEBOUNCE_CYCLES)
    {
        optical_stable = optical_candidate;
    }
}

static float counts_to_mm(int16_t counts)
{
    return MOTOR_DIRECTION * (float)counts * 0.1f * PITCH / 360.0f;
}

static float erpm_to_mm_s(int16_t erpm_counts)
{
    float erpm = (float)erpm_counts * 10.0f;
    return MOTOR_DIRECTION * erpm * PITCH /
           (AK60_POLE_PAIRS * AK60_REDUCTION * 60.0f);
}

static float mm_s_to_rad_s(float linear_velocity)
{
    return MOTOR_DIRECTION * linear_velocity * TWO_PI / PITCH;
}

static void consume_feedback(uint32_t now)
{
    can_frame_t frame;
    uint32_t sequence;
    ak60_state_t sample;

    if (!can_read_latest(&frame, &sequence) || sequence == feedback_sequence ||
        !ak60_parse_feedback(&frame, &sample))
    {
        return;
    }

    feedback_sequence = sequence;
    if (sample.error != 0U)
    {
        motor = sample;
        last_feedback_tick = now;
        feedback_seen = true;
        set_fault(FAULT_MOTOR);
        return;
    }

    if (feedback_seen)
    {
        int16_t delta = (int16_t)((uint16_t)sample.position_counts -
                                  (uint16_t)previous_position_counts);
        position_mm += counts_to_mm(delta);
        previous_position_counts = sample.position_counts;
    }
    else
    {
        previous_position_counts = sample.position_counts;
    }

    motor = sample;
    velocity_mm_s = erpm_to_mm_s(sample.velocity_erpm);
    feedback_seen = true;
    last_feedback_tick = now;
}

static bool feedback_fresh(uint32_t now)
{
    return feedback_seen && now - last_feedback_tick < FEEDBACK_TIMEOUT_MS;
}

static bool calibration_preflight_ok(uint32_t now,
                                     protocol_response_type_t *error)
{
    if (!board_estop_ok())
    {
        *error = PROTOCOL_RESPONSE_ERR_EST;
        return false;
    }
    if (!board_hard_limits_ok())
    {
        *error = PROTOCOL_RESPONSE_ERR_HRD;
        return false;
    }
    if (!last_motor_tx_ok || !feedback_fresh(now))
    {
        *error = PROTOCOL_RESPONSE_ERR_CAN;
        return false;
    }
    if (motor.error != 0U)
    {
        *error = PROTOCOL_RESPONSE_ERR_MOT;
        return false;
    }
    return true;
}

static float position_safe_target(float target)
{
    float candidate_velocity = command_velocity_mm_s;
    float candidate_acceleration = command_acceleration_mm_s2;
    motion_profile_step(target, &candidate_velocity, &candidate_acceleration);

    if (candidate_velocity > 0.0f)
    {
        float remaining = MAX_POS - POSITION_MARGIN - position_mm;
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
        float remaining = position_mm - (MIN_POS + POSITION_MARGIN);
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

static bool command_is_outward(float command)
{
    return (command < 0.0f && position_mm <= MIN_POS + POSITION_MARGIN) ||
           (command > 0.0f && position_mm >= MAX_POS - POSITION_MARGIN);
}

static bool profile_stopped(void)
{
    return command_velocity_mm_s == 0.0f &&
           command_acceleration_mm_s2 == 0.0f;
}

static void start_calibration(uint32_t now)
{
    state = STATE_CALIBRATING;
    position_calibrated = false;
    calibration_complete_pending = false;
    calibration_error_pending = false;
    calibration_start_tick = now;
    if (optical_stable)
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

static bool fault_can_clear(uint32_t now)
{
    if (!board_estop_ok() || !board_hard_limits_ok())
    {
        return false;
    }
    if (fault == FAULT_CAN || fault == FAULT_MOTOR)
    {
        return last_motor_tx_ok && feedback_fresh(now) && motor.error == 0U;
    }
    return true;
}

static void handle_request(const protocol_request_t *request, uint32_t now,
                           protocol_response_t *response)
{
    if (request->type == PROTOCOL_REQUEST_NONE)
    {
        return;
    }

    if (request->type == PROTOCOL_REQUEST_SET_KD)
    {
        velocity_gain_kd = request->gain_kd;
        response->type = PROTOCOL_RESPONSE_KD;
        response->gain_kd = velocity_gain_kd;
        return;
    }
    if (request->type == PROTOCOL_REQUEST_INVALID_KD)
    {
        response->type = PROTOCOL_RESPONSE_ERR_KD;
        return;
    }

    if (fault != FAULT_NONE)
    {
        if (request->type == PROTOCOL_REQUEST_DISARM && fault_can_clear(now))
        {
            fault = FAULT_NONE;
            state = STATE_DEACTIVATED;
            stop_fault = FAULT_NONE;
            command_velocity_mm_s = 0.0f;
            command_acceleration_mm_s2 = 0.0f;
            response->type = PROTOCOL_RESPONSE_DISARMED;
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
            if (request->type == PROTOCOL_REQUEST_DISARM)
            {
                response->type = PROTOCOL_RESPONSE_DISARMED;
            }
            else if (request->type == PROTOCOL_REQUEST_CALIBRATE)
            {
                if (calibration_preflight_ok(now, &response->type))
                {
                    start_calibration(now);
                    response->type = PROTOCOL_RESPONSE_CALIBRATING;
                }
            }
            else
            {
                response->type = PROTOCOL_RESPONSE_ERR_DIS;
            }
            break;

        case STATE_CALIBRATING:
            if (request->type == PROTOCOL_REQUEST_CALIBRATE)
            {
                response->type = PROTOCOL_RESPONSE_CALIBRATING;
            }
            else if (request->type == PROTOCOL_REQUEST_DISARM)
            {
                begin_stop(FAULT_NONE);
                response->type = PROTOCOL_RESPONSE_DISARMED;
            }
            else
            {
                begin_stop(FAULT_NONE);
                response->type = PROTOCOL_RESPONSE_ERR_CAL;
            }
            break;

        case STATE_ACTIVE:
            if (request->type == PROTOCOL_REQUEST_DISARM)
            {
                begin_stop(FAULT_NONE);
                response->type = PROTOCOL_RESPONSE_DISARMED;
            }
            else if (request->type == PROTOCOL_REQUEST_CALIBRATE)
            {
                response->type = PROTOCOL_RESPONSE_ERR_CAL;
            }
            else if (request->type == PROTOCOL_REQUEST_SETPOINT)
            {
                calibration_complete_pending = false;
                if (fabsf(request->value_mm_s) > MAX_VEL)
                {
                    set_fault(FAULT_VELOCITY_LIMIT);
                    response->type = PROTOCOL_RESPONSE_ERR_LIM;
                }
                else
                {
                    last_host_tick = now;
                    if (command_is_outward(request->value_mm_s))
                    {
                        requested_velocity_mm_s = 0.0f;
                        response->type = PROTOCOL_RESPONSE_ERR_POS;
                    }
                    else
                    {
                        requested_velocity_mm_s = request->value_mm_s;
                        response->type = PROTOCOL_RESPONSE_ACK;
                    }
                }
            }
            else
            {
                response->type = PROTOCOL_RESPONSE_ERR_CMD;
            }
            break;

        case STATE_STOPPING:
            response->type = request->type == PROTOCOL_REQUEST_DISARM ?
                             PROTOCOL_RESPONSE_DISARMED :
                             PROTOCOL_RESPONSE_ERR_DIS;
            break;

        case STATE_FAULT:
            response->type = fault_response(fault);
            break;
    }
}

static void update_calibration(void)
{
    if (state != STATE_CALIBRATING)
    {
        return;
    }

    if (calibration_phase == CAL_SEEK_MIN && optical_stable)
    {
        calibration_phase = CAL_BACK_OFF;
        requested_velocity_mm_s = -CAL_VEL;
    }
    else if (calibration_phase == CAL_BACK_OFF && !optical_stable)
    {
        position_mm -= optical_clear_position_mm;
        calibration_phase = CAL_BRAKE;
        requested_velocity_mm_s = 0.0f;
    }
    else if (calibration_phase == CAL_BRAKE && profile_stopped())
    {
        state = STATE_ACTIVE;
        position_calibrated = true;
        calibration_complete_pending = true;
        requested_velocity_mm_s = 0.0f;
    }
}

static bool send_motor_command(void)
{
    if (state == STATE_DEACTIVATED || state == STATE_FAULT)
    {
        return ak60_send_zero_torque();
    }
    return ak60_send_velocity_rad_s(mm_s_to_rad_s(command_velocity_mm_s),
                                    velocity_gain_kd);
}

static void run_control_cycle(void)
{
    uint32_t now = timer_ticks;
    protocol_request_t request = {PROTOCOL_REQUEST_NONE, 0.0f, 0.0f};
    protocol_response_t response = {0};
    bool has_request = protocol_read_request(&request);
    bool was_calibrating = state == STATE_CALIBRATING;

    if (uart_take_rx_overflow() || overrun_pending)
    {
        overrun_pending = false;
        set_fault(FAULT_SYSTEM);
    }
    if (!board_estop_ok())
    {
        set_fault(FAULT_ESTOP);
    }
    if (!board_hard_limits_ok())
    {
        set_fault(FAULT_HARD_LIMIT);
    }
    consume_feedback(now);
    update_optical_switch();

    if ((state == STATE_CALIBRATING || state == STATE_ACTIVE ||
         state == STATE_STOPPING) && !feedback_fresh(now))
    {
        set_fault(FAULT_CAN);
    }

    if (has_request)
    {
        handle_request(&request, now, &response);
    }

    if (state == STATE_ACTIVE &&
        now - last_host_tick >= HOST_TIMEOUT_MS)
    {
        begin_stop(FAULT_COMMUNICATION);
    }

    if (state == STATE_CALIBRATING &&
        now - calibration_start_tick >= CALIBRATION_TIMEOUT_MS)
    {
        begin_stop(FAULT_NONE);
        calibration_error_pending = true;
    }

    update_calibration();

    if (state == STATE_CALIBRATING || state == STATE_ACTIVE ||
        state == STATE_STOPPING)
    {
        float target = state == STATE_ACTIVE ?
                       position_safe_target(requested_velocity_mm_s) :
                       requested_velocity_mm_s;
        motion_profile_step(target, &command_velocity_mm_s,
                            &command_acceleration_mm_s2);
    }

    if (state == STATE_STOPPING && profile_stopped())
    {
        if (stop_fault == FAULT_NONE)
        {
            state = STATE_DEACTIVATED;
        }
        else
        {
            drive_fault_t completed_fault = stop_fault;
            stop_fault = FAULT_NONE;
            set_fault(completed_fault);
        }
    }

    update_calibration();

    last_motor_tx_ok = send_motor_command();
    if (!last_motor_tx_ok)
    {
        set_fault(FAULT_CAN);
    }

    if (calibration_complete_pending &&
        response.type == PROTOCOL_RESPONSE_NONE)
    {
        calibration_complete_pending = false;
        last_host_tick = now;
        response.type = PROTOCOL_RESPONSE_ACK;
    }
    else if (!has_request && was_calibrating && fault != FAULT_NONE)
    {
        response.type = fault_response(fault);
    }
    else if (calibration_error_pending &&
             response.type == PROTOCOL_RESPONSE_NONE)
    {
        calibration_error_pending = false;
        response.type = PROTOCOL_RESPONSE_ERR_CAL;
    }

    if (has_request || response.type != PROTOCOL_RESPONSE_NONE)
    {
        if (fault != FAULT_NONE &&
            response.type != PROTOCOL_RESPONSE_ERR_CAL &&
            response.type != PROTOCOL_RESPONSE_ERR_LIM &&
            response.type != PROTOCOL_RESPONSE_ERR_CMD &&
            response.type != PROTOCOL_RESPONSE_ERR_KD &&
            response.type != PROTOCOL_RESPONSE_KD)
        {
            response.type = fault_response(fault);
        }
        response.position_mm = position_mm;
        response.velocity_mm_s = velocity_mm_s;
        response.acceleration_mm_s2 = command_acceleration_mm_s2;
        if (!protocol_write_response(&response))
        {
            set_fault(FAULT_SYSTEM);
        }
    }
}

void control_init(void)
{
    state = STATE_DEACTIVATED;
    velocity_gain_kd = AK60_DEFAULT_KD;
    optical_candidate = board_optical_min_active();
    optical_stable = optical_candidate;
    optical_count = OPTICAL_DEBOUNCE_CYCLES;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
