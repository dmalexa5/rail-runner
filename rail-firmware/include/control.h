#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "ak60.h"

#define CONTROL_PERIOD_US 1000U
#define CONTROL_ARM_LEASE_MS 100U
#define CONTROL_FEEDBACK_TIMEOUT_MS 10U
#define CONTROL_TORQUE_LIMIT_NM 5.0f

typedef enum
{
    CONTROL_FAULT_NONE = 0,
    CONTROL_FAULT_SAFETY_SWITCH,
    CONTROL_FAULT_ARM_LEASE,
    CONTROL_FAULT_FEEDBACK_TIMEOUT,
    CONTROL_FAULT_OVERRUN,
    CONTROL_FAULT_CAN_TX
} control_fault_t;

typedef struct
{
    bool armed;
    bool safety_switches_ok;
    float torque_command_nm;
    control_fault_t fault;
    uint32_t cycle_count;
    uint32_t overrun_count;
    uint32_t max_execution_cycles;
    uint32_t arm_lease_age_ms;
    uint32_t feedback_age_ms;
    uint32_t feedback_sequence;
    ak60_state_t motor;
} control_status_t;

/** Initializes the disarmed control state and execution-time counter. */
void control_init(void);

/**
 * Arms torque output when both safety switches are healthy.
 *
 * Calling this again while armed refreshes the 100 ms arm lease. An explicit
 * successful arm clears the current fault; diagnostic counters are retained.
 * Arming is rejected while a newly detected overrun awaits processing.
 */
bool control_arm(void);

/** Disarms immediately and resets the requested torque to zero. */
void control_disarm(void);

/**
 * Sets the armed torque command in newton-metres.
 *
 * Returns false while disarmed, non-finite, or outside +/-5 Nm.
 * This command does not refresh the arm lease.
 */
bool control_set_torque(float torque_nm);

/** Copies the current control and motor status for non-real-time reporting. */
void control_get_status(control_status_t *status);

/**
 * Runs one released control iteration and returns true, or returns false when
 * no timer release is pending. The iteration reads safety GPIO, consumes the
 * latest CAN feedback, applies fault policy, and queues exactly one CAN command.
 */
bool control_run_pending(void);

/** Timer-ISR entrypoint. Releases a cycle and detects pending/active overruns. */
void control_timer_tick_isr(void);

#endif
