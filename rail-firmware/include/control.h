#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "motion.h"

#define CONTROL_PERIOD_US 1000U

#define MIN_POS 0.0f
#define MAX_POS 500.0f
#define POSITION_MARGIN 2.0f
#define CAL_VEL (-10.0f)
#ifdef BOARD_DRIVE
#define PITCH 4.0f
#define MOTOR_DIRECTION 1.0f
#endif

/** Initializes the selected board's deactivated control state. */
void control_init(void);

/** Runs one released 1 kHz control cycle, or returns false if none is pending. */
bool control_run_pending(void);

/** Timer-ISR entrypoint that releases a cycle and detects overruns. */
void control_timer_tick_isr(void);

#endif
