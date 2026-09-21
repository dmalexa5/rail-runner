#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_PERIOD_US 1000U

#define MAX_JRK 50.0f
#define MAX_ACC 50.0f
#define MAX_VEL 32.0f
#define MIN_POS 0.0f
#define MAX_POS 500.0f
#define POSITION_MARGIN 2.0f
#define CAL_VEL (-10.0f)
#define PITCH 4.0f
#define MOTOR_DIRECTION 1.0f

/** Initializes the uncalibrated, zero-current drive state. */
void control_init(void);

/** Runs one released 1 kHz drive cycle, or returns false if none is pending. */
bool control_run_pending(void);

/** Timer-ISR entrypoint that releases a cycle and detects overruns. */
void control_timer_tick_isr(void);

#endif
