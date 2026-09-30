#ifndef CONTROL_TIMER_H
#define CONTROL_TIMER_H

#include <stdbool.h>

/**
 * Configures TIM2 to release the control loop every 1 ms.
 *
 * The timer interrupt invokes control_timer_tick_isr(); it never executes the
 * control calculation itself.
 */
bool control_timer_init(void);

/** Starts periodic 1 kHz control releases. */
bool control_timer_start(void);

#endif
