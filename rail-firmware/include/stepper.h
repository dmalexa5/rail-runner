#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include <stdint.h>

bool stepper_init(void);
bool stepper_ready(void);
void stepper_enable(bool enable);
bool stepper_set_direction(bool positive);
bool stepper_set_rate(float pulses_per_second);
void stepper_stop(void);
void stepper_emergency_stop_isr(void);
void stepper_clear_emergency(void);
int32_t stepper_position_steps(void);

#endif
