#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdbool.h>
#include <stdint.h>

bool joystick_init(void);
bool joystick_start_sample(void);
bool joystick_ready(void);
bool joystick_read_latest(uint16_t *sample, uint32_t *sequence);

#endif
