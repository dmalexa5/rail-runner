#ifndef TELEOP_MOTION_H
#define TELEOP_MOTION_H
#include <stdint.h>
#ifndef SCALE
#define SCALE 0.35f
#endif
#define MAX_JRK 150.0f
#define MAX_ACC 100.0f
#define MAX_VEL (32.0f * SCALE)
#define MAX_POS (420.0f * SCALE)
#define POSITION_MARGIN (2.0f * SCALE)
#define TRAVEL_PER_REV_MM (20.0f * 3.14159265358979323846f)
#define CONTROL_DT_S 0.001f
#define PULSES_PER_MM (6400.0f / TRAVEL_PER_REV_MM)
#define MAX_PULSES ((int32_t)(MAX_POS * PULSES_PER_MM))
#define JOYSTICK_DEADBAND 5
void motion_profile_step(float target, float *velocity, float *acceleration);
float motion_stopping_distance(float velocity, float acceleration);
float motion_safe_target(float target, float position, float velocity, float acceleration);
float joystick_velocity(unsigned sample, unsigned center);
#endif
