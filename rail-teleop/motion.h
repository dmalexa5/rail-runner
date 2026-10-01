#ifndef TELEOP_MOTION_H
#define TELEOP_MOTION_H
#ifndef SCALE
#define SCALE 0.1f
#endif
#define MAX_JRK (50.0f * SCALE)
#define MAX_ACC (50.0f * SCALE)
#define MAX_VEL (32.0f * SCALE)
#define MAX_POS (500.0f * SCALE)
#define POSITION_MARGIN (2.0f * SCALE)
#define PITCH (4.0f * SCALE)
#define CONTROL_DT_S 0.001f
#define PULSES_PER_MM (1600.0f / PITCH)
#define MAX_PULSES 2000000L
void motion_profile_step(float target, float *velocity, float *acceleration);
float motion_stopping_distance(float velocity, float acceleration);
float motion_safe_target(float target, float position, float velocity, float acceleration);
float joystick_velocity(unsigned sample);
#endif
