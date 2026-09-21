#ifndef MOTION_H
#define MOTION_H

#define MAX_JRK 50.0f
#define MAX_ACC 50.0f
#define MAX_VEL 32.0f
#define CONTROL_DT_S 0.001f

void motion_profile_step(float target, float *velocity, float *acceleration);
float motion_stopping_distance(float velocity, float acceleration);

#endif
