#include "motion.h"

#include <math.h>

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float approach(float value, float target, float maximum_step)
{
    if (value < target - maximum_step)
    {
        return value + maximum_step;
    }
    if (value > target + maximum_step)
    {
        return value - maximum_step;
    }
    return target;
}

void motion_profile_step(float target, float *velocity, float *acceleration)
{
    const float jerk_step = MAX_JRK * CONTROL_DT_S;
    float neutral_velocity = *velocity +
                             *acceleration * fabsf(*acceleration) /
                             (2.0f * MAX_JRK);
    float neutral_error = target - neutral_velocity;
    float desired_acceleration = 0.0f;

    if (neutral_error > 0.00001f)
    {
        desired_acceleration = MAX_ACC;
    }
    else if (neutral_error < -0.00001f)
    {
        desired_acceleration = -MAX_ACC;
    }

    float old_acceleration = *acceleration;
    *acceleration = approach(*acceleration, desired_acceleration, jerk_step);
    *acceleration = clampf(*acceleration, -MAX_ACC, MAX_ACC);
    *velocity += 0.5f * (old_acceleration + *acceleration) * CONTROL_DT_S;

    if (fabsf(target - *velocity) < 0.00001f &&
        fabsf(*acceleration) <= jerk_step)
    {
        *velocity = target;
        *acceleration = approach(*acceleration, 0.0f, jerk_step);
    }
    *velocity = clampf(*velocity, -MAX_VEL, MAX_VEL);
}

static void integrate_segment(float *position, float *velocity,
                              float *acceleration, float jerk, float duration)
{
    *position += *velocity * duration +
                 0.5f * *acceleration * duration * duration +
                 jerk * duration * duration * duration / 6.0f;
    *velocity += *acceleration * duration +
                 0.5f * jerk * duration * duration;
    *acceleration += jerk * duration;
}

float motion_stopping_distance(float velocity, float acceleration)
{
    float position = 0.0f;
    if (velocity <= 0.0f)
    {
        return 0.0f;
    }

    if (acceleration < 0.0f)
    {
        float discriminant = acceleration * acceleration -
                             2.0f * MAX_JRK * velocity;
        if (discriminant >= 0.0f)
        {
            float duration = (-acceleration - sqrtf(discriminant)) / MAX_JRK;
            integrate_segment(&position, &velocity, &acceleration,
                              MAX_JRK, duration);
            return position > 0.0f ? position : 0.0f;
        }
    }

    float peak = sqrtf(MAX_JRK * velocity +
                       0.5f * acceleration * acceleration);
    if (peak <= MAX_ACC)
    {
        float first = (acceleration + peak) / MAX_JRK;
        integrate_segment(&position, &velocity, &acceleration,
                          -MAX_JRK, first);
        integrate_segment(&position, &velocity, &acceleration,
                          MAX_JRK, peak / MAX_JRK);
    }
    else
    {
        float first = (acceleration + MAX_ACC) / MAX_JRK;
        integrate_segment(&position, &velocity, &acceleration,
                          -MAX_JRK, first);
        float final_ramp_loss = MAX_ACC * MAX_ACC / (2.0f * MAX_JRK);
        float hold = (velocity - final_ramp_loss) / MAX_ACC;
        if (hold > 0.0f)
        {
            integrate_segment(&position, &velocity, &acceleration,
                              0.0f, hold);
        }
        integrate_segment(&position, &velocity, &acceleration,
                          MAX_JRK, MAX_ACC / MAX_JRK);
    }
    return position > 0.0f ? position : 0.0f;
}
