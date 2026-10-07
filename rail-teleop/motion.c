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
    /* Settle rounding-sized oscillations with a single bounded acceleration change. */
    if (fabsf(target - *velocity) <= 2.0f * jerk_step * CONTROL_DT_S &&
        fabsf(*acceleration) <= jerk_step + 0.000001f * SCALE)
    {
        *velocity = target;
        *acceleration = 0.0f;
        return;
    }
    float neutral_velocity = *velocity +
                             *acceleration * fabsf(*acceleration) /
                             (2.0f * MAX_JRK);
    float neutral_error = target - neutral_velocity;
    float desired_acceleration = 0.0f;

    if (neutral_error > (0.00001f * SCALE))
    {
        desired_acceleration = MAX_ACC;
    }
    else if (neutral_error < -(0.00001f * SCALE))
    {
        desired_acceleration = -MAX_ACC;
    }

    float old_acceleration = *acceleration;
    *acceleration = approach(*acceleration, desired_acceleration, jerk_step);
    *acceleration = clampf(*acceleration, -MAX_ACC, MAX_ACC);
    *velocity += 0.5f * (old_acceleration + *acceleration) * CONTROL_DT_S;

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

float motion_safe_target(float target, float position, float velocity, float acceleration)
{
    if ((target < 0.0f && position <= POSITION_MARGIN) ||
        (target > 0.0f && position >= MAX_POS - POSITION_MARGIN))
        target = 0.0f;
    float candidate = velocity, next_acceleration = acceleration;
    motion_profile_step(target, &candidate, &next_acceleration);
    if (candidate > 0.0f &&
        0.5f * (velocity + candidate) * CONTROL_DT_S +
        motion_stopping_distance(candidate, next_acceleration) >=
        MAX_POS - POSITION_MARGIN - position)
        return 0.0f;
    if (candidate < 0.0f &&
        -0.5f * (velocity + candidate) * CONTROL_DT_S +
        motion_stopping_distance(-candidate, -next_acceleration) >=
        position - POSITION_MARGIN)
        return 0.0f;
    return target;
}

float joystick_velocity(unsigned sample, unsigned center)
{
    int offset = (int)sample - (int)center;
    if (offset > JOYSTICK_DEADBAND)
        return MAX_VEL * ((float)(offset - JOYSTICK_DEADBAND) /
            (1023.0f - center - JOYSTICK_DEADBAND));
    if (offset < -JOYSTICK_DEADBAND)
        return MAX_VEL * ((float)(offset + JOYSTICK_DEADBAND) /
            (center - JOYSTICK_DEADBAND));
    return 0.0f;
}
