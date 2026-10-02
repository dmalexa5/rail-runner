#include "../motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void check_step(float target, float *v, float *a)
{
    float old_a = *a, old_v = *v;
    motion_profile_step(target, v, a);
    assert(fabsf(*v - old_v) <= MAX_ACC * CONTROL_DT_S + 0.0001f);
    assert(fabsf(*v) <= MAX_VEL + 0.0001f);
    assert(fabsf(*a) <= MAX_ACC + 0.0001f);
    assert(fabsf(*a - old_a) <= MAX_JRK * CONTROL_DT_S + 0.0001f);
}

int main(void)
{
    assert(fabsf(MAX_POS / SCALE - 500.0f) < 0.001f);
    assert(fabsf(MAX_VEL / SCALE - 32.0f) < 0.001f);
    assert(fabsf(MAX_ACC * SCALE - 50.0f) < 0.001f);
    assert(fabsf(MAX_JRK * SCALE - 50.0f) < 0.001f);
    assert(fabsf(TRAVEL_PER_REV_MM - 62.83185f) < 0.001f);
    assert(fabsf(PULSES_PER_MM * TRAVEL_PER_REV_MM - 3200.0f) < 0.001f);
    assert(fabsf(MAX_POS * PULSES_PER_MM - MAX_PULSES) < 1.0f);
    assert(joystick_velocity(0, 504) == -MAX_VEL);
    assert(joystick_velocity(1023, 504) == MAX_VEL);
    assert(joystick_velocity(504, 504) == 0);
    assert(joystick_velocity(479, 504) == 0);
    assert(joystick_velocity(529, 504) == 0);
    assert(joystick_velocity(478, 504) < 0);
    assert(joystick_velocity(530, 504) > 0);
    assert(joystick_velocity(480, 480) == 0);
    assert(joystick_velocity(0, 480) == -MAX_VEL);
    assert(joystick_velocity(1023, 480) == MAX_VEL);
    assert(joystick_velocity(0, 0) == 0);
    assert(joystick_velocity(1023, 1023) == 0);
    float v = 0, a = 0;
    for (int i = 0; i < 12000; ++i)
        check_step(i < 4000 ? MAX_VEL : i < 8000 ? -MAX_VEL : 0, &v, &a);
    assert(fabsf(v) < 0.001f * SCALE);
    assert(fabsf(a) < 0.01f * SCALE);
    for (int direction = -1; direction <= 1; direction += 2) {
        float position = direction > 0 ? 0 : MAX_POS;
        v = a = 0;
        for (int i = 0; i < 30000; ++i) {
            float target = motion_safe_target(direction * MAX_VEL, position, v, a);
            float old_v = v;
            check_step(target, &v, &a);
            position += (old_v + v) * 0.5f * CONTROL_DT_S;
            assert(position >= -0.001f * SCALE);
            assert(position <= MAX_POS + 0.001f * SCALE);
        }
        assert(fabsf(v) < 0.001f * MAX_VEL);
        assert(direction > 0 ? position <= MAX_POS - POSITION_MARGIN + 0.005f * SCALE :
                              position >= POSITION_MARGIN - 0.005f * SCALE);
        assert(direction > 0 ? position >= MAX_POS - 2 * POSITION_MARGIN :
                              position <= 2 * POSITION_MARGIN);
        float inward = motion_safe_target(-direction * MAX_VEL, position, v, a);
        assert(inward * direction < 0);
    }
    puts("motion and joystick checks passed");
}
