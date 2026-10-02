#define TELEOP_HOST_TEST
#include "../main.c"
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static void millisecond(void)
{
    for (int i = 0; i < 40; ++i)
        TIMER1_COMPA_vect();
    assert(pending);
    pending = false;
    control_cycle(position_pulses);
    while (tx_tail != tx_head)
        USART_UDRE_vect();
}

static void wait_ms(unsigned count)
{
    while (count--)
        millisecond();
}

static void check_broadcast(unsigned sample, int32_t position, bool motor_enabled,
                            const char *expected)
{
    ADC = sample;
    enabled = motor_enabled;
    tx_head = tx_tail = 0;
    tx_in_line = false;
    for (int i = 0; i < 10; ++i)
        control_cycle(position);
    char output[64];
    unsigned count = 0;
    while (tx_tail != tx_head) {
        USART_UDRE_vect();
        output[count++] = UDR0;
    }
    output[count] = 0;
    assert(strcmp(output, expected) == 0);
}

static void check_pulses(int16_t increment, int sign)
{
    pulse_increment = increment;
    int32_t initial = position_pulses;
    unsigned pulses = 0;
    bool last_low = false, last_dir = (PORTB & DIR) != 0;
    for (unsigned i = 0; i < TIMER_HZ; ++i) {
        TIMER1_COMPA_vect();
        bool low = !(PORTB & STEP);
        bool dir = (PORTB & DIR) != 0;
        assert(!(low && last_low));
        if (dir != last_dir)
            assert(!low);
        if (low)
            ++pulses;
        last_low = low;
        last_dir = dir;
        pending = false; /* Model a serviced control loop, retain fixed velocity. */
    }
    assert(abs((int)pulses - (int)(fabsf((float)increment) * TIMER_HZ / 65536.0f)) <= 1);
    assert(position_pulses - initial == sign * (int32_t)pulses);
}

int main(void)
{
    PORTB = STEP | DIR;
    PINB = BUTTON;
    ADC = 480;
    assert(!enabled);
    wait_ms(20);
    PINB = 0;
    wait_ms(5);
    PINB = BUTTON;
    wait_ms(1);
    PINB = 0;
    wait_ms(19);
    assert(!enabled);
    wait_ms(1);
    assert(enabled && (PORTD & ENABLE));
    assert(position_pulses == 0 && pulse_increment == 0);
    assert(joystick_center == 480);
    wait_ms(30); /* A held press must not disable the newly enabled driver. */
    assert(enabled && position_pulses == 0);
    PINB = BUTTON;
    ADC = 480;
    wait_ms(20);
    ADC = 1023;
    wait_ms(1000);
    assert(enabled && position_pulses > 0 && pulse_increment > 0);
    int32_t before_press = position_pulses;
    PINB = 0;
    wait_ms(100);
    assert(enabled && (PORTD & ENABLE) && pulse_increment > 0);
    assert(joystick_center == 480);
    assert(position_pulses > before_press);
    PINB = BUTTON;
    wait_ms(20);
    before_press = position_pulses;
    PINB = 0;
    wait_ms(20);
    assert(enabled && position_pulses > before_press);
    PINB = BUTTON;
    ADC = 480;
    wait_ms(1000);

    position_pulses = MAX_PULSES / 2;
    check_pulses(100, 1);
    check_pulses(-100, -1);
    check_pulses(1, 1);
    for (int end = 0; end < 2; ++end) {
        int32_t limit = end ? MAX_PULSES : 0;
        position_pulses = limit;
        pulse_increment = end ? 20971 : -20971;
        pending = false;
        for (int i = 0; i < 100; ++i) {
            TIMER1_COMPA_vect();
            pending = false;
            assert(PORTB & STEP); /* No outward pulses at the bound. */
        }
        assert(enabled && (PORTD & ENABLE));
        assert(position_pulses == limit);
        pulse_increment = -pulse_increment;
        for (int i = 0; i < 100; ++i) {
            TIMER1_COMPA_vect();
            pending = false;
        }
        assert(enabled && (PORTD & ENABLE));
        assert(end ? position_pulses < limit : position_pulses > limit);
    }
    /* Missed control cycles retain enable and the last pulse increment. */
    position_pulses = MAX_PULSES / 2;
    pulse_increment = 1000;
    int32_t before_delay = position_pulses;
    for (int i = 0; i < 400; ++i)
        TIMER1_COMPA_vect();
    assert(pending && enabled && (PORTD & ENABLE));
    assert(pulse_increment == 1000 && position_pulses > before_delay);
    millisecond();
    assert(enabled && (PORTD & ENABLE));

    check_broadcast(0, 0, true, "sp -6.400\n");
    check_broadcast(1023, MAX_PULSES, true, "sp 6.400\n");
    check_broadcast(0, 0, false, "sp -6.400\n");
    check_broadcast(1023, MAX_PULSES, false, "sp 6.400\n");
    check_broadcast(480, 0, false, "sp 0.000\n");

    tx_head = tx_tail = 0;
    report(-3.2f);
    char output[64];
    unsigned count = 0;
    while (tx_tail != tx_head) {
        USART_UDRE_vect();
        output[count++] = UDR0;
    }
    output[count] = 0;
    assert(strcmp(output, "sp -3.200\n") == 0);
    serial_send("sp 1.234\n", false);
    serial_send("sp 0\n", true);
    count = 0;
    while (tx_tail != tx_head) {
        USART_UDRE_vect();
        output[count++] = UDR0;
    }
    output[count] = 0;
    assert(strcmp(output, "sp 0\n") == 0);
    serial_send("sp 1.234\n", false);
    USART_UDRE_vect();
    assert(UDR0 == 's');
    serial_send("sp 2.345\n", false);
    serial_send("sp 0\n", true);
    count = 0;
    while (tx_tail != tx_head) {
        USART_UDRE_vect();
        output[count++] = UDR0;
    }
    output[count] = 0;
    assert(strcmp(output, "p 1.234\nsp 0\n") == 0);
    puts("firmware state, pulse, delayed control, and serial checks passed");
}
