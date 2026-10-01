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
    busy = true;
    bool did_stop = stopped;
    stopped = false;
    control_cycle(position_pulses, did_stop);
    busy = false;
    while (tx_tail != tx_head)
        USART_UDRE_vect();
}

static void wait_ms(unsigned count)
{
    while (count--)
        millisecond();
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
    PIND = BUTTON;
    ADC = 1023;
    assert(!enabled);
    wait_ms(20);
    PIND = 0;
    wait_ms(5);
    PIND = BUTTON;
    wait_ms(1);
    PIND = 0;
    wait_ms(19);
    assert(!enabled);
    wait_ms(1);
    assert(enabled && (PORTD & ENABLE));
    assert(position_pulses == 0 && pulse_increment == 0);
    wait_ms(30); /* A held press must not disable the newly enabled driver. */
    assert(enabled && position_pulses == 0);
    PIND = BUTTON;
    ADC = 512;
    wait_ms(20);
    ADC = 1023;
    wait_ms(1000);
    assert(enabled && position_pulses > 0 && pulse_increment > 0);
    PIND = 0;
    TIMER1_COMPA_vect();
    assert(!enabled && !(PORTD & ENABLE) && (PORTB & STEP));
    assert(pulse_increment == 0);
    wait_ms(100);
    assert(!enabled);
    PIND = BUTTON;
    wait_ms(20);
    PIND = 0;
    wait_ms(20);
    assert(enabled && position_pulses == 0);
    PIND = BUTTON;
    ADC = 512;
    wait_ms(20);

    position_pulses = 100000;
    check_pulses(20971, 1);
    check_pulses(-20971, -1);
    check_pulses(1, 1);
    pulse_increment = -20971;
    position_pulses = 0;
    for (int i = 0; i < 10; ++i)
        TIMER1_COMPA_vect();
    assert(!enabled && position_pulses == 0);
    enabled = true;
    PORTD |= ENABLE;
    pulse_increment = 20971;
    position_pulses = MAX_PULSES;
    pending = busy = false;
    for (int i = 0; i < 10; ++i)
        TIMER1_COMPA_vect();
    assert(!enabled && position_pulses == MAX_PULSES);
    enabled = true;
    busy = true;
    for (int i = 0; i < 40; ++i)
        TIMER1_COMPA_vect();
    assert(!enabled && stopped);
    busy = false;
    enabled = true;
    pending = true;
    for (int i = 0; i < 40; ++i)
        TIMER1_COMPA_vect();
    assert(!enabled && stopped);

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
    puts("firmware state, pulse, deadline, and serial checks passed");
}
