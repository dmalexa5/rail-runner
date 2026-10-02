#ifdef TELEOP_HOST_TEST
#include "tests/registers.h"
#else
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#endif
#include <stdbool.h>
#include <stdint.h>
#include "motion.h"

#define STEP _BV(PB1)
#define DIR _BV(PB0)
#define ENABLE _BV(PD7)
#define BUTTON _BV(PB4)
#define TIMER_HZ 40000UL
#define TX_SIZE 64

static volatile bool enabled, pending, busy, stopped;
static volatile int32_t position_pulses;
static volatile int16_t pulse_increment;
static uint16_t phase;
static bool pulse_low;
static uint8_t divider;
static bool previous_pressed;
static volatile uint8_t tx_head, tx_tail;
static volatile bool tx_in_line;
static char tx_buffer[TX_SIZE];

/* Called with interrupts masked, including from the timer ISR. */
static void disable(void)
{
    PORTD &= (uint8_t)~ENABLE;
    PORTB |= STEP;
    enabled = false;
    pulse_increment = 0;
    phase = 0;
    pulse_low = false;
    stopped = true;
}

ISR(TIMER1_COMPA_vect)
{
    /* Stop at the first sampled press; debounce is only needed to re-enable. */
    bool pressed = !(PINB & BUTTON);
    if (enabled && pressed && !previous_pressed)
        disable();
    previous_pressed = pressed;

    int16_t increment = pulse_increment;
    bool was_low = pulse_low;
    PORTB |= STEP;
    pulse_low = false;
    if (enabled && increment) {
        bool positive = increment > 0;
        bool old_positive = (PORTB & DIR) != 0;
        if (positive != old_positive) {
            PORTB ^= DIR;
            phase = 0;
        } else {
            uint32_t next = (uint32_t)phase +
                (uint16_t)(positive ? increment : -increment);
            if (next >= 65536UL && !was_low) {
                if ((positive && position_pulses < MAX_PULSES) ||
                    (!positive && position_pulses > 0)) {
                    PORTB &= (uint8_t)~STEP;
                    pulse_low = true;
                    position_pulses += positive ? 1 : -1;
                    next -= 65536UL;
                } else {
                    disable();
                    next = 0;
                }
            }
            phase = (uint16_t)next;
        }
    } else {
        phase = 0;
    }
    if (++divider == 40) {
        divider = 0;
        if (enabled && (pending || busy))
            disable();
        pending = true;
    }
}

ISR(USART_UDRE_vect)
{
    if (tx_tail == tx_head) {
        UCSR0B &= (uint8_t)~_BV(UDRIE0);
    } else {
        char byte = tx_buffer[tx_tail];
        UDR0 = byte;
        tx_in_line = byte != '\n';
        tx_tail = (tx_tail + 1) & (TX_SIZE - 1);
    }
}

static void serial_send(const char *line, bool priority)
{
    if (priority) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            uint8_t keep = tx_tail;
            /* Finish an already-started line so sp 0 remains a complete line. */
            if (tx_in_line) {
                while (keep != tx_head) {
                    char byte = tx_buffer[keep];
                    keep = (keep + 1) & (TX_SIZE - 1);
                    if (byte == '\n')
                        break;
                }
            }
            tx_head = keep;
        }
    }
    uint8_t length = 0;
    while (line[length])
        ++length;
    uint8_t head = tx_head;
    uint8_t free_bytes = (tx_tail - head - 1) & (TX_SIZE - 1);
    if (length > free_bytes)
        return; /* Drop the entire telemetry line rather than a partial line. */
    while (*line) {
        tx_buffer[head] = *line++;
        head = (head + 1) & (TX_SIZE - 1);
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tx_head = head;
        UCSR0B |= _BV(UDRIE0);
    }
}

static void report(float velocity)
{
    char line[20] = "sp ";
    uint8_t length = 3;
    int32_t milli = (int32_t)(velocity * 1000.0f);
    if (milli < 0) {
        line[length++] = '-';
        milli = -milli;
    }
    uint32_t whole = (uint32_t)milli / 1000;
    char digits[10];
    uint8_t count = 0;
    do {
        digits[count++] = '0' + whole % 10;
        whole /= 10;
    } while (whole);
    while (count)
        line[length++] = digits[--count];
    line[length++] = '.';
    line[length++] = '0' + (milli / 100) % 10;
    line[length++] = '0' + (milli / 10) % 10;
    line[length++] = '0' + milli % 10;
    line[length++] = '\n';
    line[length] = 0;
    serial_send(line, false);
}

static void control_cycle(int32_t pulses, bool did_stop)
{
    static float velocity, acceleration;
    static bool centered, press_ready;
    static uint8_t pressed_ms, released_ms, report_ms;
    if (did_stop) {
        velocity = acceleration = 0.0f;
        centered = false;
        press_ready = false;
        serial_send("sp 0\n", true);
    }
    bool pressed = !(PINB & BUTTON);
    if (pressed) {
        released_ms = 0;
        if (pressed_ms < 20)
            ++pressed_ms;
        if (pressed_ms == 20 && press_ready && !enabled) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                position_pulses = 0;
                phase = 0;
                pulse_increment = 0;
                PORTD |= ENABLE;
                enabled = true;
            }
            pulses = 0;
            velocity = acceleration = 0.0f;
            centered = false;
            press_ready = false;
        }
    } else {
        pressed_ms = 0;
        if (released_ms < 20)
            ++released_ms;
        if (released_ms == 20)
            press_ready = true;
    }
    float target = joystick_velocity(ADC);
    if (enabled) {
        if (target == 0.0f)
            centered = true;
        if (!centered)
            target = 0.0f;
        target = motion_safe_target(target, pulses / PULSES_PER_MM,
                                    velocity, acceleration);
        motion_profile_step(target, &velocity, &acceleration);
    } else {
        velocity = acceleration = 0.0f;
    }
    int16_t increment = (int16_t)(velocity * PULSES_PER_MM *
                                  (65536.0f / TIMER_HZ));
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (enabled)
            pulse_increment = increment;
    }
    if (++report_ms == 10) {
        report_ms = 0;
        report(enabled ? increment * (TIMER_HZ / 65536.0f) /
                         PULSES_PER_MM : 0.0f);
    }
}

#ifndef TELEOP_HOST_TEST
int main(void)
{
    /* Set safe output levels before changing the data direction. */
    PORTB |= STEP | DIR;
    PORTD &= (uint8_t)~ENABLE;
    DDRB |= STEP | DIR;
    DDRD |= ENABLE;
    PORTB |= BUTTON;
    ADMUX = _BV(REFS0) | _BV(MUX0); /* AVcc reference, ADC1 (A1). */
    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) |
        _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    DIDR0 = _BV(ADC1D);
    UCSR0A = _BV(U2X0);
    UBRR0 = 16; /* 117647 baud, +2.1% from 115200 at 16 MHz. */
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
    UCSR0B = _BV(TXEN0);
    TCCR1A = 0;
    OCR1A = F_CPU / TIMER_HZ - 1;
    TCCR1B = _BV(WGM12) | _BV(CS10);
    TIMSK1 = _BV(OCIE1A);
    sei();
    serial_send("sp 0\n", true);

    for (;;) {
        bool run = false, did_stop = false;
        int32_t pulses = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (pending) {
                pending = false;
                busy = true;
                run = true;
                pulses = position_pulses;
                did_stop = stopped;
                stopped = false;
            }
        }
        if (!run)
            continue;
        control_cycle(pulses, did_stop);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            busy = false;
        }
    }
}

#endif
