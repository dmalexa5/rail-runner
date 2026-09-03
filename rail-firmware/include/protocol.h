#ifndef PROTOCOL_H
#define PROTOCOL_H

/**
 * Processes one newline-delimited host command and queues one response.
 *
 * Stable commands:
 *   a          Arm, or refresh the 100 ms arm lease.
 *   d          Disarm and clear the requested torque.
 *   t,<Nm>     Set torque in newton-metres within +/-5 Nm while armed.
 *   s          Return full control, safety, timing, CAN, and motor state.
 *   f          Return position, velocity, and torque feedback.
 *
 * Successful commands return "k,<command>". Errors return "e,<code>", where
 * the code is a (arm rejected), i (invalid torque), r (torque rejected), or
 * c (unknown command). Status and feedback are positional CSV records beginning
 * with "s" and "f". Floating-point quantities are integer milli-units.
 *
 * The feedback fields are position, velocity, and torque. The status fields are
 * armed, safe, fault, commanded torque, cycles, overruns, maximum cycles, arm
 * lease age, feedback age, feedback sequence, motor valid, motor ID, position,
 * velocity, torque, temperature, motor error, hexadecimal CAN error, TX OK,
 * and TX failed.
 */
void protocol_handle_line(char *line);

#endif
