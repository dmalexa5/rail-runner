#ifndef PROTOCOL_H
#define PROTOCOL_H

/**
 * Processes one newline-delimited host command and queues one response.
 *
 * Stable commands:
 *   arm             Arm, or refresh the 100 ms arm lease.
 *   disarm          Disarm and clear the requested torque.
 *   torque <Nm>     Set torque in newton-metres within +/-5 Nm while armed.
 *   status          Return control, safety, timing, CAN, and motor state.
 *
 * Successful commands return "OK <command>". Rejected commands return
 * "ERR <reason>". Status is one line beginning with "STATUS" and reports
 * floating-point quantities as integer milli-units for stable parsing.
 */
void protocol_handle_line(char *line);

#endif
