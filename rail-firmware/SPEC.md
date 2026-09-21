# `rail-firmware` contains the software for two different board targets

- `BOARD=drive` targets an ak60-powered linear track
- `BOARD=teleop` targets a NEMA 17 powered teleoperation track
Both flash to STM-32 Nucleo F446RE development boards.

## The AK60-6-V3.0 system

The AK60 operates in **servo velocity mode** at 48V with a max rated speed of 490 rpm. The max current in the drive has been configured to 5A.

### Motion

Hardcoded `#define` constants    
- `MAX_ACC` 50 mm/s^2
- `MAX_VEL` 32 mm/s
- `MIN_POS` 0 mm
- `MAX_POS` 500 mm
- `POSITION_MARGIN` is 2 mm, so normal motion brakes to the 2--498 mm operating envelope. Inside either margin, only motion back toward the operating envelope is allowed.
- `CAL_VEL` is -10 mm/s 
- `PITCH` is 4 mm/rev
- Safety limits use a time-optimal jerk-limited velocity profile

The AK60 uses CubeMars servo CAN at 1 Mbps with motor ID 1 and 500 Hz feedback. The drive must already be configured for servo mode, 48 V, a 5 A current cap, and zero torque after 10 ms without CAN commands.

Servo position feedback is treated as signed 0.1-degree output position with natural int16 rollover and is unwrapped in firmware. This rollover behavior and the positive motor direction must be checked on hardware before calibration.

Comms with ros2 lifecycle node specified in `rail-interface/SPEC.md` occur over 230400 baud serial interface:
- strict req/reply
- requests accept LF/CRLF
- replies use LF

Generally, messages take the form:
- `<cmd> <val>`
- `ack <pos> <vel> <acc>` or `<status> <code>`

Requests use the exact grammar `cal 0`, `dis 0`, or `sp <signed-decimal>` with at most one fractional digit.
    - mm, mm/s, and mm/s^2 units

### Serial commands

`<rec>` <-- `<reply>`

- `cal 0` <-- `cal 0` while calibrating, then `ack <pos> <vel> 0.0` when done
    - The host must continue sending `cal 0` during calibration
    - If the debounced MIN optical switch is clear, ramp to -10 mm/s until it asserts. If already asserted, begin backing off immediately.
    - Reverse with the normal jerk/acceleration limits until the optical switch has been clear for 5 ms. Set the motor's temporary origin at that edge and require the next feedback to be within 0.2 mm of zero.
    - Brake to zero commanded velocity and acceleration. Calibration completes in active zero-velocity mode at the resulting positive measured position and does not wait for measured velocity to settle.
- `dis 0` <-- `dis 0` immediately. Continue with a jerk-limited stop, switch to servo current mode at 0 A, invalidate calibration, and enter the deactivated state.
- `sp <vel>` <-- `ack <pos> <vel> <acc>`. Valid commands refresh a 20 ms watchdog. On timeout, stop with the normal motion limits, switch to 0 A, and latch `err com`.


### Error responses

| Error | Cause | Response | Fatal? |
|---|---|---|---|
| `err cal` | Non-`cal 0` during calibration | Deactivate | No |
| `err dis` | Non-`cal 0` while deactivated | Stay deactivated | No |
| `err lim` | Setpoint exceeds velocity limit | Deactivate | Yes |
| `err hrd` | Hardstop pressed | Disable motor; manual reset | Yes |
| `err est` | Estop pressed | Disable motor | Yes |
| `err pos` | Motion farther outside 2--498 mm | Hold zero; allow inward motion | No |
| `err com` | Host command timeout | Limited stop; deactivate | Yes |
| `err mot` | CubeMars fault or origin-reset rejection | Set 0 A | Yes |
| `err can` | TX failure, position jump, or 10 ms feedback loss | Set 0 A | Yes |
| `err cmd` | Malformed active-state command | Stay active; don't refresh watchdog | No |
| `err sys` | Control overrun or UART overflow | Set 0 A | Yes |

All fatal faults invalidate calibration. 

Fault priority is estop, hard limit, system, motor, CAN, velocity limit, communication, then calibration/state/parser errors. A healthy `dis 0` clears a pending firmware fault without first reporting it; motor and CAN faults require fresh fault-free feedback. Hard-limit and estop recovery requires physical reset, `dis 0`, and a new calibration.

The MIN and MAX hard-limit switches share a normally-closed latching motor-power relay chain sensed on PA0. The normally-closed estop relay chain is sensed on PA1. Both independently remove motor power while leaving the Nucleo powered. The normally-closed MIN optical calibration switch is sensed on PC0. Emergency inputs are acted on immediately; only the optical input is debounced.

### Hotloop

This communication protocol runs inside a single nonblocking hotloop released by an STM32 timer ISR at 1 kHz. The ISR only releases cycles and detects overruns.

1) read protocol
2) read safety info
    - if hardstop switches pressed, kill
    - if estop pressed, kill
    - if a fatal fault is present, command 0 A
3) read state info
    - read motor feedback position and velocity
    - update linear position and velocity
4) control loop
    - set velocity command to the requested velocity, adjusted for acceleration, jerk, and position limits
5) reply protocol

## The NEMA17 system

The teleoperation linear track is driven by a nema17 stepper motor and tb6600 driver. There is no estop for this system, and only a single limit switch for the belt driven linear slide. The stepper pulses are sent via the common timer-base frequency-to-velocity control architecture.

### Motion

The same motion hardcodes apply, scaled down for the linear track:
- `SCALE` is 0.5, as the teleop track is half the size of the  
- There is no `PITCH` param, only `MM_PER_REV` 20 mm

### Serial commands

The serial interface is similar to the AK60 interface, but is simplified for an open-loop stepper motor controlled rail (no feedback). Also, the teleop rail velocity is commanded with a joystick module- we rely on the motion manager to prevent velocities that `rail-drive` cannot keep up with.

`<rec>` <-- `<reply>`

- `cal` <-- `cal` while calibrating, then `ack <pos> <vel> 0.0` when done
    - The host must continue sending `cal` during calibration
    - Reverse with the normal jerk/acceleration limits until the optical switch has been clear for 5 ms. Set the current position to 0 at that edge.
    - Brake to zero commanded velocity and acceleration. Calibration completes in active zero-velocity mode at the resulting positive position.
- `dis` <-- `dis` immediately. Continue with a jerk-limited stop, switch to servo current mode at 0 A, invalidate calibration, and enter the deactivated state.
- `req` <-- `ack <pos> <vel>`. Valid commands refresh a 20 ms watchdog. On timeout, stop with the normal motion limits, switch to 0 A, and latch `err com`.
    - `pos` is an integrated counter. If stepper motor skips steps, this will drift. Ok for now.
    - `vel` is the scaled commanded velocity, 1:1 to what `rail-drive` will be tracking

### Error responses

| Error | Cause | Response | Fatal? |
|---|---|---|---|
| `err cal` | Non-`cal` during calibration | Deactivate | No |
| `err dis` | Non-`cal` while deactivated | Stay deactivated | No |
| `err hrd` | Limit switch pressed during operation | Disable motor; manual reset | Yes |
| `err pos` | Motion farther outside 2--498 mm | Hold zero; allow inward motion | No |
| `err com` | Host command timeout | Limited stop; deactivate | Yes |
| `err joy` | joystick failure, or 20 ms feedback loss | Disable | Yes |

All fatal faults invalidate calibration. 

### Hotloop

