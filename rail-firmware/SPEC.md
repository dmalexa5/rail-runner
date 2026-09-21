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

The teleoperation track uses a NEMA17 stepper, a TB6600 driver configured for 1600 pulses/rev, a 20 mm/rev belt drive, and a local analog joystick. It has one normally-closed MIN switch and no estop. Position is the commanded pulse count and may drift if the motor skips steps.

### Motion

The drive motion limits and jerk-limited profile are reused in virtual drive coordinates. `SCALE` is 0.5, so the 250 mm physical track reports 0--500 mm, ±16 physical mm/s reports ±32 mm/s, and each STEP pulse is 0.025 reported mm. Normal motion brakes within the 2--498 mm virtual envelope; an outward joystick command at an endpoint is clamped to zero.

The 3.3 V joystick is sampled at 1 kHz. Its selected axis maps linearly outside a 5% center deadband to ±32 virtual mm/s. Motion is armed only after the joystick has entered the deadband following calibration. ADC initialization failure or 20 ms without a conversion is `err joy`.

Teleop pins are PA0/TIM5_CH1 STEP, PB0 DIR, PB1 ENA, PC0 MIN, PC1 joystick ADC, and PA2/PA3 USART2. STEP, DIR-positive, and ENA are active high. PC0 uses a pull-up; an open circuit or asserted normally-closed switch is active.

### Serial commands

USART2 runs at 230400 baud with strict request/reply, LF or CRLF requests, and LF replies. Position and velocity use one decimal place.

`<rec>` <-- `<reply>`

- `cal` <-- `cal` while calibrating, then `ack <pos> <vel>` when done
    - The host must continue sending `cal` during calibration
    - Seek MIN at -10 virtual mm/s, or back off immediately if MIN is already asserted.
    - Stop STEP on assertion, then reverse with the normal motion limits. Snapshot the first clear sample, require five consecutive clear samples, and set that edge to zero.
    - Brake to zero. Calibration completes at the resulting positive position.
    - Seeking is limited to 500 virtual mm and backoff to 10 virtual mm. Exceeding either immediately disables the driver and latches `err cal` until `dis`.
- `dis` <-- `dis` after a jerk-limited stop has completed and ENA is disabled. The host must allow 3 seconds for the reply. Calibration is invalidated.
- `req` <-- `ack <pos> <vel>`. `pos` is the scaled STEP count and `vel` is the scaled profile command sent for `rail-drive` to track. Valid requests refresh a 20 ms watchdog. Timeout performs a limited stop, disables ENA, and latches `err com`.

### Error responses

| Error | Cause | Response | Fatal? |
|---|---|---|---|
| `err cal` | Non-`cal` during calibration | Deactivate | No |
| `err dis` | Non-`cal` while deactivated | Stay deactivated | No |
| `err hrd` | Limit switch pressed during operation | Disable motor; manual reset | Yes |
| `err com` | Host command timeout | Limited stop; deactivate | Yes |
| `err joy` | ADC failure or 20 ms conversion loss | Disable | Yes |
| `err cmd` | Malformed active-state command | Stay active; don't refresh watchdog | No |
| `err sys` | Control overrun, UART overflow, or step-timer failure | Disable | Yes |

All fatal faults invalidate calibration. Priority is hard limit, system, joystick, then communication. A healthy `dis` clears a fault; hard-limit recovery also requires the switch to be released, and joystick recovery requires fresh ADC data.

### Hotloop

TIM2 releases a nonblocking foreground control cycle at 1 kHz and detects overruns. The cycle reads one request, samples safety and joystick state, advances calibration or the motion profile, updates the step rate, and writes at most one reply.

TIM5 is a separate 1 MHz, 32-bit output-compare edge scheduler. It emits 10 us STEP pulses, preserves phase when frequency changes, and counts rising edges. Direction changes only after the profile reaches zero; STEP is held low, DIR changes, and motion waits one full control cycle before restarting. PC0 EXTI stops STEP and disables ENA immediately; the foreground cycle treats this as a calibration event or an active hard-limit fault.
