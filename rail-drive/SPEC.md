# `rail-drive` contains the software for the drive board

`BOARD=drive` targets an ak60-powered linear track and flashes to an STM-32 Nucleo F446RE development board.

---

## The AK60-6-V3.0 system

The AK60 operates in **MIT torque mode** at 48V with a max rated speed of 490 rpm. Velocity is commanded through the MIT frame with zero position target, zero position gain, zero feed-forward torque, and a velocity gain Kd that defaults to 0.100.

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

The AK60 uses CubeMars extended-ID CAN at 1 Mbps with motor ID 2 and 500 Hz feedback. The drive must already be configured for MIT torque mode, 48 V, a 5 A current cap, and zero torque after 10 ms without CAN commands. Each active command is one 8-byte packet-type-8 frame containing zero Kp, the selected Kd, zero position, output-shaft velocity in rad/s, and zero feed-forward torque. Deactivated and faulted states send the same frame with all five logical fields zero.

Servo position feedback is treated as signed 0.1-degree output position with natural int16 rollover and is unwrapped in firmware. Calibration establishes a firmware-local position origin without changing the motor's internal origin or feedback configuration. This rollover behavior and the positive motor direction must be checked on hardware before calibration.

Comms with ros2 lifecycle node specified in `rail-interface/SPEC.md` occur over a 115200 baud serial interface at a 250 Hz transaction rate:
- requests receive one reply, except that `cal` also emits its completion reply asynchronously
- requests accept LF/CRLF
- replies use LF

Generally, messages take the form:
- `<cmd> <val>`
- `ack <pos> <vel> <acc>` or `<status> <code>`

Requests use the exact grammar `cal`, `dis`, `sp <signed-decimal>` with at most one fractional digit, or `kd <unsigned-decimal>` with at most three fractional digits.
    - mm, mm/s, and mm/s^2 units

### Serial commands

`<rec>` <-- `<reply>`

- `cal` <-- `cal` while calibrating, then `ack <pos> <vel> 0.0` when done
    - `cal` is sent once. Calibration times out after 30 seconds with `err cal`.
    - If the debounced MIN optical switch is clear, ramp to -10 mm/s until it asserts. If already asserted, begin backing off immediately.
    - Reverse with the normal jerk/acceleration limits until the optical switch has been clear for 5 ms. Set the firmware position origin to the first clear sample in that confirmed interval.
    - Brake to zero commanded velocity and acceleration. Calibration completes in active zero-velocity mode at the resulting positive measured position and does not wait for measured velocity to settle.
- `dis` <-- `dis` immediately. `dis` is sent once. Continue with a jerk-limited stop, switch to the all-zero MIT command, invalidate calibration, and enter the deactivated state.
- `sp <vel>` <-- `ack <pos> <vel> <acc>`. Valid commands refresh a 50 ms watchdog. On timeout, stop with the normal motion limits, switch to the all-zero MIT command, and latch `err com`.
- `kd <gain>` <-- `kd <gain>` formatted to three fractional digits. Values from 0 through 1.000 are accepted in every state, apply immediately, persist until MCU reset, and do not refresh the host watchdog or alter the current state. Malformed and out-of-range values return nonfatal `err kd` without changing the gain.


### Error responses

| Error | Cause | Response | Fatal? |
|---|---|---|---|
| `err cal` | Non-`cal` during calibration | Deactivate | No |
| `err dis` | Non-`cal` while deactivated | Stay deactivated | No |
| `err lim` | Setpoint exceeds velocity limit | Deactivate | Yes |
| `err hrd` | Hardstop pressed | Disable motor; manual reset | Yes |
| `err est` | Estop pressed | Disable motor | Yes |
| `err pos` | Motion farther outside 2--498 mm | Hold zero; allow inward motion | No |
| `err com` | Host command timeout | Limited stop; deactivate | Yes |
| `err mot` | CubeMars fault | Send all-zero MIT command | Yes |
| `err can` | TX failure or 10 ms feedback loss | Send all-zero MIT command | Yes |
| `err cmd` | Malformed active-state command | Keep current state; don't refresh watchdog | No |
| `err kd` | Invalid `kd` in any state | Keep current state; don't refresh watchdog | No |
| `err sys` | Control overrun or UART overflow | Send all-zero MIT command | Yes |

All fatal faults invalidate calibration. 

Fault priority is estop, hard limit, system, motor, CAN, velocity limit, communication, then calibration/state/parser errors. A healthy `dis` clears a pending firmware fault without first reporting it; motor and CAN faults require fresh fault-free feedback. Hard-limit and estop recovery requires physical reset, `dis`, and a new calibration.

The MIN and MAX hard-limit switches share a normally-open, active-low input sensed on PA0. The normally-closed estop relay chain is sensed on PA1. Both independently remove motor power while leaving the Nucleo powered. The normally-open, active-low MIN optical calibration switch is sensed on PC0. Emergency inputs are acted on immediately; only the optical input is debounced.

### Hotloop

This communication protocol runs inside a single nonblocking hotloop released by an STM32 timer ISR at 1 kHz. The ISR only releases cycles and detects overruns.

1) read protocol
2) read safety info
    - if hardstop switches pressed, kill
    - if estop pressed, kill
    - if a fatal fault is present, send the all-zero MIT command
3) read state info
    - read motor feedback position and velocity
    - update linear position and velocity
4) control loop
    - set velocity command to the requested velocity, adjusted for acceleration, jerk, and position limits, and convert from mm/s to output-shaft rad/s
5) reply protocol

### Electrical

The NUCLEO-F446RE connections for `BOARD=drive` are:

| Function | Nucleo connector | MCU pin | Connect to |
|---|---|---|---|
| CAN receive | D15, CN5 pin 10 | PB8 / CAN1_RX | CAN transceiver RXD |
| CAN transmit | D14, CN5 pin 9 | PB9 / CAN1_TX | CAN transceiver TXD |
| Hard-limit chain | CN7 pin 28 | PA0 | Normally-open switch contact to GND |
| Estop chain | CN7 pin 30 | PA1 | Normally-closed relay contact to GND |
| MIN optical switch | CN7 pin 38 | PC0 | Normally-open switch output to GND |
| Serial transmit | D1, CN9 pin 2 | PA2 / USART2_TX | Onboard ST-LINK USB virtual COM |
| Serial receive | D0, CN9 pin 1 | PA3 / USART2_RX | Onboard ST-LINK USB virtual COM |
| Logic ground | CN7 pin 20 or 22 | GND | All external logic grounds |

PB8 and PB9 are logic-level CAN signals, not CANH and CANL.
Use a 3.3 V-compatible CAN transceiver between the Nucleo and the motor CAN bus.
The three safety inputs use internal pull-ups. The hard-limit and optical inputs are high
when inactive and low when their normally-open contacts close. The estop input is low
when its normally-closed contact is healthy and high when the contact opens.
The relay contacts must be voltage-free; never apply motor voltage to a Nucleo pin.

USART2 is connected to the onboard ST-LINK virtual COM port by default and runs at
115200 baud, 8 data bits, no parity, and 1 stop bit.
No jumper wires are required for the normal USB serial connection.
Before using D0 and D1 with an external 3.3 V UART, configure the board's solder bridges
to avoid contention with the ST-LINK virtual COM port.
