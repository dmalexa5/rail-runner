Create a plan to implement the following system in this repository:

# Overview

This repository contains all the software for two linear tracks
- _drive_ is the main ak-60-powered linear track and carries a franka-gello-duo configuration. The control of the franka arms is outside the scope of this system.
- _teleop_ is the teleoperation system. It is a nema-17-powered linear track, velocity-controlled by a joystick module.

## `rail-firmware` contains the software for two different board targets

- `BOARD=drive` targets an ak60-powered linear track
- `BOARD=teleop` targets a NEMA 17 powered teleoperation track
Both will flash to STM-32 Nucleo F446RE development boards.

**The AK60-6-V3.0 system**
The AK60 will operate in **servo velocity mode** at 48V with a max rated speed of 490 rpm. The max current in the drive has been configured to 5A.
- Right now, it is written as an MIT-mode controller. Completely overwrite this.
- Safety constants should be hardcoded `#define` constants
    - `MAX_JRK` 50 mm/s^3
    - `MAX_ACC` 50 mm/s^2
    - `MAX_VEL` 32 mm/s
    - `MIN_POS` 0 mm
    - `MAX_POS` 500 mm
- `POSITION_MARGIN` is 2 mm, so normal motion brakes to the 2--498 mm operating envelope. Inside either margin, only motion back toward the operating envelope is allowed.
- `CAL_VEL` is -10 mm/s. `PITCH` is 4 mm/rev, the AK60 has 14 pole pairs and 6:1 reduction, and positive motion is away from MIN. These constants scale linear values to ERPM and motor feedback.
- Safety limits use a time-optimal jerk-limited velocity profile. Position stopping calculations use measured position and commanded velocity/acceleration while guaranteeing the command does not exceed `MAX_JRK`, `MAX_ACC`, or `MAX_VEL`.
- The AK60 uses CubeMars servo CAN at 1 Mbps with motor ID 1 and 500 Hz feedback. The drive must already be configured for servo mode, 48 V, a 5 A current cap, and zero torque after 10 ms without CAN commands.
- The STM32 HAL and CMSIS dependencies come from the tracked `rail-firmware/vendor/STM32CubeF4` git submodule, including its HAL-driver and STM32F4-device nested submodules.
- Servo position feedback is treated as signed 0.1-degree output position with natural int16 rollover and is unwrapped in firmware. This rollover behavior and the positive motor direction must be checked on hardware before calibration.

Communication with the ros2 lifecycle node happens over a strict synchronous request/reply serial interface at 230400 baud and 500 Hz. There is no unsolicited boot or fault output. Requests accept LF or CRLF; replies use LF. Linear reply fields have one decimal place and use mm, mm/s, and mm/s^2.

Generally, messages take the form:
- `<cmd> <val>`
- `ack <pos> <vel> <acc>` or `<status> <code>`

Requests use the exact grammar `cal 0`, `dis 0`, or `sp <signed-decimal>` with at most one fractional digit.

All commands and non-error replies:
- `cal 0` <-- `cal 0` while calibrating, then `ack <pos> <vel> 0.0` when done
    - The host must continue sending `cal 0`; a 20 ms gap causes a constrained stop and communication fault.
    - If the debounced MIN optical switch is clear, ramp to -10 mm/s until it asserts. If already asserted, begin backing off immediately.
    - Reverse with the normal jerk/acceleration limits until the optical switch has been clear for 5 ms. Set the motor's temporary origin at that edge and require the next feedback to be within 0.2 mm of zero.
    - Brake to zero commanded velocity and acceleration. Calibration completes in active zero-velocity mode at the resulting positive measured position; it does not wait for measured velocity to settle.
    - Calibration has no time or travel guard. Independent hard-limit relays remove motor power at the physical ends.
- `dis 0` <-- `dis 0` immediately. Continue with a jerk-limited stop, switch to servo current mode at 0 A, invalidate calibration, and enter the deactivated state.
- `sp <vel>` <-- `ack <pos> <vel> <acc>`. Valid commands refresh a 20 ms watchdog. On timeout, stop with the normal motion limits, switch to 0 A, and latch `err com`.

All error values:
- `err cal` -- non `cal 0` message received while calibrating. Deactivate immediately.
- `err dis` -- non `cal 0` message received while in deactivated state. Remain deactivated.
- `err lim` -- setpoint received outside velocity limit, which is a fatal issue. Deactivate immediately.
- `err hrd` -- hardstop pressed. Disable the motor immediately. This requires manual intervention to resolve, intentionally.
- `err est` -- estop pressed. Disable the motor immediately.
- `err pos` -- motion requested farther outside the 2--498 mm operating envelope. Hold a zero target, remain active, and allow inward recovery.
- `err com` -- host command timeout. Stop under the motion limits, then deactivate.
- `err mot` -- CubeMars fault or rejected temporary-origin reset. Switch immediately to 0 A.
- `err can` -- CAN transmit failure, implausible position jump, or 10 ms without motor feedback. Switch immediately to 0 A.
- `err cmd` -- malformed command while active. Remain active; this does not refresh the watchdog.
- `err sys` -- control-loop overrun or UART queue overflow. Switch immediately to 0 A.

`err lim` applies only to an out-of-range host setpoint; measured overspeed is not fatal. `err pos` and an active-state `err cal` are nonfatal. All other faults invalidate calibration. Fault priority is estop, hard limit, system, motor, CAN, velocity limit, communication, then calibration/state/parser errors. A healthy `dis 0` clears a pending firmware fault without first reporting it; motor and CAN faults require fresh fault-free feedback. Hard-limit and estop recovery requires physical reset, `dis 0`, and a new calibration.

The MIN and MAX hard-limit switches share a normally-closed latching motor-power relay chain sensed on PA0. The normally-closed estop relay chain is sensed on PA1. Both independently remove motor power while leaving the Nucleo powered. The normally-closed MIN optical calibration switch is sensed on PC0. Emergency inputs are acted on immediately; only the optical input is debounced.

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

## `rail-interface` contains the software for two different ROS2 control nodes

- `rail-drive` node 
- `rail-teleop` node  

- setpoint velocity should be read on the `rail_velocity` node at 500 Hz
- state information should be a `sensor_msgs/msg/JointState` msg published on `rail_state`

## `rail-bringup` contains ros2 launch files and demos
Also, any ros2-based test scripts live in `rail-bringup/scripts`
- `ros2 launch rail-bringup drive.launch.py` -> launches the drive node and teleop node in unconfigured states
- `scripts/ui-drive.py` runs a debugging script that

## repo root

The ros2 portion of this system is designed to run in a docker container. Copy/adapt the docker container and compose file from @../crisp_dev, but strip out anything that is not needed for this project. Notably, the submodules and franka software.

- Do not update @README.md yet
- Keep code implementation readable but concise
- Keep documentation to an absolute minimum, doxygen-style header file documentation


Grill me incessantly with questions about the plan, walking through every branch of the design tree until we have reached a shared understanding of the requirements for the project.
