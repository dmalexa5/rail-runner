# `rail-interface` contains the software for two different ROS2 control nodes

## Teleop node

`rail-teleop` is an ordinary ROS 2 node named `rail_teleop`. It opens serial on
startup at fixed 115200 baud, 8N1, and receives newline-terminated `sp <velocity>`
messages from [rail-teleop](../rail-teleop/README.md). It sends no serial commands
and requires no handshake or lifecycle transitions.

Each valid finite setpoint is published as `std_msgs/msg/Float64` in m/s after
converting the firmware value from mm/s. Publication follows the firmware stream
(normally 100 Hz after the first joystick press), with SensorDataQoS and depth 1.
No clamping, deadband, position limits, or acceleration/jerk limits are applied.
Safety and stale-command handling belong to the rail-drive stack.

Parameters are read-only: `serial.port` defaults to `/dev/ttyUSB0`, and
`command_topic` defaults to `rail_velocity`. Silence produces no messages.
Malformed, nonfinite, or oversized lines (over 95 characters excluding newline)
are skipped with throttled warnings. Serial open failures or disconnections end
the node with a nonzero exit status; shutdown releases the serial connection.

The lifecycle behavior below applies only to `rail-drive`.

---

### lifecycle nodes

The following specifies the finite state machine behavior of this node for each lifecycle state and its transitions

--- 

**Unconfigured** state
The do-nothing unconfigured state

**Configuring** — Unconfigured → Inactive
- Load parameters with generate parameters ros2 library
    - [rail-drive.yaml](./src/rail-drive.yaml) for drive node params
- Open the serial link at 230400 baud and require a successful `dis 0` handshake. If this fails, enter Finalized.

---

**Inactive** state
The passive, connection-established state. A `dis 0` acknowledgement means the firmware accepted the disable request; its limited stop to 0 A may still be completing.

**Activating** — Inactive → Active
- Run calibration (see [rail-drive/SPEC.md](../rail-drive/SPEC.md) for calibration protocol)
- If calibration succeeds within the configured timeout, enter Active. Otherwise, recover to Unconfigured only if `dis 0` succeeds.

**CleaningUp** — Inactive → Unconfigured
- Send disable message to verify healthy state. If fails, shutdown.

---

**Active** state

The active node state, where constant communication with the rail firmware is occuring.

See [rail-drive/SPEC.md](../rail-drive/SPEC.md) for each respective communication protocol

- `rail-drive` will receive a stream of velocity setpoints

Note that due to the nature of the design, either system should work independantly of whether the other is launched.

`rail-drive` performs strict request/reply transactions on a 500 Hz wall timer. Stale or invalid ROS commands produce `sp 0.0`. A nonfatal `err pos` leaves the node Active; any other firmware error or communication failure finalizes it after a best-effort disable.

**Deactivating** — Active → Inactive
- Send disable message to verify healthy connection. If fails, shutdown. Otherwise, enter inactive state.

---

**Finalized** state
ShuttingDown — from any primary state → Finalized

---

**ErrorProcessing** — transition failures lead back to Unconfigured only when activation recovery successfully disables the firmware; otherwise they lead to Finalized.

---


- velocity setpoints are received as `std_msgs/msg/Float64` in m/s on `rail_velocity`
- position and velocity are published in SI units as `sensor_msgs/msg/JointState` on `rail_state`
