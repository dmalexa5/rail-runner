# `rail-interface` contains the software for two different ROS2 control nodes

---

### lifecycle nodes

The following specifies the finite state machine behavior of this node for each lifecycle state and its transitions

--- 

**Unconfigured** state
The do-nothing unconfigured state

**Configuring** — Unconfigured → Inactive
- Load parameters with generate parameters ros2 library
    - [rail-drive.yaml](./src/rail-drive.yaml) for drive node params
    - [rail-teleop.yaml](./src/rail-teleop.yaml) for teleop node params
- Send disable message to verify healthy state connection. If this fails, shutdown and enter finalized state immediately.

---

**Inactive** state
The passive, but connection-established state. Motor disabled/0 A.

**Activating** — Inactive → Active
- Run calibration (see [rail-firmware/SPEC.md](../rail-firmware/SPEC.md) for calibration protocol)
- If calibration successfull, enter Active state.

**CleaningUp** — Inactive → Unconfigured
- Send disable message to verify healthy state. If fails, shutdown.

---

**Active** state

The active node state, where constant communication with the rail firmware is occuring.

See [rail-firmware/SPEC.md](../rail-firmware/SPEC.md) for each respective communication protocol

- `rail-drive` will receive a stream of velocity setpoints
- `rail-teleop` will publish a stream of velcooity setpoints

Note that due to the nature of the design, either system should work independantly of whether the other is launched.

**Deactivating** — Active → Inactive
- Send disable message to verify healthy connection. If fails, shutdown. Otherwise, enter inactive state.

---

**Finalized** state
ShuttingDown — from any primary state → Finalized

---

**ErrorProcessing** — entered on any failure; leads either back to Unconfigured or on to Finalized depending on whether recovery succeeds

---


- setpoint velocity should be read on the `rail_velocity` node at 500 Hz
- state information should be a `sensor_msgs/msg/JointState` msg published on `rail_state`

### `rail-teleop` lifecycle node
