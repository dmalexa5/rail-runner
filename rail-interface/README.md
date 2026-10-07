# rail-interface

ROS 2 nodes for the rail firmware. See [SPEC.md](SPEC.md) for their behavior.

- `rail-drive` is a lifecycle node controlling the drive firmware.
- `rail-teleop` starts forwarding joystick serial setpoints immediately, without
  lifecycle transitions. It applies no motion limits; safety belongs to rail-drive.

Build in a ROS 2 Jazzy environment:

```sh
colcon build --packages-select rail_interface
source install/setup.bash
ros2 run rail_interface rail-teleop --ros-args -p serial.port:=/dev/ttyUSB0
```

Teleop uses fixed 115200 baud and publishes `std_msgs/msg/Float64` in m/s on
`/rail_velocity`, converting firmware values from mm/s without scaling.
Full joystick deflection publishes ±0.032 m/s (±32.0 mm/s). Override `command_topic` to
select another topic. Serial output begins after the firmware's first joystick
press. Silence produces no ROS messages; serial connection failures end the
node.

```sh
colcon test --packages-select rail_interface
colcon test-result --verbose
```

The teleop integration tests use pseudo-terminals and require no hardware.
