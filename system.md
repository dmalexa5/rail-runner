Create a plan to implement the following system in this repository:

# Overview

This repository contains all the software for two linear tracks
- _drive_ is the main ak-60-powered linear track and carries a franka-gello-duo configuration. The control of the franka arms is outside the scope of this system.
- _teleop_ is the teleoperation system. It is a nema-17-powered linear track, velocity-controlled by a joystick module.

## `rail-firmware` contains the software for two different board targets

- `BOARD=drive` targets an ak60-powered linear track
- `BOARD=teleop` targets a NEMA 17 powered teleoperation track
Both will flash to STM-32 Nucleo F446RE development boards.

[rail-firmware/SPEC.md](rail-firmware/SPEC.md)

### The AK60-6-V3.0 system


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
