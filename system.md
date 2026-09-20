Create a plan to implement the following system in this repository:

# Overview

This repository contains all the software for two linear tracks
- _drive_ is the main ak-60-powered linear track and carries a franka-gello-duo configuration. The control of the franka arms is outside the scope of this system.
- _teleop_ is the teleoperation system. It is a nema-17-powered linear track, velocity-controlled by a joystick module.

## `rail-firmware` contains the software for two different board targets

- `BOARD=drive` targets an ak60-powered linear track
- `BOARD=teleop` targets a NEMA 17 powered teleoperation track
Both will flash to STM-32 Nucleo F446RE development boards.

The AK60-6-V3.0 system will operate in **servo velocity mode** at 48V with a max rated speed of 490 rpm. The max current in the drive has been configured to 5A.
- Right now, it is written as an MIT-mode controller. Completely overwrite this.
- 
 


## `rail-interface` contains the software for two different ROS2 control nodes

- `rail-drive` node 
- `rail-teleop` node  

Grill me incessantly with questions about the plan, walking through every branch of the design tree until we have reached a shared understanding of the requirements for the project.

