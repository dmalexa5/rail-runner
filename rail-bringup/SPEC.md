# `rail-bringup` contains test scripts and launch files

## launch files

- ros2-based test scripts live in `rail-bringup/scripts`
- `ros2 launch rail-bringup drive-alone.launch.py` launches the drive node alone
- `ros2 launch rail-bringup teleop-alone.launch.py` launches the teleop node alone
- `ros2 launch rail-bringup teleop.launch.py` launches both nodes

<!-- Eventually, also, there will be a foxglove launch script -->
<!-- - `ros2 launch rail-bringup foxglove.launch.py` launches the foxglove bridge and then websocket with a specified layout -->

## test scripts

**`scripts/drive-node.py --vel <velocity>`** runs a debugging script that lets a user configure, activate, and deactivate the drive node (designed to run alongside `drive-alone` for testing)

- after `activate`, read the left and right arrow keys until `Ctrl+C`, then immediately `deactivate`
- left/right inject 10 mm/s velocity steps while held, 0 mm/s while released

This will test the rail-firmware smoothing functionality

**`scripts/teleop-node.py`** runs a debugging script that lets a user configure, activate, and deactivate the drive node (designed to run alongside `drive-alone` for testing)

<!-- Eventually, this will launch a foxglove vizualization, but I still need to create the layout and get the layoutID -->
- after `activate` do nothing until `Ctrl+C`, then immediately `deactivate`
