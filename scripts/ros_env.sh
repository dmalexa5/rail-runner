#!/usr/bin/env bash
# Must bash source this file
source /opt/ros/jazzy/setup.bash

cd /ros2_ws

if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [ -f "$HOME/.ros/cyclonedds.xml" ]; then
  export CYCLONEDDS_URI="file://$HOME/.ros/cyclonedds.xml"
fi

echo "Sourced ros2 environment."
