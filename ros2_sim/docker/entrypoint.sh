#!/bin/bash
# Source ROS 2 and the workspace overlay (if built), then run the command.
export GZ_PARTITION="${GZ_PARTITION:-$(hostname)}"
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_sim/install/setup.bash" ] && source "$HOME/ros2_sim/install/setup.bash"
exec "$@"
