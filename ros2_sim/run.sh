#!/bin/bash
# One command: builds the workspace in Docker and opens Gazebo (camera follows the robot) + the Fire Detection window.
# The robot waits 15 s (countdown on the window) or until you press Enter / click START; simulation runs at half speed.
# Extra args go to the launch file, e.g.
#   ./run.sh speed:=1.0 start_delay:=5        ./run.sh start_delay:=0        ./run.sh gui:=false monitor_window:=false
#   ./run.sh autonomous:=false                ./run.sh tank_percent:=8       ./run.sh extinguish_time:=8
cd "$(dirname "$0")"
exec docker compose run --rm sim /home/ubuntu/ros2_sim/scripts/start.sh "$@"
