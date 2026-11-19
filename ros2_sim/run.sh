#!/bin/bash
# One command: builds the workspace in Docker and opens Gazebo with the robot and the fires.
# Extra args go to the launch file, e.g.  ./run.sh gui:=false   ./run.sh autonomous:=false
cd "$(dirname "$0")"
exec docker compose run --rm sim /home/ubuntu/ros2_sim/scripts/start.sh "$@"
