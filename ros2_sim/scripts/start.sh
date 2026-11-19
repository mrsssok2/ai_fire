#!/bin/bash
# Runs INSIDE the container: build the workspace, launch, and read the keyboard.
#   Enter   start the mission now (otherwise it starts after start_delay seconds)
#   r+Enter reset: robot back to the start, tank refilled, fires relit
#   Ctrl+C  quit (nothing else closes the simulation)
# Extra args go to the launch file, e.g.  start_delay:=30 speed:=1.0 gui:=false
cd ~/ros2_sim
colcon build --symlink-install --event-handlers console_cohesion- 2>&1 | tail -n 15
set +u; source install/setup.bash
ros2 launch fire_robot_bringup sim.launch.py "$@" &
LAUNCH=$!
trap 'kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH; exit 0' INT TERM
if [ -t 0 ]; then
  echo
  echo ">>> Press Enter to start the mission now, r + Enter to reset, Ctrl+C to quit"
  while kill -0 $LAUNCH 2>/dev/null; do
    if read -r -t 1 line; then
      case "$line" in
        r|R|reset) ros2 topic pub --once /mission/command std_msgs/msg/String "{data: reset}" >/dev/null ;;
        *)         ros2 topic pub --once /mission/command std_msgs/msg/String "{data: start}" >/dev/null ;;
      esac
    fi
  done
else
  wait $LAUNCH
fi
