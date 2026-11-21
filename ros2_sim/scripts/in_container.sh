#!/bin/bash
# Build the workspace and run a script inside the container, e.g.
#   docker compose run --rm -T sim /home/ubuntu/ros2_sim/scripts/in_container.sh scripts/eval_fusion.py --seeds 6
cd ~/ros2_sim
colcon build --symlink-install --event-handlers console_cohesion- 2>&1 | tail -n 3
set +u; source install/setup.bash
exec python3 -u "$@"
