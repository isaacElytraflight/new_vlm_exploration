#!/usr/bin/env bash
# Run explorer_bridge unit tests inside the sim container (or any Jazzy env).
set -eo pipefail

WS="${EXPLORER_ROS_WS:-/opt/explorer_workspace/ros_workspace}"

export PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/bin:/opt/ros/jazzy/bin
export PYTHON_EXECUTABLE=/usr/bin/python3

source /opt/ros/jazzy/setup.bash
cd "$WS"

# Bind-mount replaces image install/ — rebuild with system Python (not conda).
rm -rf build
colcon build --symlink-install \
  --packages-select explorer_msgs explorer_bridge \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
pytest src/explorer_bridge/test/ -v
