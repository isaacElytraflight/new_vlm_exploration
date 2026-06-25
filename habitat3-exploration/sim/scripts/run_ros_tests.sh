#!/usr/bin/env bash
# Run explorer ROS unit tests (gtest + pytest) inside the sim container or Jazzy env.
set -eo pipefail

WS="${EXPLORER_ROS_WS:-/opt/explorer_workspace/ros_workspace}"

export PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/bin:/opt/ros/jazzy/bin
export PYTHON_EXECUTABLE=/usr/bin/python3

source /opt/ros/jazzy/setup.bash
cd "$WS"

rm -rf build
colcon build --symlink-install \
  --packages-select explorer_msgs explorer_bridge explorer_mission \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

colcon test --packages-select explorer_mission --event-handlers console_direct+
colcon test-result --verbose

pytest src/explorer_bridge/test/ -v
pytest src/explorer_mission/test_py/ -v
