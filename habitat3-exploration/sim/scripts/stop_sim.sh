#!/usr/bin/env bash
# Hard-stop helper: kills episode, ROS bridge, habitat engine, and live viewer.
pkill -INT -f "ros2 launch explorer_bridge" 2>/dev/null
pkill -INT -f explorer_bridge_node 2>/dev/null
pkill -INT -f habitat_engine.py 2>/dev/null
pkill -INT -f explore_episode.py 2>/dev/null
sleep 2
pkill -f "ros2 launch explorer_bridge" 2>/dev/null
pkill -f explorer_bridge_node 2>/dev/null
pkill -f habitat_engine.py 2>/dev/null
pkill -f explore_episode.py 2>/dev/null
pkill -f "feh.*habitat_live" 2>/dev/null
pkill -f "live_viewer.py" 2>/dev/null
rm -f /tmp/habitat_engine.sock 2>/dev/null || true
echo "Stopped."
exit 0
