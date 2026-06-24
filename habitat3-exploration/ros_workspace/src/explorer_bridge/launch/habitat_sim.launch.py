#!/usr/bin/env python3
"""Launch explorer_bridge with habitat backend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("driver_backend", default_value="habitat"),
            DeclareLaunchArgument("habitat_socket_path", default_value="/tmp/habitat_engine.sock"),
            DeclareLaunchArgument("publish_hz", default_value="15.0"),
            Node(
                package="explorer_bridge",
                executable="explorer_bridge_node",
                name="explorer_bridge_node",
                output="screen",
                parameters=[
                    {"driver_backend": LaunchConfiguration("driver_backend")},
                    {"habitat_socket_path": LaunchConfiguration("habitat_socket_path")},
                    {"publish_hz": LaunchConfiguration("publish_hz")},
                ],
            ),
        ]
    )
