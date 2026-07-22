#!/usr/bin/env python3
"""Exploration stack: privileged pose + sensor occupancy + Nav2 (no slam_toolbox)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    explorer_share = get_package_share_directory("explorer_mission")
    nav2_params = os.path.join(explorer_share, "config", "nav2_params.yaml")

    use_privileged_map = LaunchConfiguration("use_privileged_map")
    realtime_mode = LaunchConfiguration("realtime_mode")
    navigation_mode = LaunchConfiguration("navigation_mode")
    frontiers_grid_topic = LaunchConfiguration("frontiers_grid_topic")

    exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("explorer_mission"),
                "launch",
                "exploration.launch.py",
            ])
        ]),
        launch_arguments={
            "use_privileged_map": use_privileged_map,
            "grid_topic": frontiers_grid_topic,
            "navigation_mode": navigation_mode,
        }.items(),
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("explorer_mission"),
                "launch",
                "navigation_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_privileged_map", default_value="false"),
        DeclareLaunchArgument("realtime_mode", default_value="false"),
        DeclareLaunchArgument("navigation_mode", default_value="nav2"),
        DeclareLaunchArgument(
            "frontiers_grid_topic",
            default_value="/global_costmap/costmap",
        ),

        exploration_launch,

        # map ≡ odom origin (episode start). Real robot: T265 provides odom→base;
        # map→odom stays identity (or from T265 world frame).
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_depth",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "depth_frame"],
        ),

        Node(
            package="explorer_bridge",
            executable="depth_camera_info_node",
            name="depth_camera_info",
            output="screen",
        ),

        Node(
            package="explorer_bridge",
            executable="depth_to_laserscan_node",
            name="depth_to_laserscan",
            parameters=[{
                "output_frame": "base_link",
                "range_min": 0.1,
                "range_max": 10.0,
                "scan_height": 24,
                "full_360": False,
                "band_anchor": "upper_third",
                "sensor_far": 50.0,
                "sat_eps": 0.5,
            }],
            output="screen",
        ),

        Node(
            package="explorer_bridge",
            executable="known_pose_mapper_node",
            name="known_pose_mapper",
            parameters=[{
                "scan_topic": "/scan",
                "grid_topic": "/grid_map",
                "map_frame": "map",
                "base_frame": "base_link",
                "resolution": 0.05,
                "initial_size_m": 20.0,
                "publish_hz": 5.0,
                "odom_topic": "/odom",
                "max_stamp_skew_sec": 0.0,
                "odom_cache_size": 2048,
                "pending_scan_limit": 128,
                "obstacle_inflation_m": 0.10,
            }],
            output="screen",
        ),

        TimerAction(
            period=5.0,
            actions=[nav2_navigation],
        ),

        Node(
            package="explorer_bridge",
            executable="cmd_vel_to_discrete_node",
            name="cmd_vel_to_discrete",
            parameters=[{
                "realtime_mode": realtime_mode,
                "realtime_max_linear_m_s": 0.1,
                "realtime_max_angular_deg_s": 30.0,
                "angular_threshold": 0.05,
                "linear_threshold": 0.03,
            }],
            output="screen",
        ),
    ])
