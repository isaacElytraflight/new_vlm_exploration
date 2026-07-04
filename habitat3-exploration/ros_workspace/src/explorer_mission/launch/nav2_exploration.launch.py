#!/usr/bin/env python3
"""Full exploration stack: SLAM + Nav2 + mission nodes (sensor-driven mapping)."""

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
    slam_params = os.path.join(explorer_share, "config", "slam_toolbox.yaml")

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
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py",
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

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_depth",
            arguments=["0", "0", "1.5", "0", "0", "0", "base_link", "depth_frame"],
        ),

        Node(
            package="explorer_bridge",
            executable="depth_camera_info_node",
            name="depth_camera_info",
            output="screen",
        ),

        Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            name="depthimage_to_laserscan",
            remappings=[
                ("depth", "/depth_data"),
                ("depth_camera_info", "/depth/camera_info"),
                ("scan", "/scan"),
            ],
            parameters=[{
                "output_frame": "depth_frame",
                "range_min": 0.1,
                "range_max": 3.5,
                "scan_height": 50,
            }],
            output="screen",
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    name="slam_toolbox",
                    output="screen",
                    parameters=[slam_params],
                    remappings=[
                        ("/map", "/grid_map"),
                        ("/map_metadata", "/grid_map_metadata"),
                    ],
                ),
            ],
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
            }],
            output="screen",
        ),
    ])
