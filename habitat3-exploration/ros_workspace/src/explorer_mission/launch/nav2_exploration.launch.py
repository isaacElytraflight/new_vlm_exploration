#!/usr/bin/env python3
"""Full exploration stack: SLAM + Nav2 + mission nodes (sensor-driven mapping)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


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

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": False, "use_lifecycle_manager": False},
        ],
        remappings=[
            ("/map", "/grid_map"),
            ("/map_metadata", "/grid_map_metadata"),
        ],
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[exploration] Activating slam_toolbox lifecycle node"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
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
            # Low forward depth camera (~10 cm above ground), level with base_link.
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
                "range_max": 5.0,
                "scan_height": 24,
                "full_360": True,
                "band_anchor": "bottom",
            }],
            output="screen",
        ),

        TimerAction(
            period=3.0,
            actions=[slam_toolbox_node, slam_configure],
        ),

        slam_activate,

        TimerAction(
            period=20.0,
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
