from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    grid_topic = LaunchConfiguration("grid_topic")
    driver_backend = LaunchConfiguration("driver_backend")
    habitat_socket_path = LaunchConfiguration("habitat_socket_path")
    publish_hz = LaunchConfiguration("publish_hz")

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("explorer_bridge"),
                "launch",
                "habitat_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "driver_backend": driver_backend,
            "habitat_socket_path": habitat_socket_path,
            "publish_hz": publish_hz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("grid_topic", default_value="/grid_map"),
        DeclareLaunchArgument("driver_backend", default_value="habitat"),
        DeclareLaunchArgument("habitat_socket_path", default_value="/tmp/habitat_engine.sock"),
        DeclareLaunchArgument("publish_hz", default_value="15.0"),

        bridge_launch,

        Node(
            package="explorer_bridge",
            executable="habitat_map_node",
            name="habitat_map_node",
            parameters=[{
                "habitat_socket_path": habitat_socket_path,
                "grid_topic": grid_topic,
                "map_frame": map_frame,
                "publish_hz": 1.0,
            }],
            output="screen",
        ),
        Node(
            package="explorer_mission",
            executable="frontiers_node",
            name="frontiers",
            parameters=[{
                "map_frame": map_frame,
                "base_frame": base_frame,
                "grid_topic": grid_topic,
            }],
            output="screen",
        ),
        Node(
            package="explorer_mission",
            executable="graph_node",
            name="graph_node",
            parameters=[{"frame_id": map_frame}],
            output="screen",
        ),
        Node(
            package="explorer_mission",
            executable="actions_node",
            name="actions",
            parameters=[{
                "map_frame": map_frame,
                "base_frame": base_frame,
                "image_topic": "/image_data",
                "rotate_steps": 36,
            }],
            output="screen",
        ),
        Node(
            package="explorer_mission",
            executable="frontier_vlm_client_node",
            name="frontier_vlm_client",
            output="screen",
        ),
        Node(
            package="explorer_mission",
            executable="explore_node",
            name="explore",
            parameters=[{
                "map_frame": map_frame,
                "base_frame": base_frame,
            }],
            output="screen",
        ),
        Node(
            package="explorer_mission",
            executable="maprender_node",
            name="map_renderer",
            output="screen",
        ),
        Node(
            package="explorer_mission",
            executable="vlm_node",
            name="vlm_server",
            output="screen",
        ),
    ])
