"""
gateway.launch.py — Semantic Gateway full-stack launch (synthetic source).

Starts:
  1. demo_publisher_node  — synthetic LaserScan / IMU / Temperature + static TF
  2. gateway_node         — ROS → SmartData translator. Broadcasts SmartData on
                            UDP multicast (see docs/wire_format.md) and mirrors
                            on /smartdata/* topics for debug.
  3. sd_to_ros_node       — listens to the bus and republishes ROS messages on
                            <topic>_recovered. Useful for end-to-end sanity.

Fidelity validation lives in `test/test_validator.cpp` and
`test/test_bus_roundtrip.cpp`; run via
`colcon test --packages-select semantic_gateway`.

Usage:
  ros2 launch semantic_gateway gateway.launch.py
  ros2 launch semantic_gateway gateway.launch.py use_demo_publisher:=false
  ros2 launch semantic_gateway gateway.launch.py enable_sd_to_ros:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_demo_publisher", default_value="true",
            description="Launch the synthetic sensor publisher (disable on real robots)"),
        DeclareLaunchArgument(
            "enable_sd_to_ros", default_value="true",
            description="Launch the bus → ROS recovery node"),
        DeclareLaunchArgument(
            "target_frame", default_value="map",
            description="World frame used for SmartData origin coordinates"),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="ROS log level: debug | info | warn | error"),

        Node(
            package="semantic_gateway",
            executable="demo_publisher_node",
            name="demo_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_demo_publisher")),
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),

        Node(
            package="semantic_gateway",
            executable="gateway_node",
            name="semantic_gateway",
            output="screen",
            parameters=[{
                "target_frame":      LaunchConfiguration("target_frame"),
                "stats_interval_s":  5,
                "enable_ros_mirror": True,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),

        Node(
            package="semantic_gateway",
            executable="sd_to_ros_node",
            name="sd_to_ros",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_sd_to_ros")),
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),

        LogInfo(msg="Generic Semantic Gateway stack launched. "
                    "Inspect /smartdata/* topics or sniff the UDP multicast "
                    "bus on 224.1.1.1:5000."),
    ])
