"""
gateway.launch.py — Semantic Gateway full-stack launch (synthetic source).

Starts:
  1. demo_publisher_node  — synthetic LaserScan / IMU / Temperature + static TF
  2. gateway_node         — exports the ROS sensors/actuators as SmartData
                            transducers on the SmartData network (UDP
                            multicast, see docs/wire_format.md) and mirrors
                            every update on /smartdata/* topics.
  3. proxy_node           — announces interests, republishes the responses as
                            ROS messages on <topic>_recovered and turns
                            <topic>_command messages into SmartData commands.

End-to-end fidelity is pinned by `test/test_bus_roundtrip.cpp`; run via
`colcon test --packages-select semantic_gateway`.

Usage:
  ros2 launch semantic_gateway gateway.launch.py
  ros2 launch semantic_gateway gateway.launch.py use_demo_publisher:=false
  ros2 launch semantic_gateway gateway.launch.py enable_proxy:=false
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
            "enable_proxy", default_value="true",
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
            executable="proxy_node",
            name="proxy",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_proxy")),
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),

        LogInfo(msg="Generic Semantic Gateway stack launched. "
                    "Inspect /smartdata/* and *_recovered topics, or sniff the "
                    "SmartData network on 224.1.1.1:5000."),
    ])
