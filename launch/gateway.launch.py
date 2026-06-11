"""
gateway.launch.py — Semantic Gateway full-stack launch (synthetic source).

Starts:
  1. demo_publisher_node  — synthetic LaserScan / IMU / Temperature + static TF
  2. gateway_node         — generic ROS → SmartData translator (config-driven)
  3. validator_node       — fidelity & expiry validator (config-driven)

Both gateway_node and validator_node read `config/sensors.yaml`. To add or
change a sensor, edit that YAML — no code change needed.

Usage:
  ros2 launch semantic_gateway gateway.launch.py
  ros2 launch semantic_gateway gateway.launch.py use_demo_publisher:=false
  ros2 launch semantic_gateway gateway.launch.py config_path:=/path/to/other.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("semantic_gateway"),
        "config", "sensors.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_demo_publisher", default_value="true",
            description="Launch the synthetic sensor publisher (disable on real robots)"),
        DeclareLaunchArgument(
            "config_path", default_value=default_config,
            description="Path to the sensors.yaml configuration file"),
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
                "config_path":      LaunchConfiguration("config_path"),
                "target_frame":     LaunchConfiguration("target_frame"),
                "stats_interval_s": 5,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),

        Node(
            package="semantic_gateway",
            executable="validator_node",
            name="smartdata_validator",
            output="screen",
            parameters=[{
                "config_path":        LaunchConfiguration("config_path"),
                "target_frame":       LaunchConfiguration("target_frame"),
                "report_interval_s":  5,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),

        LogInfo(msg="Generic Semantic Gateway stack launched. "
                    "Inspect /smartdata/* topics and the validator report."),
    ])
