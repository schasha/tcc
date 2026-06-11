"""
gazebo_gateway.launch.py — Semantic Gateway with TurtleBot3 + Gazebo Classic

Moving-robot scenario for the proposta §3.3 ("robô em movimento"):

  1. Gazebo (gzserver, optionally gzclient) + world + spawn TurtleBot3 burger
  2. gateway_node      — generic ROS → SmartData, driven by Gazebo YAML config
  3. validator_node    — fidelity & integrity report
  4. cmd_vel publisher — drives the robot in a slow circle so TF is dynamic

Usage:
  export TURTLEBOT3_MODEL=burger
  ros2 launch semantic_gateway gazebo_gateway.launch.py            # with GUI
  ros2 launch semantic_gateway gazebo_gateway.launch.py gui:=false # headless
  ros2 launch semantic_gateway gazebo_gateway.launch.py drive:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    pkg_share        = get_package_share_directory("semantic_gateway")
    default_config   = os.path.join(pkg_share, "config", "gazebo_sensors.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(tb3_gazebo_share, "worlds", "empty_world.world"),
            description=("Gazebo world file. Default empty_world avoids fetching online "
                         "model assets. Set to turtlebot3_world.world for the full scenario "
                         "after the model cache is warm.")
        ),
        DeclareLaunchArgument(
            "gui",          default_value="true",
            description="Launch gzclient (Gazebo GUI). Set to false for headless runs"),
        DeclareLaunchArgument(
            "drive",        default_value="true",
            description="Publish a constant cmd_vel so the robot moves"),
        DeclareLaunchArgument(
            "config_path",  default_value=default_config,
            description="Path to sensors YAML config"),
        DeclareLaunchArgument(
            "target_frame", default_value="odom",
            description="Frame in which the gateway expresses sensor origins"),
        DeclareLaunchArgument(
            "x_pose",       default_value="-2.0"),
        DeclareLaunchArgument(
            "y_pose",       default_value="-0.5"),
        DeclareLaunchArgument(
            "log_level",    default_value="info"),

        # gzserver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
            ),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),

        # gzclient
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),

        # robot_state_publisher (URDF + dynamic TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_share, "launch", "robot_state_publisher.launch.py")
            ),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),

        # spawn TurtleBot3 
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_burger",
            output="screen",
            arguments=[
                "-entity", "burger",
                "-file", os.path.join(
                    tb3_gazebo_share, "models", "turtlebot3_burger", "model.sdf"),
                "-x", LaunchConfiguration("x_pose"),
                "-y", LaunchConfiguration("y_pose"),
                "-z", "0.01",
                "-timeout", "120",
            ],
        ),

        # semantic gateway
        Node(
            package="semantic_gateway",
            executable="gateway_node",
            name="semantic_gateway",
            output="screen",
            parameters=[{
                "use_sim_time":     True,
                "config_path":      LaunchConfiguration("config_path"),
                "target_frame":     LaunchConfiguration("target_frame"),
                "stats_interval_s": 5,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),

        # validator
        Node(
            package="semantic_gateway",
            executable="validator_node",
            name="smartdata_validator",
            output="screen",
            parameters=[{
                "use_sim_time":      True,
                "config_path":       LaunchConfiguration("config_path"),
                "target_frame":      LaunchConfiguration("target_frame"),
                "report_interval_s": 5,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),

        # cmd_vel publisher
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("drive")),
            cmd=[
                "ros2", "topic", "pub", "--rate", "10", "/cmd_vel",
                "geometry_msgs/msg/Twist",
                "{linear: {x: 0.10, y: 0.0, z: 0.0}, "
                "angular: {x: 0.0, y: 0.0, z: 0.30}}",
            ],
            output="log",
        ),

        LogInfo(msg=(
            "Generic Semantic Gateway stack launched on Gazebo. "
            "Inspect /smartdata/* and the validator report."
        )),
    ])
