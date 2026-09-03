"""Standalone robot-description publisher."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("home_robotics_description"))
    description = ParameterValue(
        Command(["xacro ", str(share / "urdf/home_robotics.urdf.xacro")]),
        value_type=str,
    )
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": True}],
        )
    ])
