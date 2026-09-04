"""Load the authoritative static MuJoCo environment into MoveIt's PlanningScene."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = Path(get_package_share_directory("home_robotics_moveit_config"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene_config",
            default_value=str(moveit_share / "config/scene.yaml"),
        ),
        Node(
            package="home_robotics_moveit_config",
            executable="planning_scene_environment",
            output="screen",
            parameters=[{
                "scene_config": LaunchConfiguration("scene_config"),
                "use_sim_time": True,
            }],
        ),
    ])
