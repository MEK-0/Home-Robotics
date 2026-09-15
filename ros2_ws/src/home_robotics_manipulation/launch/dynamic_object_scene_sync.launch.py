"""Forward the authoritative runtime object poses into MoveIt world geometry."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package="home_robotics_manipulation", executable="dynamic_object_scene_sync",
        output="screen", parameters=[{"use_sim_time": True, "sync_rate": 20.0}],
    )])
