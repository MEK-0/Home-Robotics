"""Run an offline cube collision rejection test; no robot commands."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description = Path(get_package_share_directory("home_robotics_description"))
    moveit_config = (
        MoveItConfigsBuilder("home_robotics", package_name="home_robotics_moveit_config")
        .robot_description(file_path=description / "urdf/home_robotics.urdf.xacro")
        .robot_description_semantic(file_path="config/home_robotics.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"], load_all=False)
        .to_moveit_configs()
    )
    validation = Node(
        package="home_robotics_manipulation",
        executable="cube_collision_validation",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "use_sim_time": True,
            },
        ],
    )
    return LaunchDescription([
        validation,
    ])
