"""Panda1 cube-only contact-verified pick, lift and return; execution is opt-in."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
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
        executable="cube_pick_lift_return_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "use_sim_time": True,
                "robot": ParameterValue(LaunchConfiguration("robot"), value_type=str),
                "object": ParameterValue(LaunchConfiguration("object"), value_type=str),
                "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
                "lift_height": ParameterValue(LaunchConfiguration("lift_height"), value_type=float),

            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="panda1"),
        DeclareLaunchArgument("object", default_value="cube"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("lift_height", default_value="0.10"),

        validation,
    ])
