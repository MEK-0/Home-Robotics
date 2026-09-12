"""Run one guarded joint-space validation through MoveGroupInterface."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
        package="home_robotics_moveit_config",
        executable="dual_robot_collision_validation",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "use_sim_time": True,
                "mode": LaunchConfiguration("mode"),
                "execute": LaunchConfiguration("execute"),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="current"),
        DeclareLaunchArgument("execute", default_value="false"),
        validation,
    ])
