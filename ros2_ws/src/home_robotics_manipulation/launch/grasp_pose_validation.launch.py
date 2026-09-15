"""Generate cube grasp poses; optionally execute only the validated pre-grasp."""
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
        executable="grasp_pose_validation",
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
                "execute_pregrasp": ParameterValue(LaunchConfiguration("execute_pregrasp"), value_type=bool),
                "validate_grasp": ParameterValue(LaunchConfiguration("validate_grasp"), value_type=bool),
                "keep_alive": ParameterValue(LaunchConfiguration("keep_alive"), value_type=bool),
                "pre_grasp_distance": ParameterValue(LaunchConfiguration("pre_grasp_distance"), value_type=float),
                "grasp_clearance": ParameterValue(LaunchConfiguration("grasp_clearance"), value_type=float),
                "finger_open_width": ParameterValue(LaunchConfiguration("finger_open_width"), value_type=float),
                "expected_cube_width": ParameterValue(LaunchConfiguration("expected_cube_width"), value_type=float),

            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="panda1"),
        DeclareLaunchArgument("object", default_value="cube"),
        DeclareLaunchArgument("execute_pregrasp", default_value="false"),
        DeclareLaunchArgument("validate_grasp", default_value="true"),
        DeclareLaunchArgument("keep_alive", default_value="true"),
        DeclareLaunchArgument("pre_grasp_distance", default_value="0.10"),
        DeclareLaunchArgument("grasp_clearance", default_value="0.008"),
        DeclareLaunchArgument("finger_open_width", default_value="0.0"),
        DeclareLaunchArgument("expected_cube_width", default_value="0.0"),

        validation,
    ])
