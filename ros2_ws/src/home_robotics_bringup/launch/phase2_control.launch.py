"""Launch the complete Phase 2 dual-Panda ros2_control stack."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = Path(get_package_share_directory("home_robotics_description"))
    bringup_share = Path(get_package_share_directory("home_robotics_bringup"))
    robot_description = ParameterValue(
        Command(["xacro ", str(description_share / "urdf/home_robotics.urdf.xacro")]),
        value_type=str,
    )
    common = {"robot_description": robot_description, "use_sim_time": True}
    runtime = Node(
        package="home_robotics_bringup",
        executable="mujoco_runtime",
        name="mujoco_runtime",
        parameters=[{"use_viewer": LaunchConfiguration("use_viewer")}],
        remappings=[("/joint_states", "/mujoco/joint_states")],
        output="screen",
    )
    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[common, str(bringup_share / "config/controllers.yaml")],
        output="screen",
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[common],
        output="screen",
    )
    controller_startup = Node(
        package="home_robotics_bringup",
        executable="controller_startup",
        parameters=[{"controller_manager": "/controller_manager"}],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_viewer", default_value="false"),
        runtime, control, state_publisher, controller_startup,
    ])
