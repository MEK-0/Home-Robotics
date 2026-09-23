from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false"),
        Node(package="home_robotics_task_executor", executable="task_executor",
             name="task_executor", output="screen",
             parameters=[{"use_sim_time": True, "execute": LaunchConfiguration("execute")}]),
    ])
