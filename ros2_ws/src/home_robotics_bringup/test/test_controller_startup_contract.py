"""Controller startup ownership and ordering contract tests."""

from pathlib import Path

from home_robotics_bringup.controller_startup import CONTROLLERS


LAUNCH = Path(__file__).resolve().parents[1] / 'launch/phase2_control.launch.py'


def test_controller_startup_order_is_explicit():
    assert CONTROLLERS == (
        'joint_state_broadcaster',
        'panda1_trajectory_controller',
        'panda2_trajectory_controller',
        'panda1_gripper_controller',
        'panda2_gripper_controller',
    )


def test_launch_has_one_controller_lifecycle_owner():
    launch_text = LAUNCH.read_text(encoding='utf-8')
    assert 'executable="controller_startup"' in launch_text
    assert 'executable="spawner"' not in launch_text
