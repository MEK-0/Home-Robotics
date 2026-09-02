"""Focused deterministic runtime-reset tests for the Phase 2 control stack."""
from pathlib import Path

import numpy as np
import pytest

from simulation.arm_control import PandaArmPositionController
from simulation.config_loader import ConfigLoader
from simulation.rail_control import RailTargetController
from simulation.reset_manager import ResetManager
from simulation.scene_builder import SceneBuilder
from simulation.scene_geometry import resolve_object_pose


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def runtime():
    config = ConfigLoader(ROOT / "config").load()
    with SceneBuilder(config).build(headless=True) as simulator:
        controller = RailTargetController(config, simulator)
        arm_controller = PandaArmPositionController(config, simulator)
        reset_manager = ResetManager(
            simulator,
            config=config,
            settle_steps=config.physics["reset"]["settle_steps"],
        )
        reset_manager.add_state_reset_hook(
            lambda _simulator: (
                controller.reset_to_home(), arm_controller.reset_to_home()
            )
        )
        yield config, simulator, controller, reset_manager


def step_control(controller, simulator, steps):
    for _ in range(steps):
        controller.apply_targets()
        simulator.step()


def assert_rail_control_home(config, simulator, controller):
    for robot_id, robot in config.robots.items():
        home = float(robot["rail"]["home_position"])
        joint = robot["rail"]["joint"]
        actuator = f"{robot_id}_rail_actuator"
        assert simulator.joint_position(joint) == pytest.approx(home, abs=5e-6)
        assert simulator.joint_velocity(joint) == pytest.approx(0.0, abs=1e-5)
        assert controller.target(robot_id) == pytest.approx(home)
        assert controller.actuator_targets[robot_id] == pytest.approx(home)
        assert simulator.actuator_control(actuator) == pytest.approx(home)


def test_rail_targets_and_actuator_controls_initialize_at_validated_home(runtime):
    config, simulator, controller, _ = runtime

    assert_rail_control_home(config, simulator, controller)


def test_runtime_reset_during_motion_clears_stale_rail_command(runtime):
    config, simulator, controller, reset_manager = runtime
    assert controller.accept_target("panda1", 0.2)[0]
    step_control(controller, simulator, 300)
    assert simulator.joint_position("panda1_rail_joint") > -0.9

    reset_manager.reset()

    assert_rail_control_home(config, simulator, controller)
    minimum = float(config.scene["shared_rail"]["minimum_carriage_separation"])
    separation = (
        simulator.joint_position("panda2_rail_joint")
        - simulator.joint_position("panda1_rail_joint")
    )
    assert separation >= minimum

    step_control(controller, simulator, 1000)
    assert_rail_control_home(config, simulator, controller)


def test_runtime_reset_restores_robots_grippers_and_dynamic_objects(runtime):
    config, simulator, controller, reset_manager = runtime

    for joint_name, home in config.robot_home_positions.items():
        simulator.set_joint_position(joint_name, float(home) + 0.01)
        simulator.set_joint_velocity(joint_name, 0.2)
    for object_id, obj in config.objects.items():
        if not obj["dynamic"]:
            continue
        simulator.set_free_joint_pose(
            f"{object_id}_free_joint", (0.0, 0.0, 2.0), (1.0, 0.0, 0.0, 0.0)
        )
        simulator.set_free_joint_velocity(
            f"{object_id}_free_joint", (0.2, -0.1, 0.3, 0.4, -0.2, 0.1)
        )
    simulator.forward()

    reset_manager.reset()

    for joint_name, home in config.robot_home_positions.items():
        assert simulator.joint_position(joint_name) == pytest.approx(home)
        assert simulator.joint_velocity(joint_name) == pytest.approx(0.0)
    for object_id, obj in config.objects.items():
        if not obj["dynamic"]:
            continue
        expected_position, expected_orientation = resolve_object_pose(config, obj)
        assert simulator.body_position(object_id) == pytest.approx(expected_position)
        assert simulator.body_orientation(object_id) == pytest.approx(
            expected_orientation
        )
        assert simulator.body_velocity(object_id) == pytest.approx(
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), abs=1e-12
        )

    assert_rail_control_home(config, simulator, controller)


def test_repeated_runtime_reset_is_deterministic(runtime):
    config, simulator, controller, reset_manager = runtime
    snapshots = []

    for target in (-0.2, 0.1, 0.2):
        assert controller.accept_target("panda1", target)[0]
        step_control(controller, simulator, 100)
        reset_manager.reset()
        snapshots.append(
            (
                simulator.time,
                simulator.data.qpos.copy(),
                simulator.data.qvel.copy(),
                simulator.data.ctrl.copy(),
                dict(controller.targets),
                dict(controller.actuator_targets),
            )
        )

    first = snapshots[0]
    for snapshot in snapshots[1:]:
        assert snapshot[0] == first[0] == 0.0
        assert np.array_equal(snapshot[1], first[1])
        assert np.array_equal(snapshot[2], first[2])
        assert np.array_equal(snapshot[3], first[3])
        assert snapshot[4:] == first[4:]
    assert_rail_control_home(config, simulator, controller)
