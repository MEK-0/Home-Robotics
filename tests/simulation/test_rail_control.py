"""Focused Phase 2 tests for dynamic shared-rail position control."""
from pathlib import Path

import pytest

from simulation.config_loader import ConfigLoader
from simulation.rail_control import RailTargetController
from simulation.reset_manager import ResetManager
from simulation.scene_builder import SceneBuilder


PROJECT_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def rail_system():
    config = ConfigLoader(PROJECT_ROOT / "config").load()
    with SceneBuilder(config).build() as simulator:
        yield config, simulator, RailTargetController(config, simulator)


def step_control(controller, simulator, steps=4000):
    positions = []
    for _ in range(steps):
        controller.apply_targets()
        simulator.step()
        positions.append(
            (
                simulator.joint_position("panda1_rail_joint"),
                simulator.joint_position("panda2_rail_joint"),
            )
        )
    return positions


def test_rail_actuator_names_and_joint_mappings(rail_system):
    _, simulator, _ = rail_system

    assert simulator.model.nu == 18
    assert simulator.actuator_exists("panda1_rail_actuator")
    assert simulator.actuator_exists("panda2_rail_actuator")
    assert simulator.actuator_joint("panda1_rail_actuator") == "panda1_rail_joint"
    assert simulator.actuator_joint("panda2_rail_actuator") == "panda2_rail_joint"


@pytest.mark.parametrize(
    ("robot_id", "target"),
    (("panda1", 0.1), ("panda2", 1.2)),
)
def test_valid_target_causes_dynamic_motion_and_converges(
    rail_system, robot_id, target
):
    config, simulator, controller = rail_system
    joint = config.robots[robot_id]["rail"]["joint"]
    initial = simulator.joint_position(joint)

    accepted, reason = controller.accept_target(robot_id, target)
    positions = step_control(controller, simulator)

    assert accepted, reason
    assert simulator.joint_position(joint) != pytest.approx(initial, abs=0.05)
    assert simulator.joint_position(joint) == pytest.approx(target, abs=0.015)
    assert all(
        config.robots["panda1"]["rail"]["lower_limit"] <= q1
        <= config.robots["panda1"]["rail"]["upper_limit"]
        and config.robots["panda2"]["rail"]["lower_limit"] <= q2
        <= config.robots["panda2"]["rail"]["upper_limit"]
        for q1, q2 in positions
    )


def test_minimum_separation_is_preserved_during_safe_boundary_motion(rail_system):
    config, simulator, controller = rail_system
    minimum = config.scene["shared_rail"]["minimum_carriage_separation"]

    accepted, reason = controller.accept_target("panda1", 0.2)
    positions = step_control(controller, simulator)

    assert accepted, reason
    assert min(q2 - q1 for q1, q2 in positions) >= minimum


def test_crossing_command_is_rejected_and_previous_target_is_kept(rail_system):
    _, _, controller = rail_system
    assert controller.accept_target("panda1", 0.1)[0]
    previous = controller.target("panda2")

    accepted, reason = controller.accept_target("panda2", 0.5)

    assert not accepted
    assert "minimum separation" in reason
    assert controller.target("panda2") == previous


def test_out_of_limit_and_nonfinite_targets_are_rejected(rail_system):
    _, _, controller = rail_system

    assert not controller.accept_target("panda1", -2.0)[0]
    assert not controller.accept_target("panda2", float("nan"))[0]


def test_reset_still_returns_both_rails_to_validated_home(rail_system):
    config, simulator, controller = rail_system
    assert controller.accept_target("panda1", 0.1)[0]
    step_control(controller, simulator, steps=1000)

    ResetManager(simulator, config=config).reset()

    for robot in config.robots.values():
        rail = robot["rail"]
        assert simulator.joint_position(rail["joint"]) == pytest.approx(
            rail["home_position"]
        )
        assert simulator.joint_velocity(rail["joint"]) == pytest.approx(0.0)
