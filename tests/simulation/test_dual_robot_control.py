"""Focused tests for dual Panda arm and Franka Hand control."""
from pathlib import Path

import pytest

from simulation.arm_control import PandaArmPositionController
from simulation.config_loader import ConfigLoader
from simulation.gripper_control import GripperWidthController
from simulation.rail_control import RailTargetController
from simulation.reset_manager import ResetManager
from simulation.scene_builder import SceneBuilder


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def system():
    config = ConfigLoader(ROOT / "config").load()
    with SceneBuilder(config).build(headless=True) as simulator:
        rail = RailTargetController(config, simulator)
        arms = {
            robot_id: PandaArmPositionController(config, simulator, robot_id)
            for robot_id in ("panda1", "panda2")
        }
        grippers = {
            robot_id: GripperWidthController(config, simulator, robot_id)
            for robot_id in ("panda1", "panda2")
        }
        reset = ResetManager(simulator, config=config)
        reset.add_state_reset_hook(
            lambda _simulator: (
                rail.reset_to_home(),
                *(controller.reset_to_home() for controller in arms.values()),
                *(controller.reset_to_home() for controller in grippers.values()),
            )
        )
        yield config, simulator, rail, arms, grippers, reset


def step(system, count):
    _, simulator, rail, arms, grippers, _ = system
    for _ in range(count):
        rail.apply_targets()
        for controller in arms.values():
            controller.apply_targets()
        for controller in grippers.values():
            controller.apply_target()
        simulator.step()


def test_all_control_actuators_exist_with_expected_count(system):
    _, simulator, _, arms, _, _ = system
    assert simulator.model.nu == 18
    for robot_id in ("panda1", "panda2"):
        for index, joint_name in enumerate(arms[robot_id].joint_names, start=1):
            actuator = f"{robot_id}_joint{index}_actuator"
            assert simulator.actuator_exists(actuator)
            assert simulator.actuator_joint(actuator) == joint_name
        assert simulator.actuator_exists(f"{robot_id}_gripper_actuator")


def test_panda2_arm_moves_without_changing_panda1_targets(system):
    _, simulator, _, arms, _, _ = system
    target = [0.05, -0.05, 0.05, -1.50, 0.02, 1.50, -0.75]
    panda1_before = dict(arms["panda1"].targets)
    assert arms["panda2"].accept_command(arms["panda2"].joint_names, target)[0]
    step(system, 2500)
    assert simulator.joint_position("panda2_joint1") == pytest.approx(
        target[0], abs=0.02
    )
    assert arms["panda1"].targets == panda1_before


@pytest.mark.parametrize(("robot_id", "width"), (("panda1", 0.04), ("panda2", 0.02)))
def test_gripper_width_command_moves_both_coupled_fingers(system, robot_id, width):
    _, simulator, _, _, grippers, _ = system
    assert grippers[robot_id].accept_target(width)[0]
    step(system, 2000)
    expected = width / 2.0
    assert simulator.joint_position(
        f"{robot_id}_finger_joint1"
    ) == pytest.approx(expected, abs=0.001)
    assert simulator.joint_position(
        f"{robot_id}_finger_joint2"
    ) == pytest.approx(expected, abs=0.001)


def test_invalid_gripper_width_is_rejected_atomically(system):
    _, _, _, _, grippers, _ = system
    controller = grippers["panda1"]
    assert controller.accept_target(0.04)[0]
    previous = controller.target_width
    assert not controller.accept_target(0.09)[0]
    assert controller.target_width == previous


def test_reset_synchronizes_both_arms_and_grippers(system):
    config, simulator, _, arms, grippers, reset = system
    target = [0.05, -0.05, 0.05, -1.50, 0.02, 1.50, -0.75]
    for controller in arms.values():
        assert controller.accept_command(controller.joint_names, target)[0]
    for controller in grippers.values():
        assert controller.accept_target(0.02)[0]
    step(system, 300)
    reset.reset()
    step(system, 500)
    for robot_id, controller in arms.items():
        homes = config.robots[robot_id]["arm"]["home_joints"]
        for name, home in zip(controller.joint_names, homes, strict=True):
            assert simulator.joint_position(name) == pytest.approx(home, abs=0.005)
            assert controller.targets[name] == pytest.approx(home)
    for robot_id, controller in grippers.items():
        assert controller.target_width == pytest.approx(0.08)
        assert controller.actuator_width == pytest.approx(0.08)
        assert simulator.joint_position(
            f"{robot_id}_finger_joint1"
        ) == pytest.approx(0.04, abs=0.001)
