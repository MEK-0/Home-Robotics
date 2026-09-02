"""Focused Phase 2 tests for Panda 1 dynamic arm position control."""
from pathlib import Path

import pytest

from simulation.arm_control import PandaArmPositionController
from simulation.config_loader import ConfigLoader
from simulation.rail_control import RailTargetController
from simulation.reset_manager import ResetManager
from simulation.scene_builder import SceneBuilder


ROOT = Path(__file__).resolve().parents[2]
SAFE_TARGET = [0.10, -0.10, 0.10, -1.45, 0.05, 1.45, -0.70]


@pytest.fixture
def control_system():
    config = ConfigLoader(ROOT / "config").load()
    with SceneBuilder(config).build(headless=True) as simulator:
        rail = RailTargetController(config, simulator)
        arm = PandaArmPositionController(config, simulator)
        reset = ResetManager(simulator, config=config)
        reset.add_state_reset_hook(
            lambda _simulator: (rail.reset_to_home(), arm.reset_to_home())
        )
        yield config, simulator, rail, arm, reset


def step_control(rail, arm, simulator, steps):
    for _ in range(steps):
        rail.apply_targets()
        arm.apply_targets()
        simulator.step()


def test_panda1_arm_actuator_names_mappings_and_home_controls(control_system):
    config, simulator, _, arm, _ = control_system
    homes = config.robots["panda1"]["arm"]["home_joints"]

    assert simulator.model.nu == 18
    for index, (joint_name, home) in enumerate(
        zip(arm.joint_names, homes, strict=True), start=1
    ):
        actuator = f"panda1_joint{index}_actuator"
        assert simulator.actuator_exists(actuator)
        assert simulator.actuator_joint(actuator) == joint_name
        assert simulator.actuator_control(actuator) == pytest.approx(home)
        assert arm.targets[joint_name] == pytest.approx(home)
        assert arm.actuator_targets[joint_name] == pytest.approx(home)


def test_valid_arm_command_is_atomic_rate_limited_and_moves_dynamically(
    control_system,
):
    _, simulator, rail, arm, _ = control_system
    initial = [simulator.joint_position(name) for name in arm.joint_names]

    accepted, reason = arm.accept_command(arm.joint_names, SAFE_TARGET)
    arm.apply_targets()
    first_setpoints = dict(arm.actuator_targets)
    simulator.step()
    first_actual = [simulator.joint_position(name) for name in arm.joint_names]

    assert accepted, reason
    maximum_step = 0.25 * 0.002
    for name, home in zip(arm.joint_names, initial, strict=True):
        assert abs(first_setpoints[name] - home) <= maximum_step + 1e-12
    assert max(
        abs(actual - target)
        for actual, target in zip(first_actual, SAFE_TARGET, strict=True)
    ) > 0.05
    assert max(
        abs(actual - home)
        for actual, home in zip(first_actual, initial, strict=True)
    ) < 0.001

    step_control(rail, arm, simulator, 4000)
    actual = [simulator.joint_position(name) for name in arm.joint_names]
    assert max(
        abs(position - target)
        for position, target in zip(actual, SAFE_TARGET, strict=True)
    ) < 0.03
    assert max(
        abs(position - home)
        for position, home in zip(actual, initial, strict=True)
    ) > 0.05


@pytest.mark.parametrize(
    ("names", "positions"),
    (
        (
            [
                "panda1_joint1",
                "panda1_joint2",
                "panda1_joint3",
                "panda1_joint4",
                "panda1_joint5",
                "panda1_joint6",
                "unknown_joint",
            ],
            SAFE_TARGET,
        ),
        (
            [
                "panda1_joint1",
                "panda1_joint2",
                "panda1_joint3",
                "panda1_joint4",
                "panda1_joint5",
                "panda1_joint6",
            ],
            SAFE_TARGET[:6],
        ),
        (
            [
                "panda1_joint1",
                "panda1_joint1",
                "panda1_joint3",
                "panda1_joint4",
                "panda1_joint5",
                "panda1_joint6",
                "panda1_joint7",
            ],
            SAFE_TARGET,
        ),
        (
            [
                "panda1_joint1",
                "panda1_joint2",
                "panda1_joint3",
                "panda1_joint4",
                "panda1_joint5",
                "panda1_joint6",
                "panda1_joint7",
            ],
            SAFE_TARGET[:6],
        ),
        (
            [
                "panda1_joint1",
                "panda1_joint2",
                "panda1_joint3",
                "panda1_joint4",
                "panda1_joint5",
                "panda1_joint6",
                "panda1_joint7",
            ],
            [float("nan"), *SAFE_TARGET[1:]],
        ),
        (
            [
                "panda1_joint1",
                "panda1_joint2",
                "panda1_joint3",
                "panda1_joint4",
                "panda1_joint5",
                "panda1_joint6",
                "panda1_joint7",
            ],
            [3.0, *SAFE_TARGET[1:]],
        ),
    ),
)
def test_invalid_arm_command_is_rejected_without_partial_application(
    control_system, names, positions
):
    _, _, _, arm, _ = control_system
    assert arm.accept_command(arm.joint_names, SAFE_TARGET)[0]
    previous = dict(arm.targets)

    accepted, reason = arm.accept_command(names, positions)

    assert not accepted
    assert reason
    assert arm.targets == previous


def test_arm_and_rail_control_coexist_without_actuator_index_overlap(control_system):
    _, simulator, rail, arm, _ = control_system
    assert rail.accept_target("panda1", -0.5)[0]
    assert arm.accept_command(arm.joint_names, SAFE_TARGET)[0]

    step_control(rail, arm, simulator, 2500)

    assert simulator.joint_position("panda1_rail_joint") == pytest.approx(
        -0.5, abs=0.02
    )
    assert simulator.joint_position("panda2_rail_joint") == pytest.approx(
        0.9, abs=0.005
    )
    assert simulator.joint_position("panda1_joint1") == pytest.approx(
        SAFE_TARGET[0], abs=0.03
    )
    assert simulator.actuator_control("panda1_rail_actuator") == pytest.approx(
        -0.5
    )
    assert simulator.actuator_control("panda1_joint1_actuator") == pytest.approx(
        SAFE_TARGET[0]
    )


def test_runtime_reset_during_arm_motion_clears_old_arm_target(control_system):
    config, simulator, rail, arm, reset = control_system
    homes = dict(
        zip(
            arm.joint_names,
            map(float, config.robots["panda1"]["arm"]["home_joints"]),
            strict=True,
        )
    )
    assert arm.accept_command(arm.joint_names, SAFE_TARGET)[0]
    step_control(rail, arm, simulator, 300)
    assert any(
        abs(simulator.joint_position(name) - home) > 0.01
        for name, home in homes.items()
    )

    reset.reset()

    for index, name in enumerate(arm.joint_names, start=1):
        assert simulator.joint_position(name) == pytest.approx(homes[name])
        assert simulator.joint_velocity(name) == pytest.approx(0.0)
        assert arm.targets[name] == pytest.approx(homes[name])
        assert arm.actuator_targets[name] == pytest.approx(homes[name])
        assert simulator.actuator_control(
            f"panda1_joint{index}_actuator"
        ) == pytest.approx(homes[name])

    step_control(rail, arm, simulator, 1000)
    for name, home in homes.items():
        assert simulator.joint_position(name) == pytest.approx(home, abs=0.005)
