"""Headless tests for Phase 1B.2 dual Franka Panda model integration."""
from __future__ import annotations
import math
from pathlib import Path
import pytest
from simulation.config_loader import ConfigLoader
from simulation.reset_manager import ResetManager
from simulation.scene_builder import SceneBuilder
from simulation.validation import validate_no_initial_penetration, validate_robot_structure, validate_state

ROOT = Path(__file__).resolve().parents[2]
KINEMATIC_TOLERANCE = 1e-9
TEST_RAIL_DISPLACEMENT = 0.20

def _config():
    return ConfigLoader(ROOT / "config").load()

def test_two_panda_models_exist():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert all(simulator.body_exists(robot["arm"]["link0_frame"]) for robot in config.robots.values())

def test_seven_arm_joints_per_panda():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        for robot in config.robots.values():
            assert len(robot["arm"]["joint_names"]) == 7
            assert all(simulator.joint_exists(name) for name in robot["arm"]["joint_names"])

def test_two_franka_hands_and_finger_pairs_exist():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        for robot in config.robots.values():
            assert simulator.body_exists(robot["gripper"]["hand_frame"])
            assert len(robot["gripper"]["finger_joint_names"]) == 2
            assert all(simulator.joint_exists(name) for name in robot["gripper"]["finger_joint_names"])

def test_tcp_frames_exist_and_have_finite_home_fk():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        for robot in config.robots.values():
            tcp = robot["gripper"]["tcp_frame"]
            assert simulator.site_exists(tcp)
            position = simulator.site_position(tcp)
            assert all(math.isfinite(value) for value in position)
            assert position[2] >= robot["home_validation"]["minimum_tcp_height"]

def test_robot_joint_limits_and_homes_are_valid():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        for robot in config.robots.values():
            names = [*robot["arm"]["joint_names"], *robot["gripper"]["finger_joint_names"]]
            homes = [*robot["arm"]["home_joints"], *robot["gripper"]["home_joints"]]
            for name, home in zip(names, homes, strict=True):
                lower, upper = simulator.joint_range(name)
                assert lower < upper
                assert lower <= home <= upper
                assert simulator.joint_position(name) == pytest.approx(home, abs=KINEMATIC_TOLERANCE)

def test_robot_home_has_no_illegal_collision():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        validate_robot_structure(config, simulator)
        validate_no_initial_penetration(simulator)
        assert simulator.penetrating_contacts() == []

def test_panda_mounts_and_tcps_follow_carriages():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        for robot in config.robots.values():
            rail = robot["rail"]
            tracked = (rail["carriage_frame"], robot["arm"]["base_frame"], robot["arm"]["link0_frame"])
            before = {name: simulator.body_position(name) for name in tracked}
            tcp_before = simulator.site_position(robot["gripper"]["tcp_frame"])
            simulator.set_joint_position(rail["joint"], rail["home_position"] + TEST_RAIL_DISPLACEMENT)
            simulator.forward()
            for name in tracked:
                after = simulator.body_position(name)
                assert after[0] - before[name][0] == pytest.approx(TEST_RAIL_DISPLACEMENT, abs=KINEMATIC_TOLERANCE)
                assert after[1:] == pytest.approx(before[name][1:], abs=KINEMATIC_TOLERANCE)
            tcp_after = simulator.site_position(robot["gripper"]["tcp_frame"])
            assert tcp_after[0] - tcp_before[0] == pytest.approx(TEST_RAIL_DISPLACEMENT, abs=KINEMATIC_TOLERANCE)
            assert tcp_after[1:] == pytest.approx(tcp_before[1:], abs=KINEMATIC_TOLERANCE)
            simulator.set_joint_position(rail["joint"], rail["home_position"])
            simulator.forward()

def test_robot_reset_is_deterministic_and_clears_all_velocities():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        reset = ResetManager(simulator, config=config)
        expected_positions = config.reset_joint_positions
        expected_tcp = {robot_id: simulator.site_position(robot["gripper"]["tcp_frame"]) for robot_id, robot in config.robots.items()}
        snapshots = []
        for _ in range(3):
            for joint_name, home in expected_positions.items():
                lower, upper = simulator.joint_range(joint_name)
                offset = min(0.01, (upper - lower) / 10.0)
                simulator.set_joint_position(joint_name, min(upper, max(lower, home - offset)))
                simulator.set_joint_velocity(joint_name, 0.2)
            simulator.forward()
            reset.reset()
            snapshots.append(tuple(simulator.joint_position(name) for name in expected_positions))
            for name, home in expected_positions.items():
                assert simulator.joint_position(name) == pytest.approx(home, abs=KINEMATIC_TOLERANCE)
                assert simulator.joint_velocity(name) == pytest.approx(0.0, abs=KINEMATIC_TOLERANCE)
            for robot_id, robot in config.robots.items():
                assert simulator.site_position(robot["gripper"]["tcp_frame"]) == pytest.approx(expected_tcp[robot_id], abs=KINEMATIC_TOLERANCE)
            validate_state(simulator)
            validate_no_initial_penetration(simulator)
        assert snapshots[0] == snapshots[1] == snapshots[2]

def test_one_hundred_full_robot_resets_are_repeatable():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        reset = ResetManager(simulator, config=config)
        expected_positions = config.reset_joint_positions
        expected_tcp = {robot_id: simulator.site_position(robot["gripper"]["tcp_frame"]) for robot_id, robot in config.robots.items()}
        for _ in range(100):
            for joint_name in expected_positions:
                simulator.set_joint_velocity(joint_name, 0.1)
            simulator.step(1)
            reset.reset()
            for joint_name, home in expected_positions.items():
                assert simulator.joint_position(joint_name) == pytest.approx(home, abs=KINEMATIC_TOLERANCE)
                assert simulator.joint_velocity(joint_name) == pytest.approx(0.0, abs=KINEMATIC_TOLERANCE)
            for robot_id, robot in config.robots.items():
                tcp = simulator.site_position(robot["gripper"]["tcp_frame"])
                assert tcp == pytest.approx(expected_tcp[robot_id], abs=KINEMATIC_TOLERANCE)
                assert all(math.isfinite(value) for value in tcp)
            validate_no_initial_penetration(simulator)
            validate_state(simulator)

def test_panda2_is_physical_and_low_level_control_active():
    config = _config()
    assert config.robots["panda2"]["active"] is True
    with SceneBuilder(config).build(headless=True) as simulator:
        assert simulator.body_exists("panda2_link0")
        assert simulator.body_exists("panda2_hand")
        assert simulator.site_exists("panda2_tcp")

def test_model_integration_adds_rail_and_panda1_arm_actuators():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert simulator.model.nu == 18
        assert simulator.actuator_joint("panda1_rail_actuator") == "panda1_rail_joint"
        assert simulator.actuator_joint("panda2_rail_actuator") == "panda2_rail_joint"
