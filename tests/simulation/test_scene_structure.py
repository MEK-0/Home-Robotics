"""Headless validation of the Phase 1B physical scene skeleton."""
from __future__ import annotations
from pathlib import Path
import pytest
from simulation.config_loader import ConfigLoader, SURFACE_IDS
from simulation.reset_manager import ResetManager
from simulation.scene_builder import SceneBuilder
from simulation.validation import validate_no_initial_penetration, validate_simulation

ROOT = Path(__file__).resolve().parents[2]
KINEMATIC_TOLERANCE = 1e-9  # Exact primitive/joint construction should agree to numerical precision.
TEST_RAIL_DISPLACEMENT = 0.25

def _config():
    return ConfigLoader(ROOT / "config").load()

def test_six_work_surfaces_exist():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert tuple(config.scene["surfaces"]) == SURFACE_IDS
        assert all(simulator.body_exists(name) for name in SURFACE_IDS)

def test_two_rails_and_carriages_exist():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert all(simulator.body_exists(name) for name in ("panda1_rail", "panda2_rail"))
        assert all(simulator.body_exists(name) for name in ("panda1_carriage", "panda2_carriage"))

def test_rail_joint_ids_exist():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert all(simulator.joint_exists(robot["rail"]["joint"]) for robot in config.robots.values())

def test_rail_home_inside_limits():
    config = _config()
    for robot in config.robots.values():
        rail = robot["rail"]
        assert rail["lower_limit"] <= rail["home_position"] <= rail["upper_limit"]

def test_positive_rail_motion_moves_carriage_and_mount_positive_x():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        for robot in config.robots.values():
            rail = robot["rail"]
            carriage = rail["carriage_frame"]
            mount = robot["arm"]["base_frame"]
            carriage_before = simulator.body_position(carriage)
            mount_before = simulator.body_position(mount)
            simulator.set_joint_position(rail["joint"], rail["home_position"] + TEST_RAIL_DISPLACEMENT)
            simulator.forward()
            carriage_after = simulator.body_position(carriage)
            mount_after = simulator.body_position(mount)
            assert carriage_after[0] - carriage_before[0] == pytest.approx(TEST_RAIL_DISPLACEMENT, abs=KINEMATIC_TOLERANCE)
            assert mount_after[0] - mount_before[0] == pytest.approx(TEST_RAIL_DISPLACEMENT, abs=KINEMATIC_TOLERANCE)
            assert carriage_after[1:] == pytest.approx(carriage_before[1:], abs=KINEMATIC_TOLERANCE)
            assert mount_after[1:] == pytest.approx(mount_before[1:], abs=KINEMATIC_TOLERANCE)
            simulator.set_joint_position(rail["joint"], rail["home_position"])
            simulator.forward()

def test_reset_restores_rail_home_and_clears_velocity():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        reset = ResetManager(simulator, config=config)
        for robot in config.robots.values():
            rail = robot["rail"]
            simulator.set_joint_position(rail["joint"], 0.5)
            simulator.set_joint_velocity(rail["joint"], 0.3)
        simulator.forward()
        reset.reset()
        for robot in config.robots.values():
            rail = robot["rail"]
            assert simulator.joint_position(rail["joint"]) == pytest.approx(rail["home_position"], abs=KINEMATIC_TOLERANCE)
            assert simulator.joint_velocity(rail["joint"]) == pytest.approx(0.0, abs=KINEMATIC_TOLERANCE)

def test_no_initial_illegal_penetration():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert simulator.penetrating_contacts() == []
        validate_no_initial_penetration(simulator)

def test_scene_geometry_matches_configuration():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        validate_simulation(config, simulator)
        for surface_id, surface in config.scene["surfaces"].items():
            assert simulator.body_position(surface_id) == pytest.approx(surface["pose"]["position"], abs=KINEMATIC_TOLERANCE)
            assert simulator.geom_dimensions(f"{surface_id}_top") == pytest.approx(surface["dimensions"], abs=KINEMATIC_TOLERANCE)

def test_scene_layout_is_deterministic():
    config = _config()
    names = (*SURFACE_IDS, "panda1_rail", "panda2_rail", "panda1_carriage", "panda2_carriage", "panda1_base", "panda2_base")
    snapshots = []
    for _ in range(3):
        with SceneBuilder(config).build(headless=True) as simulator:
            snapshots.append({name: simulator.body_position(name) for name in names})
    assert snapshots[0] == snapshots[1] == snapshots[2]

def test_one_hundred_resets_preserve_mount_transforms():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        reset = ResetManager(simulator, config=config)
        expected = {robot_id: simulator.body_position(robot["arm"]["base_frame"]) for robot_id, robot in config.robots.items()}
        for _ in range(100):
            for robot in config.robots.values():
                simulator.set_joint_position(robot["rail"]["joint"], 0.4)
                simulator.set_joint_velocity(robot["rail"]["joint"], 0.2)
            simulator.forward()
            reset.reset()
            validate_no_initial_penetration(simulator)
            for robot_id, robot in config.robots.items():
                assert simulator.body_position(robot["arm"]["base_frame"]) == pytest.approx(expected[robot_id], abs=KINEMATIC_TOLERANCE)

def test_rail_endpoints_remain_clear_of_scene_geometry():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        for endpoint in ("lower_limit", "upper_limit"):
            for robot in config.robots.values():
                rail = robot["rail"]
                simulator.set_joint_position(rail["joint"], rail[endpoint])
                simulator.set_joint_velocity(rail["joint"], 0.0)
            simulator.forward()
            assert simulator.penetrating_contacts() == []
