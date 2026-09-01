"""Phase 1B.3 functional-object and final-scene acceptance tests."""
from __future__ import annotations

import math
from pathlib import Path

import mujoco
import pytest

from simulation.config_loader import ConfigLoader, OBJECT_IDS, SURFACE_IDS
from simulation.reset_manager import ResetManager
from simulation.scene_builder import SceneBuilder
from simulation.scene_geometry import resolve_object_pose
from simulation.validation import validate_carriage_separation, validate_simulation, validate_state

ROOT = Path(__file__).resolve().parents[2]
IDLE_DURATION = 10.0
TRANSLATION_TOLERANCE = 0.001
ROTATION_TOLERANCE = 0.001
VELOCITY_TOLERANCE = 1e-5
JOINT_TOLERANCE = 1e-6
SUPPORT_PENETRATION_TOLERANCE = 0.0005


def _config():
    return ConfigLoader(ROOT / "config").load()


def _quaternion_distance(first, second):
    dot = min(1.0, abs(sum(a * b for a, b in zip(first, second, strict=True))))
    return 2.0 * math.acos(dot)


def _assert_only_support_contacts(config, simulator):
    allowed = {
        frozenset((f"{object_id}_collision", f"{obj['initial']['support_surface']}_top"))
        for object_id, obj in config.objects.items() if obj["dynamic"]
    }
    illegal = []
    for first, second, distance in simulator.penetrating_contacts():
        if frozenset((first, second)) not in allowed or distance < -SUPPORT_PENETRATION_TOLERANCE:
            illegal.append((first, second, distance))
    assert illegal == []


def test_phase1_scene_and_physics_baselines_are_locked_at_version_one():
    config = _config()
    assert config.scene["version"] == "1.0"
    assert config.scene["build_stage"] == "phase1_baseline_locked"
    assert config.physics["version"] == "1.0"


def test_complete_scene_contains_five_objects_and_twenty_four_colliding_table_legs():
    config = _config()
    with SceneBuilder(config).build() as simulator:
        assert tuple(config.objects) == OBJECT_IDS
        assert all(simulator.body_exists(object_id) for object_id in OBJECT_IDS)
        legs = []
        for surface_id in SURFACE_IDS:
            for corner in ("front_left", "front_right", "back_left", "back_right"):
                name = f"{surface_id}_leg_{corner}"
                legs.append(name)
                geom_id = mujoco.mj_name2id(simulator.model, mujoco.mjtObj.mjOBJ_GEOM, name)
                assert geom_id >= 0
                assert simulator.model.geom_contype[geom_id] == 1
                assert simulator.model.geom_conaffinity[geom_id] == 1
                assert simulator.geom_dimensions(name) == pytest.approx(config.scene["table_geometry"]["leg_dimensions"])
        assert len(legs) == 24
        assert not any(simulator.geom_exists(f"{surface_id}_base") for surface_id in SURFACE_IDS)


def test_object_registry_maps_to_valid_mujoco_bodies_inertials_and_semantic_frames():
    config = _config()
    with SceneBuilder(config).build() as simulator:
        validate_simulation(config, simulator)
        for object_id, obj in config.objects.items():
            body_id = mujoco.mj_name2id(simulator.model, mujoco.mjtObj.mjOBJ_BODY, object_id)
            assert body_id >= 0
            if obj["dynamic"]:
                assert simulator.model.body_mass[body_id] == pytest.approx(obj["mass"])
                assert all(value > 0 for value in simulator.model.body_inertia[body_id])
        assert simulator.site_exists("bowl_inner")
        assert simulator.site_exists("pan_handle")


def test_bowl_inner_has_object_sized_collision_free_volume():
    config = _config()
    bowl = config.objects["bowl"]
    location = config.locations["bowl"]
    walls = [primitive for primitive in bowl["collision"]["primitives"] if primitive["name"].startswith("wall_")]
    inner_radius = min(math.hypot(*primitive["position"][:2]) - primitive["dimensions"][0] / 2.0 for primitive in walls)
    cube = config.objects["cube"]["collision"]["dimensions"]
    cube_xy_radius = math.hypot(cube[0] / 2.0, cube[1] / 2.0)
    safe_radius = float(location["safe_placement_region"]["radius"])
    assert safe_radius + cube_xy_radius < inner_radius
    assert location["valid_region"]["vertical_range"][1] < bowl["semantic_frames"]["bowl_rim"]["position"][2]
    with SceneBuilder(config).build() as simulator:
        inner = simulator.site_position("bowl_inner")
        bowl_position = simulator.body_position("bowl")
        assert inner[2] > bowl_position[2]


def test_purple_ball_is_idle_stable_and_can_roll_when_disturbed():
    config = _config()
    with SceneBuilder(config).build() as simulator:
        reset = ResetManager(simulator, config=config)
        start = simulator.body_position("purple_ball")
        simulator.step(int(2.0 / config.physics["timestep"]))
        assert math.dist(start, simulator.body_position("purple_ball")) <= TRANSLATION_TOLERANCE
        reset.reset()
        start = simulator.body_position("purple_ball")
        simulator.set_free_joint_velocity("purple_ball_free_joint", (0.15, 0.0, 0.0, 0.0, 2.0, 0.0))
        simulator.step(int(0.5 / config.physics["timestep"]))
        moved = simulator.body_position("purple_ball")
        assert math.dist(start, moved) > 0.02
        assert moved[2] > config.scene["floor"]["pose"]["position"][2] + 0.1


def test_full_scene_idle_stability_for_ten_simulated_seconds():
    config = _config()
    with SceneBuilder(config).build() as simulator:
        object_start = {name: (simulator.body_position(name), simulator.body_orientation(name)) for name in OBJECT_IDS}
        joint_start = {name: simulator.joint_position(name) for name in config.reset_joint_positions}
        simulator.step(int(IDLE_DURATION / config.physics["timestep"]))
        assert simulator.time == pytest.approx(IDLE_DURATION)
        for name, (position, orientation) in object_start.items():
            assert math.dist(position, simulator.body_position(name)) <= TRANSLATION_TOLERANCE
            assert _quaternion_distance(orientation, simulator.body_orientation(name)) <= ROTATION_TOLERANCE
            assert max(map(abs, simulator.body_velocity(name))) <= VELOCITY_TOLERANCE
        for name, position in joint_start.items():
            assert simulator.joint_position(name) == pytest.approx(position, abs=JOINT_TOLERANCE)
            assert simulator.joint_velocity(name) == pytest.approx(0.0, abs=VELOCITY_TOLERANCE)
        validate_carriage_separation(config, simulator)
        validate_state(simulator)
        _assert_only_support_contacts(config, simulator)


def test_one_hundred_complete_scene_resets_restore_every_state():
    config = _config()
    with SceneBuilder(config).build() as simulator:
        reset = ResetManager(simulator, config=config)
        expected_objects = {object_id: resolve_object_pose(config, obj) for object_id, obj in config.objects.items()}
        expected_tcp = {robot_id: simulator.site_position(robot["gripper"]["tcp_frame"]) for robot_id, robot in config.robots.items()}
        for _ in range(100):
            for joint_name, home in config.reset_joint_positions.items():
                lower, upper = simulator.joint_range(joint_name)
                simulator.set_joint_position(joint_name, min(upper, max(lower, home + 0.01)))
                simulator.set_joint_velocity(joint_name, 0.1)
            for object_id, obj in config.objects.items():
                if obj["dynamic"]:
                    position, quaternion = expected_objects[object_id]
                    simulator.set_free_joint_pose(f"{object_id}_free_joint", (position[0] + 0.01, position[1], position[2] + 0.01), quaternion)
                    simulator.set_free_joint_velocity(f"{object_id}_free_joint", (0.1, 0.0, 0.0, 0.0, 0.1, 0.0))
            reset.reset()
            for joint_name, home in config.reset_joint_positions.items():
                assert simulator.joint_position(joint_name) == pytest.approx(home, abs=JOINT_TOLERANCE)
                assert simulator.joint_velocity(joint_name) == pytest.approx(0.0, abs=VELOCITY_TOLERANCE)
            for object_id, (position, orientation) in expected_objects.items():
                assert simulator.body_position(object_id) == pytest.approx(position, abs=JOINT_TOLERANCE)
                assert simulator.body_orientation(object_id) == pytest.approx(orientation, abs=JOINT_TOLERANCE)
                assert max(map(abs, simulator.body_velocity(object_id))) <= VELOCITY_TOLERANCE
            for robot_id, robot in config.robots.items():
                assert simulator.site_position(robot["gripper"]["tcp_frame"]) == pytest.approx(expected_tcp[robot_id], abs=JOINT_TOLERANCE)
            validate_carriage_separation(config, simulator)
            validate_state(simulator)
            assert simulator.penetrating_contacts() == []
