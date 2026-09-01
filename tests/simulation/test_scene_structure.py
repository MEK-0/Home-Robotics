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

def test_arena_uses_one_floor_and_four_raised_border_boxes():
    config = _config()
    border = config.scene["floor"]["arena_border"]
    with SceneBuilder(config).build(headless=True) as simulator:
        import mujoco
        plane_geoms = [index for index in range(simulator.model.ngeom) if int(simulator.model.geom_type[index]) == int(mujoco.mjtGeom.mjGEOM_PLANE)]
        assert len(plane_geoms) == 1
        assert mujoco.mj_id2name(simulator.model, mujoco.mjtObj.mjOBJ_GEOM, plane_geoms[0]) == "floor"
        floor_id = plane_geoms[0]
        assert tuple(simulator.model.geom_size[floor_id]) == pytest.approx(config.scene["floor"]["geom_size"])
        forbidden = ("workspace_floor", "arena_floor", "outer_floor", "background_floor", "outside_front", "outside_back", "outside_left", "outside_right")
        assert all(not simulator.geom_exists(name) for name in forbidden)
        expected = {
            "arena_front": ([border["width"], border["dimensions"][1], border["height"]]),
            "arena_back": ([border["width"], border["dimensions"][1], border["height"]]),
            "arena_left": ([border["dimensions"][0] - 2 * border["width"], border["width"], border["height"]]),
            "arena_right": ([border["dimensions"][0] - 2 * border["width"], border["width"], border["height"]]),
        }
        for name, dimensions in expected.items():
            assert simulator.geom_exists(name)
            assert simulator.geom_dimensions(name) == pytest.approx(dimensions)
            geom_id = mujoco.mj_name2id(simulator.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            assert simulator.model.geom_pos[geom_id][2] - simulator.model.geom_size[geom_id][2] == pytest.approx(border["floor_clearance"])
            assert simulator.model.geom_contype[geom_id] == 0
            assert simulator.model.geom_conaffinity[geom_id] == 0

def test_six_work_surfaces_exist():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert tuple(config.scene["surfaces"]) == SURFACE_IDS
        assert all(simulator.body_exists(name) for name in SURFACE_IDS)

def test_one_shared_rail_and_two_carriages_exist():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert simulator.body_exists("shared_rail")
        assert not simulator.body_exists("panda1_rail")
        assert not simulator.body_exists("panda2_rail")
        assert all(simulator.body_exists(name) for name in ("panda1_carriage", "panda2_carriage"))

def test_rail_joint_ids_exist():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert all(simulator.joint_exists(robot["rail"]["joint"]) for robot in config.robots.values())

def test_shared_rail_is_static_and_axis_is_positive_x():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        assert simulator.body_joint_count("shared_rail") == 0
        assert config.scene["shared_rail"]["axis"] == [1.0, 0.0, 0.0]
        assert all(simulator.joint_axis(robot["rail"]["joint"]) == (1.0, 0.0, 0.0) for robot in config.robots.values())

def test_elevated_shared_rail_has_static_floor_supports():
    config = _config()
    shared_rail = config.scene["shared_rail"]
    with SceneBuilder(config).build(headless=True) as simulator:
        assert simulator.body_position("shared_rail") == pytest.approx([1.15, 0.0, 0.61], abs=KINEMATIC_TOLERANCE)
        for support_name in shared_rail["supports"]["names"]:
            assert simulator.geom_exists(support_name)
            assert simulator.geom_dimensions(support_name) == pytest.approx(shared_rail["supports"]["dimensions"])

def test_panda1_home_has_clearance_from_nearest_tables():
    """Regression guard for the near-table initial home pose."""
    import mujoco
    import numpy as np
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        model, data = simulator.model, simulator.data
        table_ids = [i for i in range(model.ngeom) if model.geom(i).name.startswith(("surface_left_1_", "surface_right_1_"))]
        panda1_ids = [i for i in range(model.ngeom) if "panda1" in model.body(int(model.geom_bodyid[i])).name]
        distances = [mujoco.mj_geomDistance(model, data, p, t, 10.0, np.zeros(6)) for p in panda1_ids for t in table_ids]
        assert distances and min(distances) >= 0.01
        assert not simulator.penetrating_contacts()

def test_distinct_rail_homes_have_safe_ordered_separation():
    config = _config()
    q1 = config.robots["panda1"]["rail"]["home_position"]
    q2 = config.robots["panda2"]["rail"]["home_position"]
    assert q1 != q2
    assert q2 - q1 >= config.scene["shared_rail"]["minimum_carriage_separation"]

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

def test_each_carriage_moves_independently():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        for moving_id, fixed_id in (("panda1", "panda2"), ("panda2", "panda1")):
            moving = config.robots[moving_id]["rail"]
            fixed = config.robots[fixed_id]["rail"]
            moving_before = simulator.body_position(moving["carriage_frame"])
            fixed_before = simulator.body_position(fixed["carriage_frame"])
            simulator.set_joint_position(moving["joint"], moving["home_position"] + 0.1)
            simulator.forward()
            assert simulator.body_position(moving["carriage_frame"])[0] - moving_before[0] == pytest.approx(0.1, abs=KINEMATIC_TOLERANCE)
            assert simulator.body_position(fixed["carriage_frame"]) == pytest.approx(fixed_before, abs=KINEMATIC_TOLERANCE)
            simulator.set_joint_position(moving["joint"], moving["home_position"])
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

def test_table_rows_are_symmetric_and_have_target_rail_clearance():
    config = _config()
    surfaces = config.scene["surfaces"]
    for index in range(1, 4):
        left = surfaces[f"surface_left_{index}"]
        right = surfaces[f"surface_right_{index}"]
        assert left["pose"]["position"][0] == right["pose"]["position"][0]
        assert left["pose"]["position"][1] == pytest.approx(-right["pose"]["position"][1])
        assert left["top_height"] == right["top_height"] == pytest.approx(0.68)
        assert left["dimensions"] == right["dimensions"] == [0.825, 0.75, 0.08]
    nearest_edge = abs(surfaces["surface_left_1"]["pose"]["position"][1]) - surfaces["surface_left_1"]["dimensions"][1] / 2.0
    assert nearest_edge == pytest.approx(0.115)
    assert config.scene["layout"]["corridor_clear_width"] == pytest.approx(2.0 * nearest_edge)

def test_scene_geometry_matches_configuration():
    config = _config()
    with SceneBuilder(config).build(headless=True) as simulator:
        validate_simulation(config, simulator)
        for surface_id, surface in config.scene["surfaces"].items():
            assert simulator.body_position(surface_id) == pytest.approx(surface["pose"]["position"], abs=KINEMATIC_TOLERANCE)
            assert simulator.geom_dimensions(f"{surface_id}_top") == pytest.approx(surface["dimensions"], abs=KINEMATIC_TOLERANCE)

def test_scene_layout_is_deterministic():
    config = _config()
    names = (*SURFACE_IDS, "shared_rail", "panda1_carriage", "panda2_carriage", "panda1_base", "panda2_base")
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
