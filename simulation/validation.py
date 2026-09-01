"""Startup validation for configuration, scene geometry, and simulation state."""
from __future__ import annotations
import math
from collections.abc import Iterable
from .config_loader import ConfigBundle, SURFACE_IDS
from .simulator import Simulator

MODEL_TOLERANCE = 1e-9
PENETRATION_TOLERANCE = 1e-7

class ValidationError(RuntimeError):
    pass

def _close_vector(actual: Iterable[float], expected: Iterable[float], tolerance: float = MODEL_TOLERANCE) -> bool:
    return all(math.isclose(float(a), float(e), abs_tol=tolerance, rel_tol=0.0) for a, e in zip(actual, expected, strict=True))

def validate_simulation(config: ConfigBundle, simulator: Simulator) -> None:
    timestep = float(simulator.model.opt.timestep)
    if not math.isfinite(timestep) or timestep <= 0:
        raise ValidationError(f"Invalid MuJoCo timestep: {timestep}")
    gravity = tuple(map(float, simulator.model.opt.gravity))
    if len(gravity) != 3 or not all(map(math.isfinite, gravity)):
        raise ValidationError(f"Invalid MuJoCo gravity: {gravity}")
    missing = [name for name in config.expected_scene_entities if not simulator.entity_exists(name)]
    if missing:
        raise ValidationError(f"Expected scene entities are missing: {', '.join(missing)}")
    validate_scene_geometry(config, simulator)
    validate_robot_structure(config, simulator)
    validate_no_initial_penetration(simulator)
    validate_state(simulator)

def validate_scene_geometry(config: ConfigBundle, simulator: Simulator) -> None:
    if tuple(config.scene["surfaces"].keys()) != SURFACE_IDS:
        raise ValidationError("Configured surface IDs do not match the canonical six-surface map")
    shared_rail = config.scene["shared_rail"]
    if not simulator.body_exists("shared_rail") or simulator.body_exists("panda1_rail") or simulator.body_exists("panda2_rail"):
        raise ValidationError("Exactly one shared rail body and no obsolete rail bodies are required")
    if not _close_vector(simulator.body_position("shared_rail"), shared_rail["pose"]["position"]):
        raise ValidationError("Shared rail transform does not match configuration")
    if not _close_vector(simulator.geom_dimensions("shared_rail_support"), shared_rail["dimensions"]):
        raise ValidationError("Shared rail dimensions do not match configuration")
    for support_name in shared_rail["supports"]["names"]:
        if not simulator.geom_exists(support_name):
            raise ValidationError(f"Shared rail support '{support_name}' is missing")
        if not _close_vector(simulator.geom_dimensions(support_name), shared_rail["supports"]["dimensions"]):
            raise ValidationError(f"Shared rail support '{support_name}' dimensions do not match configuration")
    for surface_id, surface in config.scene["surfaces"].items():
        if not simulator.body_exists(surface_id):
            raise ValidationError(f"Configured surface body is missing: {surface_id}")
        if not _close_vector(simulator.body_position(surface_id), surface["pose"]["position"]):
            raise ValidationError(f"Surface '{surface_id}' transform does not match configuration")
        if not _close_vector(simulator.geom_dimensions(f"{surface_id}_top"), surface["dimensions"]):
            raise ValidationError(f"Surface '{surface_id}' dimensions do not match configuration")
    for robot_id, robot in config.robots.items():
        rail = robot["rail"]
        for body_name in (rail["carriage_frame"], robot["arm"]["base_frame"]):
            if not simulator.body_exists(body_name):
                raise ValidationError(f"Configured rail-chain body is missing: {body_name}")
        joint_name = rail["joint"]
        if not simulator.joint_exists(joint_name):
            raise ValidationError(f"Configured rail joint is missing: {joint_name}")
        if simulator.joint_type(joint_name) != 2:
            raise ValidationError(f"Rail joint '{joint_name}' must be prismatic")
        if not _close_vector(simulator.joint_axis(joint_name), rail["axis"]):
            raise ValidationError(f"Rail joint '{joint_name}' axis does not match configuration")
        if not _close_vector(simulator.joint_range(joint_name), (rail["lower_limit"], rail["upper_limit"])):
            raise ValidationError(f"Rail joint '{joint_name}' limits do not match configuration")
        if not _close_vector(simulator.geom_dimensions(f"{rail['carriage_frame']}_geom"), rail["carriage_dimensions"]):
            raise ValidationError(f"Carriage '{rail['carriage_frame']}' dimensions do not match configuration")
        if not _close_vector(simulator.geom_dimensions(f"{robot_id}_mount_geom"), rail["mount_dimensions"]):
            raise ValidationError(f"Mount '{robot_id}_base' dimensions do not match configuration")

def validate_carriage_separation(config: ConfigBundle, simulator: Simulator) -> None:
    rail1 = config.robots["panda1"]["rail"]
    rail2 = config.robots["panda2"]["rail"]
    separation = simulator.joint_position(rail2["joint"]) - simulator.joint_position(rail1["joint"])
    minimum = float(config.scene["shared_rail"]["minimum_carriage_separation"])
    if separation < minimum - MODEL_TOLERANCE:
        raise ValidationError(f"Carriage separation {separation} violates ordered minimum {minimum}")

def validate_robot_structure(config: ConfigBundle, simulator: Simulator) -> None:
    if simulator.model.nu != 0:
        raise ValidationError("Robot actuators must not be introduced during model-only Phase 1B.2")
    for robot_id, robot in config.robots.items():
        arm, gripper = robot["arm"], robot["gripper"]
        for body_name in (arm["link0_frame"], gripper["hand_frame"]):
            if not simulator.body_exists(body_name):
                raise ValidationError(f"Configured Panda body is missing: {body_name}")
        if not simulator.site_exists(gripper["tcp_frame"]):
            raise ValidationError(f"Configured TCP site is missing: {gripper['tcp_frame']}")
        joint_names = [*arm["joint_names"], *gripper["finger_joint_names"]]
        home_values = [*arm["home_joints"], *gripper["home_joints"]]
        for joint_name, home in zip(joint_names, home_values, strict=True):
            if not simulator.joint_exists(joint_name):
                raise ValidationError(f"Configured robot joint is missing: {joint_name}")
            lower, upper = simulator.joint_range(joint_name)
            if not all(math.isfinite(value) for value in (lower, upper)) or not lower < upper:
                raise ValidationError(f"Robot joint '{joint_name}' has invalid limits: [{lower}, {upper}]")
            if not lower <= float(home) <= upper:
                raise ValidationError(f"Robot joint '{joint_name}' home {home} is outside [{lower}, {upper}]")
        configured_finger_range = tuple(map(float, gripper["opening_range_per_finger"]))
        for joint_name in gripper["finger_joint_names"]:
            if not _close_vector(simulator.joint_range(joint_name), configured_finger_range):
                raise ValidationError(f"Finger joint '{joint_name}' range does not match configuration")
        tcp_position = simulator.site_position(gripper["tcp_frame"])
        if not all(math.isfinite(value) for value in tcp_position):
            raise ValidationError(f"TCP '{gripper['tcp_frame']}' contains NaN or Inf")
        if tcp_position[2] < float(robot["home_validation"]["minimum_tcp_height"]):
            raise ValidationError(f"TCP '{gripper['tcp_frame']}' is below its configured safe home height")
        hand_position = simulator.body_position(gripper["hand_frame"])
        actual_offset = math.dist(hand_position, tcp_position)
        configured_offset = math.sqrt(sum(float(value) ** 2 for value in gripper["tcp_offset"]["position"]))
        if not math.isclose(actual_offset, configured_offset, abs_tol=MODEL_TOLERANCE):
            raise ValidationError(f"TCP '{gripper['tcp_frame']}' offset does not match configuration")

def validate_home_state(config: ConfigBundle, simulator: Simulator) -> None:
    for joint_name, home in config.reset_joint_positions.items():
        if not math.isclose(simulator.joint_position(joint_name), float(home), abs_tol=MODEL_TOLERANCE):
            raise ValidationError(f"Joint '{joint_name}' is not at configured home")
        if not math.isclose(simulator.joint_velocity(joint_name), 0.0, abs_tol=MODEL_TOLERANCE):
            raise ValidationError(f"Joint '{joint_name}' velocity is not zero at home")
    for robot in config.robots.values():
        tcp_frame = robot["gripper"]["tcp_frame"]
        tcp_position = simulator.site_position(tcp_frame)
        if not all(math.isfinite(value) for value in tcp_position):
            raise ValidationError(f"TCP '{tcp_frame}' contains NaN or Inf at home")
        if tcp_position[2] < float(robot["home_validation"]["minimum_tcp_height"]):
            raise ValidationError(f"TCP '{tcp_frame}' is below its configured safe home height")
    validate_carriage_separation(config, simulator)
    validate_no_initial_penetration(simulator)

def validate_no_initial_penetration(simulator: Simulator) -> None:
    penetrations = simulator.penetrating_contacts(PENETRATION_TOLERANCE)
    if penetrations:
        details = "; ".join(f"{first}<->{second}: {distance:.6g} m" for first, second, distance in penetrations)
        raise ValidationError(f"Illegal initial penetration detected: {details}")

def validate_state(simulator: Simulator) -> None:
    for name, values in {"qpos": simulator.data.qpos, "qvel": simulator.data.qvel, "act": simulator.data.act}.items():
        if not all(math.isfinite(float(value)) for value in values):
            raise ValidationError(f"Simulation state '{name}' contains NaN or Inf")
    if not math.isfinite(simulator.time):
        raise ValidationError("Simulation time contains NaN or Inf")
