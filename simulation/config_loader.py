"""Explicit loading and validation for the project's YAML source of truth."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence
import math
import yaml

SURFACE_IDS = (
    "surface_left_1", "surface_left_2", "surface_left_3",
    "surface_right_1", "surface_right_2", "surface_right_3",
)
ROBOT_IDS = ("panda1", "panda2")

class ConfigError(ValueError):
    """Project configuration is missing or inconsistent."""

class _UniqueKeyLoader(yaml.SafeLoader):
    pass

def _construct_unique_mapping(loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False) -> dict[Any, Any]:
    result: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ConfigError(f"Duplicate canonical ID or mapping key: {key!r}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result

_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping)

@dataclass(frozen=True)
class ConfigBundle:
    root: Path
    scene: Mapping[str, Any]
    robots: Mapping[str, Any]
    objects: Mapping[str, Any]
    locations: Mapping[str, Any]
    grasp_profiles: Mapping[str, Any]
    physics: Mapping[str, Any]

    @property
    def expected_scene_entities(self) -> tuple[str, ...]:
        return tuple(self.scene["expected_entities"])

    @property
    def rail_home_positions(self) -> dict[str, float]:
        return {str(robot["rail"]["joint"]): float(robot["rail"]["home_position"]) for robot in self.robots.values()}

    @property
    def robot_home_positions(self) -> dict[str, float]:
        positions: dict[str, float] = {}
        for robot in self.robots.values():
            positions.update(zip(robot["arm"]["joint_names"], map(float, robot["arm"]["home_joints"]), strict=True))
            positions.update(zip(robot["gripper"]["finger_joint_names"], map(float, robot["gripper"]["home_joints"]), strict=True))
        return positions

    @property
    def reset_joint_positions(self) -> dict[str, float]:
        return {**self.rail_home_positions, **self.robot_home_positions}

class ConfigLoader:
    FILES = {
        "scene": "scene.yaml", "robots": "robots.yaml", "objects": "objects.yaml",
        "locations": "locations.yaml", "grasp_profiles": "grasp_profiles.yaml", "physics": "physics.yaml",
    }

    def __init__(self, config_dir: Path | str) -> None:
        self.config_dir = Path(config_dir).resolve()

    def load(self) -> ConfigBundle:
        loaded = {name: self._load_file(filename, name) for name, filename in self.FILES.items()}
        bundle = ConfigBundle(root=self.config_dir, **loaded)
        self._validate(bundle)
        return bundle

    def _load_file(self, filename: str, top_level: str) -> Mapping[str, Any]:
        path = self.config_dir / filename
        if not path.is_file():
            raise ConfigError(f"Required configuration file is missing: {path}")
        try:
            document = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
        except ConfigError:
            raise
        except yaml.YAMLError as exc:
            raise ConfigError(f"Invalid YAML in {path}: {exc}") from exc
        if not isinstance(document, dict):
            raise ConfigError(f"Configuration file {path} must contain a YAML mapping")
        if top_level not in document:
            raise ConfigError(f"Configuration file {path} requires top-level field '{top_level}'")
        value = document[top_level]
        if not isinstance(value, dict):
            raise ConfigError(f"Top-level field '{top_level}' in {path} must be a mapping")
        return value

    @staticmethod
    def _require_fields(entry: Mapping[str, Any], fields: Sequence[str], context: str) -> None:
        missing = [field for field in fields if field not in entry]
        if missing:
            raise ConfigError(f"{context} missing required field(s): {', '.join(missing)}")

    @staticmethod
    def _positive_vector(value: Any, length: int, context: str) -> None:
        if not isinstance(value, list) or len(value) != length or not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in value):
            raise ConfigError(f"{context} must contain {length} positive finite numbers")

    def _validate(self, config: ConfigBundle) -> None:
        self._require_fields(config.scene, ("version", "frame", "units", "expected_entities", "floor", "surfaces", "workspaces"), "scene")
        self._require_fields(config.physics, ("version", "timestep", "gravity", "solver", "friction_profiles", "reset"), "physics")
        if config.scene["frame"] != "world":
            raise ConfigError("scene.frame must be 'world'")
        if config.scene["units"] != "SI":
            raise ConfigError("scene.units must be 'SI'")
        entities = config.scene["expected_entities"]
        if not isinstance(entities, list) or not entities:
            raise ConfigError("scene.expected_entities must be a non-empty list")
        if len(entities) != len(set(entities)):
            raise ConfigError("scene.expected_entities contains duplicate canonical IDs")
        if tuple(config.scene["surfaces"].keys()) != SURFACE_IDS:
            raise ConfigError(f"scene.surfaces must contain canonical IDs in order: {', '.join(SURFACE_IDS)}")
        for surface_id, surface in config.scene["surfaces"].items():
            self._require_fields(surface, ("id", "pose", "dimensions", "top_height", "collision_geometry", "usable_top_region", "safe_spawn_region", "safe_place_region", "edge_clearance", "workspace", "base_dimensions", "base_offset", "base_rgba"), f"surface '{surface_id}'")
            if surface["id"] != surface_id:
                raise ConfigError(f"Surface key '{surface_id}' does not match id '{surface['id']}'")
            if surface["collision_geometry"] != "box":
                raise ConfigError(f"Surface '{surface_id}' collision_geometry must be 'box'")
            self._positive_vector(surface["dimensions"], 3, f"surface '{surface_id}'.dimensions")
            self._positive_vector(surface["base_dimensions"], 3, f"surface '{surface_id}'.base_dimensions")
            pose = surface["pose"]
            if pose.get("frame") != "world" or len(pose.get("position", [])) != 3:
                raise ConfigError(f"Surface '{surface_id}' pose must be expressed in world")
            if not math.isclose(float(pose["position"][2]), float(surface["top_height"]), abs_tol=1e-12):
                raise ConfigError(f"Surface '{surface_id}' frame Z must equal top_height")
            if not isinstance(surface["edge_clearance"], (int, float)) or surface["edge_clearance"] <= 0:
                raise ConfigError(f"Surface '{surface_id}' edge_clearance must be positive")

        timestep = config.physics["timestep"]
        if not isinstance(timestep, (int, float)) or isinstance(timestep, bool) or timestep <= 0:
            raise ConfigError("physics.timestep must be a positive number")
        gravity = config.physics["gravity"]
        if not isinstance(gravity, list) or len(gravity) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in gravity):
            raise ConfigError("physics.gravity must contain exactly three finite numbers")

        surfaces, workspaces, profiles = set(config.scene["surfaces"]), set(config.scene["workspaces"]), set(config.grasp_profiles)
        if tuple(config.robots.keys()) != ROBOT_IDS:
            raise ConfigError("robots must contain canonical IDs panda1 and panda2")
        for robot_id, robot in config.robots.items():
            self._require_fields(robot, ("active", "namespace", "model_reference", "prefix", "rail", "arm", "gripper", "workspace", "nominal_facing_yaw", "reachable_surfaces", "home_validation"), f"robot '{robot_id}'")
            for reference in (robot["workspace"].get("primary"), robot["workspace"].get("shared")):
                if reference not in workspaces:
                    raise ConfigError(f"Robot '{robot_id}' references unknown workspace '{reference}'")
            unknown_surfaces = set(robot["reachable_surfaces"]) - surfaces
            if unknown_surfaces:
                raise ConfigError(f"Robot '{robot_id}' references unknown surfaces: {', '.join(sorted(unknown_surfaces))}")
            rail = robot["rail"]
            self._require_fields(rail, ("frame", "carriage_frame", "joint", "world_pose", "axis", "lower_limit", "upper_limit", "home_position", "support_dimensions", "carriage_dimensions", "carriage_offset", "mount_dimensions", "mount_offset", "mount_site_size"), f"robot '{robot_id}' rail")
            if rail["frame"] != f"{robot_id}_rail" or rail["carriage_frame"] != f"{robot_id}_carriage" or rail["joint"] != f"{robot_id}_rail_joint":
                raise ConfigError(f"Robot '{robot_id}' rail uses non-canonical IDs")
            if rail["axis"] != [1.0, 0.0, 0.0]:
                raise ConfigError(f"Robot '{robot_id}' rail axis must follow world +X")
            lower, upper, home = map(float, (rail["lower_limit"], rail["upper_limit"], rail["home_position"]))
            if not lower < upper:
                raise ConfigError(f"Robot '{robot_id}' rail lower_limit must be less than upper_limit")
            if not lower <= home <= upper:
                raise ConfigError(f"Robot '{robot_id}' rail home_position must lie inside limits")
            for field in ("support_dimensions", "carriage_dimensions", "mount_dimensions"):
                self._positive_vector(rail[field], 3, f"robot '{robot_id}' rail.{field}")

            if robot["prefix"] != f"{robot_id}_":
                raise ConfigError(f"Robot '{robot_id}' must use prefix '{robot_id}_'")
            arm, gripper = robot["arm"], robot["gripper"]
            self._require_fields(arm, ("model", "base_frame", "link0_frame", "joint_names", "home_joints"), f"robot '{robot_id}' arm")
            self._require_fields(gripper, ("model", "hand_frame", "finger_joint_names", "home_joints", "opening_range_per_finger", "tcp_frame", "tcp_offset"), f"robot '{robot_id}' gripper")
            if len(arm["joint_names"]) != 7 or len(arm["home_joints"]) != 7:
                raise ConfigError(f"Robot '{robot_id}' must define seven arm joints and seven home values")
            if len(gripper["finger_joint_names"]) != 2 or len(gripper["home_joints"]) != 2:
                raise ConfigError(f"Robot '{robot_id}' must define two finger joints and two home values")
            expected_arm_names = [f"{robot_id}_joint{index}" for index in range(1, 8)]
            if arm["joint_names"] != expected_arm_names:
                raise ConfigError(f"Robot '{robot_id}' arm joint names are not canonical")
            expected_finger_names = [f"{robot_id}_finger_joint1", f"{robot_id}_finger_joint2"]
            if gripper["finger_joint_names"] != expected_finger_names:
                raise ConfigError(f"Robot '{robot_id}' finger joint names are not canonical")
            if gripper["hand_frame"] != f"{robot_id}_hand" or gripper["tcp_frame"] != f"{robot_id}_tcp":
                raise ConfigError(f"Robot '{robot_id}' hand/TCP frame names are not canonical")
            homes = [*arm["home_joints"], *gripper["home_joints"]]
            if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in homes):
                raise ConfigError(f"Robot '{robot_id}' home values must be finite numbers")
            tcp = gripper["tcp_offset"]
            if tcp.get("frame") != gripper["hand_frame"] or len(tcp.get("position", [])) != 3 or len(tcp.get("quaternion_wxyz", [])) != 4:
                raise ConfigError(f"Robot '{robot_id}' TCP offset must be defined relative to its hand frame")

            minimum_tcp_height = robot["home_validation"].get("minimum_tcp_height")
            if not isinstance(minimum_tcp_height, (int, float)) or not math.isfinite(minimum_tcp_height) or minimum_tcp_height <= 0:
                raise ConfigError(f"Robot '{robot_id}' minimum TCP height must be a positive finite number")
        if config.robots["panda2"]["active"]:
            raise ConfigError("panda2 must remain inactive before the dual-arm phase")

        for object_id, obj in config.objects.items():
            self._require_fields(obj, ("category", "dynamic", "pickable", "place_target", "collision", "initial", "grasp_profile"), f"object '{object_id}'")
            support = obj["initial"].get("support_surface")
            if support not in surfaces:
                raise ConfigError(f"Object '{object_id}' references unknown support surface '{support}'")
            profile = obj.get("grasp_profile")
            if obj["pickable"] and profile not in profiles:
                raise ConfigError(f"Object '{object_id}' references unknown grasp profile '{profile}'")
            if obj["place_target"] and object_id not in config.locations:
                raise ConfigError(f"Place target '{object_id}' has no matching location")
