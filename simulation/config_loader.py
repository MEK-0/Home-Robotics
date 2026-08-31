"""Explicit loading and validation for the project's YAML source of truth."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping
import yaml

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

class ConfigLoader:
    FILES = {"scene": "scene.yaml", "robots": "robots.yaml", "objects": "objects.yaml", "locations": "locations.yaml", "grasp_profiles": "grasp_profiles.yaml", "physics": "physics.yaml"}
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
    def _require_fields(entry: Mapping[str, Any], fields: tuple[str, ...], context: str) -> None:
        missing = [field for field in fields if field not in entry]
        if missing:
            raise ConfigError(f"{context} missing required field(s): {', '.join(missing)}")
    def _validate(self, config: ConfigBundle) -> None:
        self._require_fields(config.scene, ("version", "frame", "expected_entities", "floor", "surfaces", "workspaces"), "scene")
        self._require_fields(config.physics, ("version", "timestep", "gravity", "solver", "friction_profiles", "reset"), "physics")
        if config.scene["frame"] != "world": raise ConfigError("scene.frame must be 'world'")
        entities = config.scene["expected_entities"]
        if not isinstance(entities, list) or not entities: raise ConfigError("scene.expected_entities must be a non-empty list")
        if len(entities) != len(set(entities)): raise ConfigError("scene.expected_entities contains duplicate canonical IDs")
        timestep = config.physics["timestep"]
        if not isinstance(timestep, (int, float)) or isinstance(timestep, bool) or timestep <= 0: raise ConfigError("physics.timestep must be a positive number")
        gravity = config.physics["gravity"]
        if not isinstance(gravity, list) or len(gravity) != 3 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in gravity): raise ConfigError("physics.gravity must contain exactly three numbers")
        surfaces, workspaces, profiles = set(config.scene["surfaces"]), set(config.scene["workspaces"]), set(config.grasp_profiles)
        for robot_id, robot in config.robots.items():
            self._require_fields(robot, ("active", "namespace", "rail", "arm", "gripper", "workspace"), f"robot '{robot_id}'")
            for reference in (robot["workspace"].get("primary"), robot["workspace"].get("shared")):
                if reference not in workspaces: raise ConfigError(f"Robot '{robot_id}' references unknown workspace '{reference}'")
            if robot["rail"].get("axis") != [1.0, 0.0, 0.0]: raise ConfigError(f"Robot '{robot_id}' rail axis must follow world +X")
        for object_id, obj in config.objects.items():
            self._require_fields(obj, ("category", "dynamic", "pickable", "place_target", "collision", "initial", "grasp_profile"), f"object '{object_id}'")
            support = obj["initial"].get("support_surface")
            if support not in surfaces: raise ConfigError(f"Object '{object_id}' references unknown support surface '{support}'")
            profile = obj.get("grasp_profile")
            if obj["pickable"] and profile not in profiles: raise ConfigError(f"Object '{object_id}' references unknown grasp profile '{profile}'")
            if obj["place_target"] and object_id not in config.locations: raise ConfigError(f"Place target '{object_id}' has no matching location")
