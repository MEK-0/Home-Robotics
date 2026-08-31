"""Build the staged MuJoCo scene from authoritative configuration."""
from __future__ import annotations
from pathlib import Path
from xml.etree import ElementTree as ET
from .config_loader import ConfigBundle, ConfigError
from .simulator import Simulator

class SceneBuilder:
    def __init__(self, config: ConfigBundle, world_path: Path | str | None = None) -> None:
        self.config = config
        self.world_path = Path(world_path or Path(__file__).parent / "mujoco" / "world.xml").resolve()
    def build_xml(self) -> str:
        if not self.world_path.is_file(): raise ConfigError(f"MuJoCo world file is missing: {self.world_path}")
        root = ET.fromstring(self.world_path.read_text(encoding="utf-8"))
        option, worldbody = root.find("option"), root.find("worldbody")
        if option is None or worldbody is None: raise ConfigError("MuJoCo world requires <option> and <worldbody> elements")
        physics = self.config.physics
        option.attrib.update(timestep=str(physics["timestep"]), gravity=" ".join(map(str, physics["gravity"])), solver=str(physics["solver"]["type"]), iterations=str(physics["solver"]["iterations"]), tolerance=str(physics["solver"]["tolerance"]))
        floor, geom = self.config.scene["floor"], worldbody.find("geom[@name='floor']")
        if geom is None: raise ConfigError("MuJoCo world infrastructure is missing the 'floor' geom")
        pose, size = floor["pose"], floor["visual_half_size"]
        geom.attrib.update(pos=" ".join(map(str, pose["position"])), quat=" ".join(map(str, pose["quaternion_wxyz"])), size=f"{size[0]} {size[1]} 0.1")
        profile = floor["friction_profile"]
        if profile not in physics["friction_profiles"]: raise ConfigError(f"Floor references unknown friction profile '{profile}'")
        geom.set("friction", " ".join(map(str, physics["friction_profiles"][profile])))
        return ET.tostring(root, encoding="unicode")
    def build(self, *, headless: bool = True) -> Simulator: return Simulator.from_xml_string(self.build_xml(), headless=headless)
