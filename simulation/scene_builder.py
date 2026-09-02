"""Build the staged MuJoCo scene from authoritative configuration."""
from __future__ import annotations
import math
from pathlib import Path
from typing import Any, Mapping
from xml.etree import ElementTree as ET
from .config_loader import ConfigBundle, ConfigError
from .panda_model import PandaModelSource
from .scene_geometry import resolve_object_pose, yaw_quaternion
from .simulator import Simulator

class SceneBuilder:
    def __init__(self, config: ConfigBundle, world_path: Path | str | None = None) -> None:
        self.config = config
        self.world_path = Path(world_path or Path(__file__).parent / "mujoco" / "world.xml").resolve()
        model_paths = {(config.root.parent / robot["model_reference"]).resolve() for robot in config.robots.values()}
        if len(model_paths) != 1:
            raise ConfigError("Phase 1B.2 requires both robots to use one shared validated Panda model")
        self.panda_source = PandaModelSource(model_paths.pop())

    @staticmethod
    def _values(values: Any) -> str:
        return " ".join(str(value) for value in values)

    @staticmethod
    def _half_size(dimensions: Any) -> str:
        return " ".join(str(float(value) / 2.0) for value in dimensions)

    @staticmethod
    def _yaw_quaternion(yaw: float) -> str:
        return f"{math.cos(yaw / 2.0)} 0.0 0.0 {math.sin(yaw / 2.0)}"

    def _configure_floor(self, worldbody: ET.Element) -> None:
        floor = self.config.scene["floor"]
        geom = worldbody.find("geom[@name='floor']")
        if geom is None:
            raise ConfigError("MuJoCo world infrastructure is missing the 'floor' geom")
        pose, size = floor["pose"], floor["geom_size"]
        geom.attrib.update(
            pos=self._values(pose["position"]), quat=self._values(pose["quaternion_wxyz"]),
            size=self._values(size), material=str(floor["material"]),
        )
        profile = floor["friction_profile"]
        try:
            geom.set("friction", self._values(self.config.physics["friction_profiles"][profile]))
        except KeyError as exc:
            raise ConfigError(f"Floor references unknown friction profile '{profile}'") from exc

    def _add_arena_border(self, worldbody: ET.Element) -> None:
        floor = self.config.scene["floor"]
        border = floor["arena_border"]
        length, width = map(float, border["dimensions"])
        strip_width = float(border["width"])
        height = float(border["height"])
        floor_clearance = float(border["floor_clearance"])
        floor_position = tuple(map(float, floor["pose"]["position"]))
        z = floor_position[2] + floor_clearance + height / 2.0
        definitions = {
            "arena_front": ((floor_position[0] + length / 2.0 - strip_width / 2.0, floor_position[1], z), (strip_width, width, height)),
            "arena_back": ((floor_position[0] - length / 2.0 + strip_width / 2.0, floor_position[1], z), (strip_width, width, height)),
            "arena_left": ((floor_position[0], floor_position[1] + width / 2.0 - strip_width / 2.0, z), (length - 2.0 * strip_width, strip_width, height)),
            "arena_right": ((floor_position[0], floor_position[1] - width / 2.0 + strip_width / 2.0, z), (length - 2.0 * strip_width, strip_width, height)),
        }
        for name, (position, dimensions) in definitions.items():
            ET.SubElement(worldbody, "geom", name=name, type="box", pos=self._values(position), size=self._half_size(dimensions), material=str(border["material"]), contype="0", conaffinity="0")

    def _add_surfaces(self, worldbody: ET.Element) -> None:
        table = self.config.scene["table_geometry"]
        leg_dimensions = tuple(map(float, table["leg_dimensions"]))
        inset_x, inset_y = map(float, table["leg_center_inset"])
        for surface_id, surface in self.config.scene["surfaces"].items():
            pose = surface["pose"]
            body = ET.SubElement(worldbody, "body", name=surface_id, pos=self._values(pose["position"]), quat=self._values(pose["quaternion_wxyz"]))
            dimensions = surface["dimensions"]
            ET.SubElement(body, "geom", name=f"{surface_id}_top", type="box", pos=f"0 0 {-float(dimensions[2]) / 2.0}", size=self._half_size(dimensions), rgba=self._values(surface["rgba"]), contype="1", conaffinity="1")
            leg_z = float(self.config.scene["floor"]["pose"]["position"][2]) - float(surface["top_height"]) + leg_dimensions[2] / 2.0
            offsets = {
                "front_left": (float(dimensions[0]) / 2.0 - inset_x, float(dimensions[1]) / 2.0 - inset_y, leg_z),
                "front_right": (float(dimensions[0]) / 2.0 - inset_x, -float(dimensions[1]) / 2.0 + inset_y, leg_z),
                "back_left": (-float(dimensions[0]) / 2.0 + inset_x, float(dimensions[1]) / 2.0 - inset_y, leg_z),
                "back_right": (-float(dimensions[0]) / 2.0 + inset_x, -float(dimensions[1]) / 2.0 + inset_y, leg_z),
            }
            for corner, offset in offsets.items():
                ET.SubElement(body, "geom", name=f"{surface_id}_leg_{corner}", type="box", pos=self._values(offset), size=self._half_size(leg_dimensions), rgba=self._values(table["leg_rgba"]), contype="1", conaffinity="1")

    def _primitive_geom(self, body: ET.Element, object_id: str, primitive: Mapping[str, Any], *, rgba: Any, friction: Any, mass: float | None = None) -> None:
        geom_type = str(primitive["type"])
        attributes = {
            "name": f"{object_id}_{primitive.get('name', 'collision')}",
            "type": geom_type,
            "pos": self._values(primitive.get("position", [0.0, 0.0, 0.0])),
            "quat": self._values(yaw_quaternion(float(primitive.get("yaw", 0.0)))),
            "rgba": self._values(rgba),
            "friction": self._values(friction),
            "contype": "1",
            "conaffinity": "1",
        }
        if geom_type == "box":
            attributes["size"] = self._half_size(primitive["dimensions"])
        elif geom_type == "sphere":
            attributes["size"] = str(float(primitive["radius"]))
        elif geom_type == "cylinder":
            attributes["size"] = f"{float(primitive['radius'])} {float(primitive['height']) / 2.0}"
        else:
            raise ConfigError(f"Object '{object_id}' uses unsupported primitive type '{geom_type}'")
        if mass is not None:
            attributes["mass"] = str(mass)
        ET.SubElement(body, "geom", **attributes)

    def _add_objects(self, worldbody: ET.Element) -> None:
        for object_id, obj in self.config.objects.items():
            position, quaternion = resolve_object_pose(self.config, obj)
            body = ET.SubElement(worldbody, "body", name=object_id, pos=self._values(position), quat=self._values(quaternion))
            if obj["dynamic"]:
                ET.SubElement(body, "freejoint", name=f"{object_id}_free_joint")
            friction = self.config.physics["friction_profiles"][obj["friction_profile"]]
            collision = obj["collision"]
            primitives = collision.get("primitives")
            if primitives is None:
                primitive = {**collision, "name": "collision"}
                mass = float(obj["mass"]) if obj["dynamic"] else None
                self._primitive_geom(body, object_id, primitive, rgba=obj["visual"]["rgba"], friction=friction, mass=mass)
            else:
                for primitive in primitives:
                    self._primitive_geom(body, object_id, primitive, rgba=obj["visual"]["rgba"], friction=friction)
            stem = obj["visual"].get("stem")
            if stem is not None:
                radius = float(collision["radius"])
                height = float(stem["height"])
                ET.SubElement(body, "geom", name=f"{object_id}_stem_visual", type="cylinder", pos=f"0 0 {radius + height / 2.0}", size=f"{stem['radius']} {height / 2.0}", rgba=self._values(stem["rgba"]), contype="0", conaffinity="0", mass="0")
            for frame_name, frame in obj.get("semantic_frames", {}).items():
                ET.SubElement(body, "site", name=str(frame_name), pos=self._values(frame["position"]), size="0.008", rgba="0.1 0.8 0.2 0.7")

    def _add_shared_rail(self, root: ET.Element, worldbody: ET.Element) -> None:
        shared_rail = self.config.scene["shared_rail"]
        pose = shared_rail["pose"]
        rail_body = ET.SubElement(worldbody, "body", name=shared_rail["id"], pos=self._values(pose["position"]), quat=self._values(pose["quaternion_wxyz"]))
        ET.SubElement(rail_body, "geom", name="shared_rail_support", type="box", size=self._half_size(shared_rail["dimensions"]), rgba=self._values(shared_rail["rgba"]), contype="1", conaffinity="1")
        supports = shared_rail["supports"]
        for name, position in zip(supports["names"], supports["positions"], strict=True):
            ET.SubElement(rail_body, "geom", name=str(name), type="box", pos=self._values(position), size=self._half_size(supports["dimensions"]), rgba=self._values(shared_rail["rgba"]), contype="1", conaffinity="1")
        for robot_id, robot in self.config.robots.items():
            rail = robot["rail"]
            carriage = ET.SubElement(rail_body, "body", name=rail["carriage_frame"], pos=self._values(rail["carriage_offset"]))
            ET.SubElement(carriage, "joint", name=rail["joint"], type="slide", axis=self._values(rail["axis"]), range=f"{rail['lower_limit']} {rail['upper_limit']}", limited="true", damping=str(self.config.physics["rail"]["damping"]))
            ET.SubElement(carriage, "geom", name=f"{rail['carriage_frame']}_geom", type="box", size=self._half_size(rail["carriage_dimensions"]), rgba=self._values(rail["carriage_rgba"]), contype="1", conaffinity="1")
            base = ET.SubElement(carriage, "body", name=robot["arm"]["base_frame"], pos=self._values(rail["mount_offset"]), quat=self._yaw_quaternion(float(robot["nominal_facing_yaw"])))
            mount_height = float(rail["mount_dimensions"][2])
            ET.SubElement(base, "geom", name=f"{robot_id}_mount_geom", type="box", pos=f"0 0 {-mount_height / 2.0}", size=self._half_size(rail["mount_dimensions"]), rgba=self._values(rail["mount_rgba"]), contype="1", conaffinity="1")
            ET.SubElement(base, "site", name=f"{robot_id}_base_site", pos="0 0 0", size=str(rail["mount_site_size"]), rgba="0.1 0.8 0.2 1")
            self.panda_source.instantiate(root, base, robot, gravity_compensation=float(self.config.physics["robot"]["phase1_gravity_compensation"]))

        actuator = root.find("actuator")
        if actuator is None:
            actuator = ET.SubElement(root, "actuator")
        servo = self.config.physics["rail"]["position_servo"]
        maximum_control = float(servo["maximum_control"])
        for robot_id, robot in self.config.robots.items():
            rail = robot["rail"]
            ET.SubElement(
                actuator, "position", name=f"{robot_id}_rail_actuator", joint=str(rail["joint"]),
                kp=str(float(servo["kp"])), ctrllimited="true",
                ctrlrange=f"{rail['lower_limit']} {rail['upper_limit']}", forcelimited="true",
                forcerange=f"{-maximum_control} {maximum_control}",
            )

        for robot_id, robot in self.config.robots.items():
            arm_control = self.config.physics["robot"][
                f"{robot_id}_arm_position_control"
            ]
            for index, (joint_name, joint_limits) in enumerate(
                zip(
                    robot["arm"]["joint_names"],
                    robot["arm"]["joint_limits"],
                    strict=True,
                )
            ):
                force_limit = float(arm_control["force_limits"][index])
                ET.SubElement(
                    actuator,
                    "position",
                    name=f"{robot_id}_joint{index + 1}_actuator",
                    joint=str(joint_name),
                    kp=str(float(arm_control["kp"][index])),
                    kv=str(float(arm_control["damping"][index])),
                    ctrllimited="true",
                    ctrlrange=f"{joint_limits[0]} {joint_limits[1]}",
                    forcelimited="true",
                    forcerange=f"{-force_limit} {force_limit}",
                )

        gripper_control = self.config.physics["robot"]["gripper_width_control"]
        force_limit = float(gripper_control["force_limit"])
        for robot_id in self.config.robots:
            ET.SubElement(
                actuator,
                "general",
                name=f"{robot_id}_gripper_actuator",
                gaintype="fixed",
                biastype="affine",
                tendon=f"{robot_id}_split",
                gainprm=str(float(gripper_control["kp"])),
                biasprm=(
                    f"0 {-float(gripper_control['kp'])} "
                    f"{-float(gripper_control['damping'])}"
                ),
                ctrllimited="true",
                ctrlrange="0 0.04",
                forcelimited="true",
                forcerange=f"{-force_limit} {force_limit}",
            )

    def build_xml(self) -> str:
        if not self.world_path.is_file():
            raise ConfigError(f"MuJoCo world file is missing: {self.world_path}")
        root = ET.fromstring(self.world_path.read_text(encoding="utf-8"))
        option, worldbody = root.find("option"), root.find("worldbody")
        if option is None or worldbody is None:
            raise ConfigError("MuJoCo world requires <option> and <worldbody> elements")
        physics = self.config.physics
        solver = physics["solver"]
        option.attrib.update(
            timestep=str(physics["timestep"]), gravity=self._values(physics["gravity"]),
            solver=str(solver["type"]), iterations=str(solver["iterations"]), tolerance=str(solver["tolerance"]),
        )
        default_geom = root.find("default/geom")
        if default_geom is None:
            raise ConfigError("MuJoCo world requires a default geom configuration")
        contact = physics["contact"]
        default_geom.attrib.update(
            solref=self._values(contact["solref"]), solimp=self._values(contact["solimp"]),
            margin=str(contact["margin"]), gap=str(contact["gap"]),
        )
        self.panda_source.add_shared_definitions(root)
        self._configure_floor(worldbody)
        self._add_arena_border(worldbody)
        self._add_surfaces(worldbody)
        self._add_shared_rail(root, worldbody)
        self._add_objects(worldbody)
        return ET.tostring(root, encoding="unicode")

    def build(self, *, headless: bool = True) -> Simulator:
        simulator = Simulator.from_xml_string(
            self.build_xml(), assets=self.panda_source.virtual_assets(), headless=headless
        )
        simulator.set_joint_positions(self.config.reset_joint_positions)
        for robot_id, robot in self.config.robots.items():
            simulator.set_actuator_control(
                f"{robot_id}_rail_actuator", float(robot["rail"]["home_position"])
            )
        for robot_id, robot in self.config.robots.items():
            for index, home in enumerate(robot["arm"]["home_joints"], start=1):
                simulator.set_actuator_control(
                    f"{robot_id}_joint{index}_actuator", float(home)
                )
            simulator.set_actuator_control(
                f"{robot_id}_gripper_actuator",
                float(robot["gripper"]["home_joints"][0]),
            )
        simulator.forward()
        return simulator
