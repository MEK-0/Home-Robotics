"""Narrow wrapper around the MuJoCo Python API."""
from __future__ import annotations
from typing import Any, Mapping

class MuJoCoUnavailableError(RuntimeError):
    pass

def _import_mujoco() -> Any:
    try:
        import mujoco
    except ImportError as exc:
        raise MuJoCoUnavailableError("MuJoCo Python package is required for simulation; dependency 'mujoco' is not installed") from exc
    if not hasattr(mujoco, "MjModel") or not hasattr(mujoco, "MjData"):
        raise MuJoCoUnavailableError("MuJoCo Python package is required for simulation; dependency 'mujoco' is not installed")
    return mujoco

class Simulator:
    def __init__(self, model: Any, data: Any, *, headless: bool = True) -> None:
        self._mujoco, self.model, self.data, self.headless = _import_mujoco(), model, data, headless
        self._viewer: Any | None = None

    @classmethod
    def from_xml_string(cls, xml: str, *, assets: Mapping[str, bytes] | None = None, headless: bool = True) -> "Simulator":
        mujoco = _import_mujoco()
        try:
            model = mujoco.MjModel.from_xml_string(xml, assets=assets)
        except Exception as exc:
            raise RuntimeError(f"Failed to load MuJoCo model: {exc}") from exc
        return cls(model, mujoco.MjData(model), headless=headless)

    @property
    def time(self) -> float:
        return float(self.data.time)

    def step(self, count: int = 1) -> None:
        if count < 0:
            raise ValueError("step count must be non-negative")
        for _ in range(count):
            self._mujoco.mj_step(self.model, self.data)

    def forward(self) -> None:
        self._mujoco.mj_forward(self.model, self.data)

    def reset(self) -> None:
        self._mujoco.mj_resetData(self.model, self.data)
        self.forward()

    def _id(self, kind: Any, name: str) -> int:
        identifier = int(self._mujoco.mj_name2id(self.model, kind, name))
        if identifier < 0:
            raise KeyError(f"MuJoCo entity does not exist: {name}")
        return identifier

    def entity_exists(self, name: str) -> bool:
        kinds = (self._mujoco.mjtObj.mjOBJ_BODY, self._mujoco.mjtObj.mjOBJ_GEOM, self._mujoco.mjtObj.mjOBJ_JOINT, self._mujoco.mjtObj.mjOBJ_SITE)
        return any(self._mujoco.mj_name2id(self.model, kind, name) >= 0 for kind in kinds)

    def body_joint_count(self, name: str) -> int:
        body_id = self._id(self._mujoco.mjtObj.mjOBJ_BODY, name)
        return int(self.model.body_jntnum[body_id])

    def body_exists(self, name: str) -> bool:
        return self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, name) >= 0

    def joint_exists(self, name: str) -> bool:
        return self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_JOINT, name) >= 0

    def geom_exists(self, name: str) -> bool:
        return self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_GEOM, name) >= 0

    def site_exists(self, name: str) -> bool:
        return self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_SITE, name) >= 0

    def body_position(self, name: str) -> tuple[float, float, float]:
        body_id = self._id(self._mujoco.mjtObj.mjOBJ_BODY, name)
        return tuple(float(value) for value in self.data.xpos[body_id])

    def body_orientation(self, name: str) -> tuple[float, float, float, float]:
        body_id = self._id(self._mujoco.mjtObj.mjOBJ_BODY, name)
        return tuple(float(value) for value in self.data.xquat[body_id])

    def body_velocity(self, name: str) -> tuple[float, float, float, float, float, float]:
        body_id = self._id(self._mujoco.mjtObj.mjOBJ_BODY, name)
        velocity = self.data.cvel[body_id]
        return tuple(float(value) for value in (*velocity[3:], *velocity[:3]))

    def site_position(self, name: str) -> tuple[float, float, float]:
        site_id = self._id(self._mujoco.mjtObj.mjOBJ_SITE, name)
        return tuple(float(value) for value in self.data.site_xpos[site_id])

    def joint_position(self, name: str) -> float:
        joint_id = self._id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        return float(self.data.qpos[self.model.jnt_qposadr[joint_id]])

    def joint_velocity(self, name: str) -> float:
        joint_id = self._id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        return float(self.data.qvel[self.model.jnt_dofadr[joint_id]])

    def joint_axis(self, name: str) -> tuple[float, float, float]:
        joint_id = self._id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        return tuple(float(value) for value in self.model.jnt_axis[joint_id])

    def joint_range(self, name: str) -> tuple[float, float]:
        joint_id = self._id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        return tuple(float(value) for value in self.model.jnt_range[joint_id])

    def joint_type(self, name: str) -> int:
        joint_id = self._id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_type[joint_id])

    def geom_dimensions(self, name: str) -> tuple[float, float, float]:
        geom_id = self._id(self._mujoco.mjtObj.mjOBJ_GEOM, name)
        return tuple(float(value) * 2.0 for value in self.model.geom_size[geom_id])

    def set_joint_position(self, name: str, value: float) -> None:
        joint_id = self._id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        self.data.qpos[self.model.jnt_qposadr[joint_id]] = value

    def set_joint_velocity(self, name: str, value: float) -> None:
        joint_id = self._id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        self.data.qvel[self.model.jnt_dofadr[joint_id]] = value

    def set_joint_positions(self, positions: Mapping[str, float]) -> None:
        for name, value in positions.items():
            self.set_joint_position(name, value)

    def set_free_joint_pose(self, name: str, position: tuple[float, float, float], quaternion_wxyz: tuple[float, float, float, float]) -> None:
        joint_id = self._id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        if int(self.model.jnt_type[joint_id]) != int(self._mujoco.mjtJoint.mjJNT_FREE):
            raise ValueError(f"Joint '{name}' is not a free joint")
        address = int(self.model.jnt_qposadr[joint_id])
        self.data.qpos[address:address + 7] = (*position, *quaternion_wxyz)

    def set_free_joint_velocity(self, name: str, velocity: tuple[float, float, float, float, float, float]) -> None:
        joint_id = self._id(self._mujoco.mjtObj.mjOBJ_JOINT, name)
        if int(self.model.jnt_type[joint_id]) != int(self._mujoco.mjtJoint.mjJNT_FREE):
            raise ValueError(f"Joint '{name}' is not a free joint")
        address = int(self.model.jnt_dofadr[joint_id])
        self.data.qvel[address:address + 6] = velocity

    def penetrating_contacts(self, tolerance: float = 1e-7) -> list[tuple[str, str, float]]:
        penetrations: list[tuple[str, str, float]] = []
        for contact in self.data.contact:
            if float(contact.dist) < -tolerance:
                first = self._mujoco.mj_id2name(self.model, self._mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or str(contact.geom1)
                second = self._mujoco.mj_id2name(self.model, self._mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or str(contact.geom2)
                penetrations.append((first, second, float(contact.dist)))
        return penetrations

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def __enter__(self) -> "Simulator":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
