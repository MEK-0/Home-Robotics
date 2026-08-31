"""Narrow wrapper around the MuJoCo Python API."""
from __future__ import annotations
from typing import Any

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
    def from_xml_string(cls, xml: str, *, headless: bool = True) -> "Simulator":
        mujoco = _import_mujoco()
        try: model = mujoco.MjModel.from_xml_string(xml)
        except Exception as exc: raise RuntimeError(f"Failed to load MuJoCo model: {exc}") from exc
        return cls(model, mujoco.MjData(model), headless=headless)
    @property
    def time(self) -> float: return float(self.data.time)
    def step(self, count: int = 1) -> None:
        if count < 0: raise ValueError("step count must be non-negative")
        for _ in range(count): self._mujoco.mj_step(self.model, self.data)
    def reset(self) -> None:
        self._mujoco.mj_resetData(self.model, self.data)
        self._mujoco.mj_forward(self.model, self.data)
    def entity_exists(self, name: str) -> bool:
        kinds = (self._mujoco.mjtObj.mjOBJ_BODY, self._mujoco.mjtObj.mjOBJ_GEOM)
        return any(self._mujoco.mj_name2id(self.model, kind, name) >= 0 for kind in kinds)
    def close(self) -> None:
        if self._viewer is not None: self._viewer.close(); self._viewer = None
    def __enter__(self) -> "Simulator": return self
    def __exit__(self, *_: object) -> None: self.close()
