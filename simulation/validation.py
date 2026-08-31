"""Startup validation for model structure and simulation state."""
from __future__ import annotations
import math
from .config_loader import ConfigBundle
from .simulator import Simulator
class ValidationError(RuntimeError): pass
def validate_simulation(config: ConfigBundle, simulator: Simulator) -> None:
    timestep = float(simulator.model.opt.timestep)
    if not math.isfinite(timestep) or timestep <= 0: raise ValidationError(f"Invalid MuJoCo timestep: {timestep}")
    gravity = tuple(map(float, simulator.model.opt.gravity))
    if len(gravity) != 3 or not all(map(math.isfinite, gravity)): raise ValidationError(f"Invalid MuJoCo gravity: {gravity}")
    missing = [name for name in config.expected_scene_entities if not simulator.entity_exists(name)]
    if missing: raise ValidationError(f"Expected scene entities are missing: {', '.join(missing)}")
    validate_state(simulator)
def validate_state(simulator: Simulator) -> None:
    for name, values in {"qpos": simulator.data.qpos, "qvel": simulator.data.qvel, "act": simulator.data.act}.items():
        if not all(math.isfinite(float(value)) for value in values): raise ValidationError(f"Simulation state '{name}' contains NaN or Inf")
    if not math.isfinite(simulator.time): raise ValidationError("Simulation time contains NaN or Inf")
