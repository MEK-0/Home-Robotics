#!/usr/bin/env python3
"""Optional interactive viewer for the validated Phase 1 scene."""
from __future__ import annotations
import sys
from pathlib import Path
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco.viewer
from simulation.config_loader import ConfigLoader
from simulation.scene_builder import SceneBuilder
from simulation.validation import validate_home_state, validate_simulation

def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    config = ConfigLoader(project_root / "config").load()
    with SceneBuilder(config).build(headless=False) as simulator:
        validate_simulation(config, simulator)
        validate_home_state(config, simulator)
        mujoco.viewer.launch(simulator.model, simulator.data)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
