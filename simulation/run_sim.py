#!/usr/bin/env python3
"""Phase 1A simulation smoke-test entry point."""
from __future__ import annotations
import logging
import sys
from pathlib import Path
if __package__ in (None, ""): sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation.config_loader import ConfigLoader
from simulation.reset_manager import ResetManager
from simulation.scene_builder import SceneBuilder
from simulation.validation import validate_simulation, validate_state

def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    log = logging.getLogger("home_robotics")
    root = Path(__file__).resolve().parents[1]
    try:
        config = ConfigLoader(root / "config").load()
        log.info("[Home Robotics]"); log.info("Config ............. OK")
        with SceneBuilder(config).build(headless=True) as simulator:
            log.info("MuJoCo model ....... OK")
            validate_simulation(config, simulator)
            log.info("Physics ............ OK"); log.info("Scene validation ... OK")
            simulator.step(5); validate_state(simulator)
            ResetManager(simulator, settle_steps=config.physics["reset"]["settle_steps"]).reset()
            validate_simulation(config, simulator); log.info("Reset .............. OK")
        return 0
    except Exception as exc:
        log.error("Startup failed: %s", exc); return 1
if __name__ == "__main__": raise SystemExit(main())
