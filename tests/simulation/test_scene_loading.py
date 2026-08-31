from pathlib import Path
import pytest
try:
    import mujoco as _mujoco
    _HAS_MUJOCO = hasattr(_mujoco, "MjModel")
except ImportError:
    _HAS_MUJOCO = False
pytestmark = pytest.mark.skipif(not _HAS_MUJOCO, reason="MuJoCo Python dependency is not installed")
from simulation.config_loader import ConfigLoader
from simulation.scene_builder import SceneBuilder
from simulation.validation import validate_simulation
ROOT = Path(__file__).resolve().parents[2]
def test_mujoco_model_and_required_base_entities_load():
    config = ConfigLoader(ROOT / "config").load()
    with SceneBuilder(config).build(headless=True) as simulator:
        validate_simulation(config, simulator); assert simulator.entity_exists("floor"); assert simulator.time == 0.0
def test_simulation_data_remains_valid_after_steps():
    config = ConfigLoader(ROOT / "config").load()
    with SceneBuilder(config).build(headless=True) as simulator: simulator.step(5); validate_simulation(config, simulator)
