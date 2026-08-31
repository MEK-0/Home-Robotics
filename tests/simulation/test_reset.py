from pathlib import Path
import pytest
try:
    import mujoco as _mujoco
    _HAS_MUJOCO = hasattr(_mujoco, "MjModel")
except ImportError:
    _HAS_MUJOCO = False
pytestmark = pytest.mark.skipif(not _HAS_MUJOCO, reason="MuJoCo Python dependency is not installed")
from simulation.config_loader import ConfigLoader
from simulation.reset_manager import ResetManager
from simulation.scene_builder import SceneBuilder
from simulation.validation import validate_state
ROOT = Path(__file__).resolve().parents[2]
def _state(simulator): return simulator.time, simulator.data.qpos.copy(), simulator.data.qvel.copy()
def test_deterministic_reset_is_repeatable_and_clears_velocities():
    config = ConfigLoader(ROOT / "config").load()
    with SceneBuilder(config).build(headless=True) as simulator:
        reset = ResetManager(simulator); simulator.step(10); reset.reset(); first = _state(simulator); simulator.step(10); reset.reset(); second = _state(simulator)
        assert first[0] == second[0] == 0.0; assert (first[1] == second[1]).all(); assert (first[2] == second[2]).all(); assert (second[2] == 0.0).all()
def test_repeated_reset_does_not_produce_invalid_state():
    config = ConfigLoader(ROOT / "config").load()
    with SceneBuilder(config).build(headless=True) as simulator:
        reset = ResetManager(simulator)
        for _ in range(100): simulator.step(2); reset.reset(); validate_state(simulator)
