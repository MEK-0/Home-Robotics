"""Reusable Phase 1 workspace coverage acceptance test."""
from pathlib import Path

from simulation.config_loader import ConfigLoader
from simulation.scene_builder import SceneBuilder
from simulation.workspace_reachability import build_targets, format_report, validate_workspace

ROOT = Path(__file__).resolve().parents[2]


def test_both_pandas_cover_all_configured_table_workspace_targets():
    config = ConfigLoader(ROOT / "config").load()
    targets = build_targets(config)
    assert len(targets) == 18
    with SceneBuilder(config).build(headless=True) as simulator:
        results = validate_workspace(config, simulator)
    failures = [result for result in results if not result.passed]
    assert not failures, "\n" + format_report(results)
