"""Reusable Phase 1 workspace coverage acceptance test."""
from pathlib import Path

from simulation.config_loader import ConfigLoader
from simulation.scene_builder import SceneBuilder
from simulation.workspace_reachability import build_targets, format_report, validate_system_coverage, validate_workspace

ROOT = Path(__file__).resolve().parents[2]


def test_system_covers_all_workspace_targets_with_at_least_one_robot():
    config = ConfigLoader(ROOT / "config").load()
    targets = build_targets(config)
    assert len(targets) == 18
    with SceneBuilder(config).build(headless=True) as simulator:
        results = validate_workspace(config, simulator)
        system_coverage = validate_system_coverage(config, simulator)
    panda1 = [result for result in results if result.target.robot == "panda1"]
    panda2 = [result for result in results if result.target.robot == "panda2"]
    assert sum(result.passed for result in panda1) == 8
    assert sum(result.passed for result in panda2) == 8
    unreachable = [result for result in system_coverage if not result.reachable_by_any]
    assert not unreachable, "\n" + format_report(results, system_coverage)
