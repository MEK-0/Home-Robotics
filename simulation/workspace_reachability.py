"""Deterministic Phase 1 rail-plus-arm workspace reachability validation."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

import mujoco
import numpy as np

from .config_loader import ConfigBundle, ConfigLoader
from .scene_builder import SceneBuilder
from .simulator import Simulator

POSITION_TOLERANCE = 0.005
TARGET_CLEARANCE = 0.07
IK_DAMPING = 2e-4
MAX_ITERATIONS = 450
RANDOM_SEEDS = 48
JOINT_LIMIT_MARGIN = 0.025


@dataclass(frozen=True)
class ReachabilityTarget:
    robot: str
    station: str
    depth: str
    xyz: tuple[float, float, float]

    @property
    def region(self) -> str:
        return f"{self.station}/{self.depth}"


@dataclass(frozen=True)
class ReachabilityResult:
    target: ReachabilityTarget
    rail_position: float
    ik_converged: bool
    position_error: float
    joints_within_limits: bool
    collisions: tuple[tuple[str, str, float], ...]
    joint_positions: tuple[float, ...]
    passed: bool
    reason: str


@dataclass(frozen=True)
class SystemCoverageResult:
    target: ReachabilityTarget
    panda1: ReachabilityResult
    panda2: ReachabilityResult

    @property
    def reachable_by_any(self) -> bool:
        return self.panda1.passed or self.panda2.passed

    @property
    def reachable_by_both(self) -> bool:
        return self.panda1.passed and self.panda2.passed


def build_targets(config: ConfigBundle) -> tuple[ReachabilityTarget, ...]:
    targets: list[ReachabilityTarget] = []
    for robot_id, robot in config.robots.items():
        surfaces = [config.scene["surfaces"][surface_id] for surface_id in robot["reachable_surfaces"]]
        for station, surface in zip(("beginning", "middle", "end"), surfaces, strict=True):
            x, y, tabletop_z = map(float, surface["pose"]["position"])
            safe_width = float(surface["safe_place_region"]["size"][1])
            toward_center = -math.copysign(safe_width / 2.0, y)
            depths = {
                "near": y + toward_center,
                "center": y,
                "far": y - toward_center,
            }
            for depth, target_y in depths.items():
                targets.append(ReachabilityTarget(robot_id, station, depth, (x, target_y, tabletop_z + TARGET_CLEARANCE)))
    return tuple(targets)


def _joint_addresses(model: mujoco.MjModel, names: Iterable[str]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ids = np.asarray([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in names])
    qpos = np.asarray([model.jnt_qposadr[joint_id] for joint_id in ids])
    dofs = np.asarray([model.jnt_dofadr[joint_id] for joint_id in ids])
    limits = np.asarray([model.jnt_range[joint_id] for joint_id in ids], dtype=float)
    return qpos, dofs, limits


def _set_other_carriage(config: ConfigBundle, simulator: Simulator, robot_id: str) -> None:
    other_id = "panda2" if robot_id == "panda1" else "panda1"
    other_rail = config.robots[other_id]["rail"]
    parked = other_rail["upper_limit"] if robot_id == "panda1" else other_rail["lower_limit"]
    simulator.set_joint_position(other_rail["joint"], float(parked))


def solve_target(config: ConfigBundle, simulator: Simulator, target: ReachabilityTarget) -> ReachabilityResult:
    robot = config.robots[target.robot]
    rail_name = robot["rail"]["joint"]
    joint_names = [rail_name, *robot["arm"]["joint_names"]]
    qpos_addresses, dof_addresses, limits = _joint_addresses(simulator.model, joint_names)
    tcp_id = mujoco.mj_name2id(simulator.model, mujoco.mjtObj.mjOBJ_SITE, robot["gripper"]["tcp_frame"])
    target_xyz = np.asarray(target.xyz)
    rail_origin_x = float(config.scene["shared_rail"]["pose"]["position"][0])
    aligned_rail = np.clip(target.xyz[0] - rail_origin_x, limits[0, 0], limits[0, 1])
    arm_home = np.asarray(robot["arm"]["home_joints"], dtype=float)
    rng = np.random.default_rng(sum(map(ord, f"{target.robot}:{target.region}")))
    seeds = [np.concatenate(([aligned_rail], arm_home))]
    for _ in range(RANDOM_SEEDS):
        random_arm = rng.uniform(limits[1:, 0] + 0.08, limits[1:, 1] - 0.08)
        seeds.append(np.concatenate(([rng.uniform(limits[0, 0], limits[0, 1])], random_arm)))

    best_error = math.inf
    best_rail = float(aligned_rail)
    best_collisions: tuple[tuple[str, str, float], ...] = ()
    best_within_limits = False
    best_joint_positions: tuple[float, ...] = ()
    for seed in seeds:
        _set_other_carriage(config, simulator, target.robot)
        simulator.data.qpos[qpos_addresses] = seed
        for _ in range(MAX_ITERATIONS):
            simulator.forward()
            error_vector = target_xyz - simulator.data.site_xpos[tcp_id]
            error = float(np.linalg.norm(error_vector))
            if error < best_error:
                best_error = error
                best_rail = float(simulator.data.qpos[qpos_addresses[0]])
                best_collisions = tuple(simulator.penetrating_contacts())
                q = simulator.data.qpos[qpos_addresses]
                best_within_limits = bool(np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]))
                best_joint_positions = tuple(map(float, q))
            if error <= POSITION_TOLERANCE:
                collisions = tuple(simulator.penetrating_contacts())
                q = simulator.data.qpos[qpos_addresses]
                within_limits = bool(np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]))
                if within_limits and not collisions:
                    return ReachabilityResult(target, float(q[0]), True, error, True, (), tuple(map(float, q)), True, "")
                best_collisions = collisions
                best_within_limits = within_limits
                break
            jacobian = np.zeros((3, simulator.model.nv))
            mujoco.mj_jacSite(simulator.model, simulator.data, jacobian, None, tcp_id)
            reduced = jacobian[:, dof_addresses]
            step = reduced.T @ np.linalg.solve(reduced @ reduced.T + IK_DAMPING * np.eye(3), error_vector)
            lower = limits[:, 0] + JOINT_LIMIT_MARGIN
            upper = limits[:, 1] - JOINT_LIMIT_MARGIN
            lower[0], upper[0] = limits[0]
            simulator.data.qpos[qpos_addresses] = np.clip(simulator.data.qpos[qpos_addresses] + np.clip(step, -0.10, 0.10), lower, upper)

    converged = best_error <= POSITION_TOLERANCE
    if not converged:
        reason = f"IK error {best_error:.4f} m exceeds {POSITION_TOLERANCE:.4f} m tolerance"
    elif not best_within_limits:
        reason = "joint limit violation"
    elif best_collisions:
        pairs = ", ".join(f"{first}<->{second}" for first, second, _ in best_collisions[:3])
        reason = f"collision: {pairs}"
    else:
        reason = "no collision-free IK solution found"
    return ReachabilityResult(target, best_rail, converged, best_error, best_within_limits, best_collisions, best_joint_positions, False, reason)


def validate_workspace(config: ConfigBundle, simulator: Simulator) -> tuple[ReachabilityResult, ...]:
    return tuple(solve_target(config, simulator, target) for target in build_targets(config))


def validate_system_coverage(config: ConfigBundle, simulator: Simulator) -> tuple[SystemCoverageResult, ...]:
    coverage: list[SystemCoverageResult] = []
    for target in build_targets(config):
        panda1_target = ReachabilityTarget("panda1", target.station, target.depth, target.xyz)
        panda2_target = ReachabilityTarget("panda2", target.station, target.depth, target.xyz)
        coverage.append(SystemCoverageResult(target, solve_target(config, simulator, panda1_target), solve_target(config, simulator, panda2_target)))
    return tuple(coverage)


def format_report(results: Iterable[ReachabilityResult], system_coverage: Iterable[SystemCoverageResult] | None = None) -> str:
    rows = list(results)
    header = f"{'robot':7} {'region':20} {'rail q':>8} {'target XYZ':30} {'IK':4} {'limits':7} {'error m':>9} {'collision':10} {'result':6}"
    lines = [header, "-" * len(header)]
    for result in rows:
        xyz = "(" + ", ".join(f"{value:.3f}" for value in result.target.xyz) + ")"
        lines.append(f"{result.target.robot:7} {result.target.region:20} {result.rail_position:8.3f} {xyz:30} {'OK' if result.ik_converged else 'FAIL':4} {'within' if result.joints_within_limits else 'FAIL':7} {result.position_error:9.4f} {'clear' if not result.collisions else 'collision':10} {'PASS' if result.passed else 'FAIL':6}")
        if not result.passed:
            lines.append(f"  reason: {result.reason}")
    for robot_id in ("panda1", "panda2"):
        robot_rows = [result for result in rows if result.target.robot == robot_id]
        passed = sum(result.passed for result in robot_rows)
        lines.append(f"{robot_id}: {passed}/{len(robot_rows)} passed ({100.0 * passed / len(robot_rows):.1f}%)")
    passed = sum(result.passed for result in rows)
    lines.append(f"overall: {passed}/{len(rows)} passed ({100.0 * passed / len(rows):.1f}%)")
    if system_coverage is not None:
        system_rows = list(system_coverage)
        any_count = sum(result.reachable_by_any for result in system_rows)
        both_count = sum(result.reachable_by_both for result in system_rows)
        lines.append(f"system reachable by any robot: {any_count}/{len(system_rows)} ({100.0 * any_count / len(system_rows):.1f}%)")
        lines.append(f"system reachable by both robots: {both_count}/{len(system_rows)}")
        lines.append(f"system unreachable by both: {len(system_rows) - any_count}/{len(system_rows)}")
    return "\n".join(lines)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    config = ConfigLoader(root / "config").load()
    with SceneBuilder(config).build(headless=True) as simulator:
        results = validate_workspace(config, simulator)
        system_coverage = validate_system_coverage(config, simulator)
    print(format_report(results, system_coverage))
    return 0 if all(result.reachable_by_any for result in system_coverage) else 1


if __name__ == "__main__":
    raise SystemExit(main())
