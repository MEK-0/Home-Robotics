#!/usr/bin/env python3
"""Animate successful Phase 1 rail-plus-arm reachability solutions."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco
import mujoco.viewer
import numpy as np

from simulation.config_loader import ConfigLoader
from simulation.scene_builder import SceneBuilder
from simulation.workspace_reachability import validate_workspace


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transition", type=float, default=1.5, help="seconds used to move between targets")
    parser.add_argument("--hold", type=float, default=0.7, help="seconds to hold each reached target")
    return parser.parse_args()


def _qpos_addresses(model: mujoco.MjModel, names: list[str]) -> np.ndarray:
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in names]
    return np.asarray([model.jnt_qposadr[joint_id] for joint_id in joint_ids])


def main() -> int:
    args = _arguments()
    if args.transition <= 0 or args.hold < 0:
        raise SystemExit("--transition must be positive and --hold must be non-negative")

    root = Path(__file__).resolve().parents[1]
    config = ConfigLoader(root / "config").load()
    print("Preparing IK solutions...")
    with SceneBuilder(config).build(headless=True) as solver:
        results = validate_workspace(config, solver)
    successful = [result for result in results if result.passed]
    print(f"{len(successful)}/{len(results)} successful targets will be animated.")

    with SceneBuilder(config).build(headless=False) as simulator:
        model, data = simulator.model, simulator.data
        with mujoco.viewer.launch_passive(model, data) as viewer:
            for result in successful:
                if not viewer.is_running():
                    break
                robot_id = result.target.robot
                robot = config.robots[robot_id]
                other_id = "panda2" if robot_id == "panda1" else "panda1"
                other_rail = config.robots[other_id]["rail"]
                parked = other_rail["upper_limit"] if robot_id == "panda1" else other_rail["lower_limit"]
                simulator.set_joint_position(other_rail["joint"], float(parked))

                names = [robot["rail"]["joint"], *robot["arm"]["joint_names"]]
                addresses = _qpos_addresses(model, names)
                start = data.qpos[addresses].copy()
                goal = np.asarray(result.joint_positions)
                print(
                    f"{robot_id} | {result.target.region:20} | "
                    f"rail={result.rail_position:+.3f} m | target={result.target.xyz} | "
                    f"error={result.position_error * 1000:.1f} mm"
                )
                started = time.monotonic()
                while viewer.is_running():
                    alpha = min(1.0, (time.monotonic() - started) / args.transition)
                    smooth = alpha * alpha * (3.0 - 2.0 * alpha)
                    data.qpos[addresses] = start + smooth * (goal - start)
                    data.qvel[:] = 0.0
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                    if alpha >= 1.0:
                        break
                    time.sleep(1.0 / 60.0)
                hold_until = time.monotonic() + args.hold
                while viewer.is_running() and time.monotonic() < hold_until:
                    viewer.sync()
                    time.sleep(1.0 / 60.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())