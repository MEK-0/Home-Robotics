"""Validated Panda arm target handling and MuJoCo actuator control."""
from __future__ import annotations

import math
from collections.abc import Sequence

from .config_loader import ConfigBundle
from .simulator import Simulator


class PandaArmPositionController:
    """Own atomic Panda arm targets and rate-limited actuator setpoints."""

    def __init__(
        self, config: ConfigBundle, simulator: Simulator, robot_id: str = "panda1"
    ) -> None:
        if robot_id not in ("panda1", "panda2"):
            raise ValueError(f"Unsupported arm robot '{robot_id}'")
        self.robot_id = robot_id
        self.config = config
        self.simulator = simulator
        arm = config.robots[self.robot_id]["arm"]
        self.joint_names = tuple(str(name) for name in arm["joint_names"])
        self.joint_limits = {
            name: tuple(map(float, limits))
            for name, limits in zip(
                self.joint_names, arm["joint_limits"], strict=True
            )
        }
        self.control_config = config.physics["robot"][
            f"{self.robot_id}_arm_position_control"
        ]
        self.limit_tolerance = float(
            self.control_config["joint_limit_tolerance"]
        )
        for name in self.joint_names:
            configured = self.joint_limits[name]
            actual = simulator.joint_range(name)
            if any(
                not math.isclose(
                    configured[index],
                    actual[index],
                    abs_tol=self.limit_tolerance,
                )
                for index in (0, 1)
            ):
                raise ValueError(
                    f"Configured limits for '{name}' do not match MuJoCo limits"
                )
        self.reset_to_home()

    def validate_command(
        self, names: Sequence[str], positions: Sequence[float]
    ) -> tuple[bool, str, dict[str, float] | None]:
        if len(names) != len(self.joint_names):
            return False, "command must contain exactly seven joint names", None
        if len(positions) != len(self.joint_names):
            return False, "command must contain exactly seven positions", None
        if len(set(names)) != len(names):
            return False, "command contains duplicate joint names", None

        expected = set(self.joint_names)
        received = set(names)
        unknown = sorted(received - expected)
        if unknown:
            return False, f"command contains unknown joints: {', '.join(unknown)}", None
        missing = sorted(expected - received)
        if missing:
            return False, f"command is missing joints: {', '.join(missing)}", None

        proposed: dict[str, float] = {}
        for name, raw_position in zip(names, positions, strict=True):
            if (
                not isinstance(raw_position, (int, float))
                or isinstance(raw_position, bool)
                or not math.isfinite(raw_position)
            ):
                return False, f"target for '{name}' must be finite", None
            position = float(raw_position)
            lower, upper = self.joint_limits[name]
            if (
                position < lower - self.limit_tolerance
                or position > upper + self.limit_tolerance
            ):
                return (
                    False,
                    f"target {position:.5f} for '{name}' is outside "
                    f"[{lower:.5f}, {upper:.5f}]",
                    None,
                )
            proposed[name] = min(upper, max(lower, position))
        return True, "", proposed

    def accept_command(
        self, names: Sequence[str], positions: Sequence[float]
    ) -> tuple[bool, str]:
        valid, reason, proposed = self.validate_command(names, positions)
        if valid:
            assert proposed is not None
            self.targets = proposed
        return valid, reason

    def reset_to_home(self) -> None:
        arm = self.config.robots[self.robot_id]["arm"]
        self.targets = {
            name: float(home)
            for name, home in zip(
                self.joint_names, arm["home_joints"], strict=True
            )
        }
        self.actuator_targets = dict(self.targets)
        for index, name in enumerate(self.joint_names, start=1):
            self.simulator.set_actuator_control(
                f"{self.robot_id}_joint{index}_actuator",
                self.actuator_targets[name],
            )

    def apply_targets(self) -> None:
        timestep = float(self.config.physics["timestep"])
        rates = self.control_config["maximum_target_rates"]
        for index, name in enumerate(self.joint_names, start=1):
            maximum_step = float(rates[index - 1]) * timestep
            error = self.targets[name] - self.actuator_targets[name]
            increment = max(-maximum_step, min(maximum_step, error))
            self.actuator_targets[name] += increment
            self.simulator.set_actuator_control(
                f"{self.robot_id}_joint{index}_actuator",
                self.actuator_targets[name],
            )
