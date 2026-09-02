"""Shared-rail target validation and MuJoCo actuator control."""
from __future__ import annotations

import math

from .config_loader import ConfigBundle
from .simulator import Simulator


class RailTargetController:
    """Own the two valid rail targets and apply them through MuJoCo actuators."""

    ROBOT_IDS = ("panda1", "panda2")

    def __init__(self, config: ConfigBundle, simulator: Simulator) -> None:
        self.config = config
        self.simulator = simulator
        self.minimum_separation = float(
            config.scene["shared_rail"]["minimum_carriage_separation"]
        )
        self.targets = {
            robot_id: float(config.robots[robot_id]["rail"]["home_position"])
            for robot_id in self.ROBOT_IDS
        }
        self.actuator_targets = dict(self.targets)
        self.apply_targets()

    def target(self, robot_id: str) -> float:
        return self.targets[robot_id]

    def validate_target(self, robot_id: str, requested: float) -> tuple[bool, str]:
        if robot_id not in self.ROBOT_IDS:
            return False, f"unknown rail robot '{robot_id}'"
        if not math.isfinite(requested):
            return False, "target must be finite"
        rail = self.config.robots[robot_id]["rail"]
        lower, upper = float(rail["lower_limit"]), float(rail["upper_limit"])
        if not lower <= requested <= upper:
            return False, (
                f"target {requested:.3f} m is outside joint limits "
                f"[{lower:.3f}, {upper:.3f}] m"
            )
        proposed = dict(self.targets)
        proposed[robot_id] = requested
        separation = proposed["panda2"] - proposed["panda1"]
        if separation < self.minimum_separation:
            return False, (
                f"target {requested:.3f} m violates panda1-before-panda2 ordering/"
                f"minimum separation {self.minimum_separation:.3f} m "
                f"(proposed separation {separation:.3f} m)"
            )
        return True, ""

    def accept_target(self, robot_id: str, requested: float) -> tuple[bool, str]:
        valid, reason = self.validate_target(robot_id, requested)
        if valid:
            self.targets[robot_id] = requested
        return valid, reason

    def reset_to_home(self) -> None:
        """Synchronize command and actuator targets to validated rail homes."""
        self.targets = {
            robot_id: float(
                self.config.robots[robot_id]["rail"]["home_position"]
            )
            for robot_id in self.ROBOT_IDS
        }
        self.actuator_targets = dict(self.targets)
        for robot_id in self.ROBOT_IDS:
            self.simulator.set_actuator_control(
                f"{robot_id}_rail_actuator", self.actuator_targets[robot_id]
            )

    def apply_targets(self) -> None:
        timestep = float(self.config.physics["timestep"])
        for robot_id in self.ROBOT_IDS:
            maximum_step = float(
                self.config.robots[robot_id]["rail"]["max_velocity"]
            ) * timestep
            error = self.targets[robot_id] - self.actuator_targets[robot_id]
            increment = max(-maximum_step, min(maximum_step, error))
            self.actuator_targets[robot_id] += increment
            self.simulator.set_actuator_control(
                f"{robot_id}_rail_actuator", self.actuator_targets[robot_id]
            )
