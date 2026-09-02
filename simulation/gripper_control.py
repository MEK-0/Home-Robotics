"""Validated Franka Hand width control through the coupled MuJoCo tendon."""
from __future__ import annotations

import math

from .config_loader import ConfigBundle
from .simulator import Simulator


class GripperWidthController:
    """Own a rate-limited total opening-width target for one Franka Hand."""

    def __init__(
        self, config: ConfigBundle, simulator: Simulator, robot_id: str
    ) -> None:
        if robot_id not in ("panda1", "panda2"):
            raise ValueError(f"Unsupported gripper robot '{robot_id}'")
        self.config = config
        self.simulator = simulator
        self.robot_id = robot_id
        self.control_config = config.physics["robot"]["gripper_width_control"]
        per_finger = config.robots[robot_id]["gripper"]["opening_range_per_finger"]
        self.minimum_width = 2.0 * float(per_finger[0])
        self.maximum_width = 2.0 * float(per_finger[1])
        self.tolerance = float(self.control_config["width_limit_tolerance"])
        self.reset_to_home()

    def accept_target(self, requested_width: float) -> tuple[bool, str]:
        if not math.isfinite(requested_width):
            return False, "gripper width target must be finite"
        if (
            requested_width < self.minimum_width - self.tolerance
            or requested_width > self.maximum_width + self.tolerance
        ):
            return (
                False,
                f"gripper width {requested_width:.5f} m is outside "
                f"[{self.minimum_width:.5f}, {self.maximum_width:.5f}] m",
            )
        self.target_width = min(
            self.maximum_width, max(self.minimum_width, requested_width)
        )
        return True, ""

    def reset_to_home(self) -> None:
        home = self.config.robots[self.robot_id]["gripper"]["home_joints"]
        self.target_width = 2.0 * float(home[0])
        self.actuator_width = self.target_width
        self.simulator.set_actuator_control(
            f"{self.robot_id}_gripper_actuator", self.actuator_width / 2.0
        )

    def apply_target(self) -> None:
        maximum_step = (
            float(self.control_config["maximum_width_rate"])
            * float(self.config.physics["timestep"])
        )
        error = self.target_width - self.actuator_width
        self.actuator_width += max(-maximum_step, min(maximum_step, error))
        self.simulator.set_actuator_control(
            f"{self.robot_id}_gripper_actuator", self.actuator_width / 2.0
        )
