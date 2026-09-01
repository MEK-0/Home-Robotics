"""Deterministic reset orchestration, extensible with ordered reset hooks."""
from __future__ import annotations
from collections.abc import Callable
from .config_loader import ConfigBundle
from .simulator import Simulator
ResetHook = Callable[[Simulator], None]

class ResetManager:
    def __init__(self, simulator: Simulator, *, config: ConfigBundle | None = None, settle_steps: int = 0) -> None:
        if settle_steps < 0:
            raise ValueError("settle_steps must be non-negative")
        self.simulator = simulator
        self.settle_steps = settle_steps
        self.rail_home_positions = config.rail_home_positions if config is not None else {}
        self.robot_home_positions = config.robot_home_positions if config is not None else {}
        self.reset_joint_positions = {**self.rail_home_positions, **self.robot_home_positions}
        self._pre_reset_hooks: list[ResetHook] = []
        self._state_reset_hooks: list[ResetHook] = []

    def add_pre_reset_hook(self, hook: ResetHook) -> None:
        self._pre_reset_hooks.append(hook)

    def add_state_reset_hook(self, hook: ResetHook) -> None:
        self._state_reset_hooks.append(hook)

    def reset(self) -> None:
        for hook in self._pre_reset_hooks:
            hook(self.simulator)
        self.simulator.reset()
        self.simulator.set_joint_positions(self.rail_home_positions)
        self.simulator.set_joint_positions(self.robot_home_positions)
        for joint_name in self.reset_joint_positions:
            self.simulator.set_joint_velocity(joint_name, 0.0)
        for hook in self._state_reset_hooks:
            hook(self.simulator)
        self.simulator.forward()
        if self.settle_steps:
            self.simulator.step(self.settle_steps)
