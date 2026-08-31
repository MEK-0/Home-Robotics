"""Deterministic reset orchestration, extensible with ordered reset hooks."""
from __future__ import annotations
from collections.abc import Callable
from .simulator import Simulator
ResetHook = Callable[[Simulator], None]
class ResetManager:
    def __init__(self, simulator: Simulator, *, settle_steps: int = 0) -> None:
        if settle_steps < 0: raise ValueError("settle_steps must be non-negative")
        self.simulator, self.settle_steps = simulator, settle_steps
        self._pre_reset_hooks: list[ResetHook] = []
        self._state_reset_hooks: list[ResetHook] = []
    def add_pre_reset_hook(self, hook: ResetHook) -> None: self._pre_reset_hooks.append(hook)
    def add_state_reset_hook(self, hook: ResetHook) -> None: self._state_reset_hooks.append(hook)
    def reset(self) -> None:
        for hook in self._pre_reset_hooks: hook(self.simulator)
        self.simulator.reset()
        for hook in self._state_reset_hooks: hook(self.simulator)
        if self.settle_steps: self.simulator.step(self.settle_steps)
