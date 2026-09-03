"""Start the Phase 2 controllers with lifecycle readiness checks."""

import time

from controller_manager import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
)
from controller_manager_msgs.srv import SwitchController
import rclpy
from rclpy.node import Node


CONTROLLERS = (
    'joint_state_broadcaster',
    'panda1_trajectory_controller',
    'panda2_trajectory_controller',
    'panda1_gripper_controller',
    'panda2_gripper_controller',
)


class ControllerStartup(Node):
    """Own the ordered load, configure, and activate lifecycle sequence."""

    def __init__(self):
        super().__init__('phase2_controller_startup')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('transition_timeout', 15.0)
        self.controller_manager = str(self.get_parameter('controller_manager').value)
        self.transition_timeout = float(self.get_parameter('transition_timeout').value)

    def controller_states(self):
        """Return the latest controller lifecycle states by name."""
        response = list_controllers(
            self,
            self.controller_manager,
            service_timeout=self.transition_timeout,
            call_timeout=self.transition_timeout,
        )
        return {controller.name: controller.state for controller in response.controller}

    def wait_for_state(self, name, expected_states, expected_names):
        """Wait until a controller reaches one of the requested states."""
        deadline = time.monotonic() + self.transition_timeout
        while time.monotonic() < deadline:
            states = self.controller_states()
            state = states.get(name)
            if set(states) == set(expected_names) and state in expected_states:
                return state
        states = self.controller_states()
        raise RuntimeError(
            f'Controller {name} did not reach {sorted(expected_states)}; '
            f'current state is {states.get(name, "absent")}'
        )

    def start_controller(self, name, previously_started):
        """Bring one controller to active without overlapping ownership."""
        expected_names = (*previously_started, name)
        states = self.wait_for_controller_set(previously_started)
        state = states.get(name)
        if state is None:
            load_controller(
                self,
                self.controller_manager,
                name,
                service_timeout=self.transition_timeout,
                call_timeout=self.transition_timeout,
            )
            state = self.wait_for_state(
                name, {'unconfigured', 'inactive', 'active'}, expected_names
            )

        if state == 'unconfigured':
            configure_controller(
                self,
                self.controller_manager,
                name,
                service_timeout=self.transition_timeout,
                call_timeout=self.transition_timeout,
            )
            state = self.wait_for_state(name, {'inactive', 'active'}, expected_names)

        if state == 'inactive':
            switch_controllers(
                self,
                self.controller_manager,
                [],
                [name],
                SwitchController.Request.STRICT,
                True,
                self.transition_timeout,
                self.transition_timeout,
            )
            state = self.wait_for_state(name, {'active'}, expected_names)

        if state != 'active':
            raise RuntimeError(f'Controller {name} has unsupported state {state}')
        self.get_logger().info(f'Controller active: {name}')

    def wait_for_empty_manager(self):
        """Wait for a stable empty list from the newly launched manager."""
        deadline = time.monotonic() + self.transition_timeout
        consecutive_empty_samples = 0
        while time.monotonic() < deadline:
            if self.controller_states():
                consecutive_empty_samples = 0
                continue
            consecutive_empty_samples += 1
            if consecutive_empty_samples == 3:
                self.get_logger().info('Controller manager ownership established')
                return
        raise RuntimeError(
            'Controller manager did not present a stable empty startup state'
        )

    def wait_for_controller_set(self, expected_names):
        """Wait for an exact controller set from the owned manager instance."""
        deadline = time.monotonic() + self.transition_timeout
        expected = set(expected_names)
        while time.monotonic() < deadline:
            states = self.controller_states()
            if set(states) == expected:
                return states
        raise RuntimeError(
            f'Controller manager did not present expected set {sorted(expected)}'
        )

    def start_all(self):
        """Start every Phase 2 controller in the declared order."""
        self.wait_for_empty_manager()
        for index, name in enumerate(CONTROLLERS):
            self.start_controller(name, CONTROLLERS[:index])
        states = self.wait_for_controller_set(CONTROLLERS)
        if any(states.get(name) != 'active' for name in CONTROLLERS):
            raise RuntimeError(f'Final controller states are not all active: {states}')
        self.get_logger().info('All Phase 2 controllers are active')


def main(args=None):
    """Run the deterministic controller startup sequence."""
    rclpy.init(args=args)
    node = ControllerStartup()
    exit_code = 0
    try:
        node.start_all()
    except Exception as error:
        node.get_logger().fatal(f'Controller startup failed: {error}')
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)
