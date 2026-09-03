"""Authoritative MuJoCo runtime adapter for the Phase 2 ros2_control stack."""
from sensor_msgs.msg import JointState
from action_msgs.srv import CancelGoal
import time
from rosgraph_msgs.msg import Clock

from home_robotics_control.mujoco_joint_state_bridge import MujocoJointStateBridge
from home_robotics_control.mujoco_joint_state_bridge import main as bridge_main


class Phase2MujocoRuntime(MujocoJointStateBridge):
    """Own one MuJoCo instance and adapt ros2_control commands and simulation time."""

    def __init__(self):
        super().__init__()
        self.clock_publisher = self.create_publisher(Clock, "/clock", 10)
        self.control_subscription = self.create_subscription(
            JointState, "/mujoco/command", self._control_command, 10
        )
        self.ignore_commands_until = 0.0
        self.cancel_clients = [
            self.create_client(CancelGoal, f"/{name}/_action/cancel_goal")
            for name in (
                "panda1_trajectory_controller/follow_joint_trajectory",
                "panda2_trajectory_controller/follow_joint_trajectory",
                "panda1_gripper_controller/gripper_cmd",
                "panda2_gripper_controller/gripper_cmd",
            )
        ]

    def _reset_simulation(self, request, response):
        cancel = CancelGoal.Request()
        for client in self.cancel_clients:
            if client.service_is_ready():
                client.call_async(cancel)
        self.ignore_commands_until = time.monotonic() + 0.5
        return super()._reset_simulation(request, response)

    def _control_command(self, message):
        if time.monotonic() < self.ignore_commands_until:
            return
        if len(message.name) != len(message.position):
            self.get_logger().warning("Rejected malformed ros2_control command")
            return
        command = dict(zip(message.name, message.position, strict=True))
        required = set(self.joint_names) - {"panda1_finger_joint2", "panda2_finger_joint2"}
        if not required.issubset(command):
            self.get_logger().warning("Rejected incomplete ros2_control command")
            return
        rail_targets = {name: command[f"{name}_rail_joint"] for name in ("panda1", "panda2")}
        separation = rail_targets["panda2"] - rail_targets["panda1"]
        if separation < self.rail_controller.minimum_separation:
            self.get_logger().warning("Rejected unsafe ros2_control rail command")
            return
        for robot_id, value in rail_targets.items():
            rail = self.config.robots[robot_id]["rail"]
            if not float(rail["lower_limit"]) <= value <= float(rail["upper_limit"]):
                self.get_logger().warning(f"Rejected out-of-range {robot_id} rail command")
                return
        for robot_id, controller in (("panda1", self.arm_controller), ("panda2", self.panda2_arm_controller)):
            positions = [command[name] for name in controller.joint_names]
            valid, reason, _ = controller.validate_command(controller.joint_names, positions)
            if not valid:
                self.get_logger().warning(f"Rejected {robot_id} arm command: {reason}")
                return
        for robot_id in ("panda1", "panda2"):
            controller = self.gripper_controllers[robot_id]
            width = 2.0 * command[f"{robot_id}_finger_joint1"]
            if not controller.minimum_width <= width <= controller.maximum_width:
                self.get_logger().warning(f"Rejected {robot_id} gripper command")
                return
        self.rail_controller.targets.update(rail_targets)
        for controller in (self.arm_controller, self.panda2_arm_controller):
            controller.accept_command(controller.joint_names, [command[name] for name in controller.joint_names])
        for robot_id in ("panda1", "panda2"):
            self.gripper_controllers[robot_id].accept_target(2.0 * command[f"{robot_id}_finger_joint1"])

    def update(self):
        super().update()
        message = Clock()
        seconds = int(self.data.time)
        message.clock.sec = seconds
        message.clock.nanosec = int((float(self.data.time) - seconds) * 1_000_000_000)
        self.clock_publisher.publish(message)


def main(args=None):
    """Run exactly one MuJoCo model/data owner."""
    import home_robotics_control.mujoco_joint_state_bridge as module

    original = module.MujocoJointStateBridge
    module.MujocoJointStateBridge = Phase2MujocoRuntime
    try:
        bridge_main(args=args)
    finally:
        module.MujocoJointStateBridge = original
