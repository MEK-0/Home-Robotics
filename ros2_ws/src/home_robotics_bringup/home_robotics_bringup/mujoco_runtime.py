"""Authoritative MuJoCo runtime adapter for the Phase 2 ros2_control stack."""
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
import json
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
        self.object_publishers = {
            object_id: self.create_publisher(
                PoseStamped, f"/mujoco/objects/{object_id}/pose", 1
            )
            for object_id, metadata in self.config.objects.items()
            if metadata["planning_scene_enabled"]
        }
        self.object_timer = self.create_timer(0.05, self._publish_object_poses)
        # Opt-in fixed validation fixture, not an arbitrary pose/teleport API.
        self.debug_shifted = False
        if self.declare_parameter("enable_scene_validation", False).value:
            self.debug_service = self.create_service(
                Trigger, "/validation/shift_cube", self._shift_cube_for_validation
            )
        from simulation.grasp_contact import CubeContact
        self.cube_contact = CubeContact(self.model, self.data, self.config)
        self.contact_publisher = self.create_publisher(String, "/mujoco/objects/cube/contact", 10)
        self.contact_services = [
            self.create_service(Trigger, "/mujoco/cube/begin_grasp", self._begin_grasp),
            self.create_service(Trigger, "/mujoco/cube/verify_grasp", self._verify_grasp),
            self.create_service(SetBool, "/mujoco/cube/stabilize", self._stabilize),
            self.create_service(Trigger, "/mujoco/cube/clear_dropped",
                                lambda request, response: self._contact_call(response, self.cube_contact.clear_dropped)),
        ]
        self.ball_contact = CubeContact(self.model, self.data, self.config, "purple_ball")
        self.ball_contact_publisher = self.create_publisher(String, "/mujoco/objects/purple_ball/contact", 10)
        for suffix, service, operation in (
            ("begin_grasp", Trigger, lambda request: self.ball_contact.begin()),
            ("verify_grasp", Trigger, lambda request: self.ball_contact.verify()),
            ("stabilize", SetBool, lambda request: self.ball_contact.stabilize(request.data)),
            ("clear_dropped", Trigger, lambda request: self.ball_contact.clear_dropped()),
        ):
            def callback(request, response, operation=operation):
                try:
                    operation(request)
                    response.success = True
                    response.message = json.dumps(self.ball_contact.observe())
                except ValueError as exc:
                    response.success = False
                    response.message = str(exc)
                return response
            self.contact_services.append(self.create_service(service, "/mujoco/purple_ball/" + suffix, callback))
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

    def _contact_call(self, response, operation):
        try:
            operation()
            response.success = True
            response.message = json.dumps(self.cube_contact.observe())
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _begin_grasp(self, request, response):
        return self._contact_call(response, self.cube_contact.begin)

    def _verify_grasp(self, request, response):
        return self._contact_call(response, self.cube_contact.verify)

    def _stabilize(self, request, response):
        return self._contact_call(response, lambda: self.cube_contact.stabilize(request.data))

    def _publish_object_poses(self):
        for object_id, publisher in self.object_publishers.items():
            message = PoseStamped()
            message.header.frame_id = "world"
            seconds = int(self.data.time)
            message.header.stamp.sec = seconds
            message.header.stamp.nanosec = int((float(self.data.time) - seconds) * 1e9)
            body = self.data.body(object_id)
            message.pose.position.x, message.pose.position.y, message.pose.position.z = (
                float(value) for value in body.xpos
            )
            w, x, y, z = (float(value) for value in body.xquat)
            message.pose.orientation.w = w
            message.pose.orientation.x = x
            message.pose.orientation.y = y
            message.pose.orientation.z = z
            publisher.publish(message)

    def _shift_cube_for_validation(self, _request, response):
        if self.debug_shifted:
            response.success = False
            response.message = "Fixture already used; reset simulation before repeating"
            return response
        body = self.data.body("cube")
        position = tuple(float(value) for value in body.xpos)
        quaternion = tuple(float(value) for value in body.xquat)
        self.sim.set_free_joint_pose(
            "cube_free_joint", (position[0] + 0.05, position[1], position[2]), quaternion
        )
        self.sim.set_free_joint_velocity("cube_free_joint", (0, 0, 0, 0, 0, 0))
        self.sim.forward()
        self.debug_shifted = True
        self._publish_object_poses()
        response.success = True
        response.message = "VALIDATION ONLY: cube shifted +0.05 m X in existing MuJoCo data"
        return response

    def _reset_simulation(self, request, response):
        cancel = CancelGoal.Request()
        for client in self.cancel_clients:
            if client.service_is_ready():
                client.call_async(cancel)
        self.ignore_commands_until = time.monotonic() + 0.5
        self.cube_contact.reset()
        self.ball_contact.reset()
        result = super()._reset_simulation(request, response)
        if result.success:
            self.debug_shifted = False
            self._publish_object_poses()
        return result

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
        self.contact_publisher.publish(String(data=json.dumps(self.cube_contact.observe())))
        self.ball_contact_publisher.publish(String(data=json.dumps(self.ball_contact.observe())))


def main(args=None):
    """Run exactly one MuJoCo model/data owner."""
    import home_robotics_control.mujoco_joint_state_bridge as module

    original = module.MujocoJointStateBridge
    module.MujocoJointStateBridge = Phase2MujocoRuntime
    try:
        bridge_main(args=args)
    finally:
        module.MujocoJointStateBridge = original
