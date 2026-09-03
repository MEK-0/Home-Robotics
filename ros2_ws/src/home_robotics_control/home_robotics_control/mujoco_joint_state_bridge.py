from pathlib import Path
import signal
import sys
import time

from ament_index_python.packages import get_package_share_directory
import mujoco
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


PROJECT_ROOT = Path(get_package_share_directory('home_robotics_control'))
sys.path.insert(0, str(PROJECT_ROOT))

from simulation.arm_control import PandaArmPositionController  # noqa: E402
from simulation.config_loader import ConfigLoader  # noqa: E402
from simulation.gripper_control import GripperWidthController  # noqa: E402
from simulation.rail_control import RailTargetController  # noqa: E402
from simulation.reset_manager import ResetManager  # noqa: E402
from simulation.scene_builder import SceneBuilder  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


class MujocoJointStateBridge(Node):

    def __init__(self):
        super().__init__('mujoco_joint_state_bridge')

        self.declare_parameter('use_viewer', False)
        self.use_viewer = bool(self.get_parameter('use_viewer').value)
        self.viewer = None

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        self.config = ConfigLoader(PROJECT_ROOT / 'config').load()
        self.scene_context = SceneBuilder(self.config).build()
        self.sim = self.scene_context.__enter__()
        self.model = self.sim.model
        self.data = self.sim.data

        if self.use_viewer:
            from mujoco import viewer as mujoco_viewer

            self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
            self.get_logger().info('MuJoCo passive viewer started')

        self.rail_controller = RailTargetController(self.config, self.sim)
        self.panda1_rail_target = self.rail_controller.target('panda1')
        self.panda2_rail_target = self.rail_controller.target('panda2')
        self.arm_controller = PandaArmPositionController(self.config, self.sim)
        self.panda1_arm_targets = dict(self.arm_controller.targets)
        self.panda2_arm_controller = PandaArmPositionController(
            self.config, self.sim, 'panda2'
        )
        self.panda2_arm_targets = dict(self.panda2_arm_controller.targets)
        self.gripper_controllers = {
            robot_id: GripperWidthController(self.config, self.sim, robot_id)
            for robot_id in ('panda1', 'panda2')
        }

        self.reset_manager = ResetManager(
            self.sim,
            config=self.config,
            settle_steps=self.config.physics['reset']['settle_steps'],
        )
        self.reset_manager.add_state_reset_hook(self._reset_control_targets)
        self.reset_service = self.create_service(
            Trigger, '/reset_simulation', self._reset_simulation
        )

        self.panda1_rail_subscription = self.create_subscription(
            Float64, '/panda1/rail_command', self._panda1_rail_command, 10
        )
        self.panda2_rail_subscription = self.create_subscription(
            Float64, '/panda2/rail_command', self._panda2_rail_command, 10
        )
        self.panda1_arm_subscription = self.create_subscription(
            JointState, '/panda1/arm_joint_command',
            self._panda1_arm_joint_command, 10,
        )
        self.panda2_arm_subscription = self.create_subscription(
            JointState, '/panda2/arm_joint_command',
            self._panda2_arm_joint_command, 10,
        )
        self.panda1_gripper_subscription = self.create_subscription(
            Float64, '/panda1/gripper_width_command',
            lambda msg: self._gripper_width_command('panda1', msg), 10,
        )
        self.panda2_gripper_subscription = self.create_subscription(
            Float64, '/panda2/gripper_width_command',
            lambda msg: self._gripper_width_command('panda2', msg), 10,
        )

        self.joint_names = [
            'panda1_rail_joint',
            'panda1_joint1',
            'panda1_joint2',
            'panda1_joint3',
            'panda1_joint4',
            'panda1_joint5',
            'panda1_joint6',
            'panda1_joint7',
            'panda1_finger_joint1',
            'panda1_finger_joint2',
            'panda2_rail_joint',
            'panda2_joint1',
            'panda2_joint2',
            'panda2_joint3',
            'panda2_joint4',
            'panda2_joint5',
            'panda2_joint6',
            'panda2_joint7',
            'panda2_finger_joint1',
            'panda2_finger_joint2',
        ]

        self.timer_period = 0.02
        self.physics_steps_per_update = max(
            1, round(self.timer_period / float(self.config.physics['timestep']))
        )
        self.timer = self.create_timer(self.timer_period, self.update)

        self.get_logger().info(
            f'MuJoCo joint state bridge started with {len(self.joint_names)} joints; '
            f'rail targets panda1={self.panda1_rail_target:.3f} m, '
            f'panda2={self.panda2_rail_target:.3f} m'
        )

    def _panda1_rail_command(self, msg):
        self._accept_rail_command('panda1', float(msg.data))

    def _panda2_rail_command(self, msg):
        self._accept_rail_command('panda2', float(msg.data))

    def _accept_rail_command(self, robot_id, requested):
        accepted, reason = self.rail_controller.accept_target(robot_id, requested)
        if not accepted:
            self.get_logger().warning(
                f'Rejected {robot_id} rail command: {reason}; keeping previous target'
            )
            return
        setattr(self, f'{robot_id}_rail_target', requested)
        self.get_logger().info(f'Accepted {robot_id} rail target: {requested:.3f} m')

    def _panda1_arm_joint_command(self, msg):
        accepted, reason = self.arm_controller.accept_command(
            msg.name, msg.position
        )
        if not accepted:
            self.get_logger().warning(
                f'Rejected Panda 1 arm command: {reason}; keeping previous targets'
            )
            return
        self.panda1_arm_targets = dict(self.arm_controller.targets)
        self.get_logger().info('Accepted Panda 1 arm joint command')

    def _panda2_arm_joint_command(self, msg):
        accepted, reason = self.panda2_arm_controller.accept_command(
            msg.name, msg.position
        )
        if not accepted:
            self.get_logger().warning(
                f'Rejected Panda 2 arm command: {reason}; keeping previous targets'
            )
            return
        self.panda2_arm_targets = dict(self.panda2_arm_controller.targets)
        self.get_logger().info('Accepted Panda 2 arm joint command')

    def _gripper_width_command(self, robot_id, msg):
        controller = self.gripper_controllers[robot_id]
        accepted, reason = controller.accept_target(float(msg.data))
        if not accepted:
            self.get_logger().warning(
                f'Rejected {robot_id} gripper command: {reason}; keeping previous target'
            )
            return
        self.get_logger().info(
            f'Accepted {robot_id} gripper width target: {float(msg.data):.3f} m'
        )

    def _reset_control_targets(self, _simulator):
        self.rail_controller.reset_to_home()
        self.arm_controller.reset_to_home()
        self.panda2_arm_controller.reset_to_home()
        for controller in self.gripper_controllers.values():
            controller.reset_to_home()

    def _reset_simulation(self, _request, response):
        """Reset the existing MuJoCo instance from the serialized ROS callback."""
        try:
            self.reset_manager.reset()
            self.panda1_rail_target = self.rail_controller.target('panda1')
            self.panda2_rail_target = self.rail_controller.target('panda2')
            self.panda1_arm_targets = dict(self.arm_controller.targets)
            self.panda2_arm_targets = dict(self.panda2_arm_controller.targets)
            if self.viewer is not None and self.viewer.is_running():
                self.viewer.sync()
            response.success = True
            response.message = (
                'Simulation reset to validated home state; rail, both arm, and both '
                'gripper targets synchronized '
                f'to panda1={self.panda1_rail_target:.3f} m and '
                f'panda2={self.panda2_rail_target:.3f} m'
            )
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f'Simulation reset failed: {exc}'
            self.get_logger().error(response.message)
        return response

    def update(self):
        for _ in range(self.physics_steps_per_update):
            self.rail_controller.apply_targets()
            self.arm_controller.apply_targets()
            self.panda2_arm_controller.apply_targets()
            for controller in self.gripper_controllers.values():
                controller.apply_target()
            mujoco.mj_step(self.model, self.data)

        if self.viewer is not None:
            if not self.viewer.is_running():
                self.get_logger().info('MuJoCo viewer closed; shutting down ROS node')
                if rclpy.ok():
                    rclpy.shutdown()
                return
            self.viewer.sync()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for joint_name in self.joint_names:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            if joint_id == -1:
                self.get_logger().error(f'Joint not found: {joint_name}')
                continue

            qpos_address = self.model.jnt_qposadr[joint_id]
            dof_address = self.model.jnt_dofadr[joint_id]
            msg.name.append(joint_name)
            msg.position.append(float(self.data.qpos[qpos_address]))
            msg.velocity.append(float(self.data.qvel[dof_address]))

        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.viewer is not None:
            viewer = self.viewer
            self.viewer = None
            viewer.close()
            deadline = time.monotonic() + 2.0
            while viewer.is_running() and time.monotonic() < deadline:
                time.sleep(0.01)
        self.scene_context.__exit__(None, None, None)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MujocoJointStateBridge()
    shutdown_requested = False

    def request_shutdown(_signum, _frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    previous_sigint_handler = signal.signal(signal.SIGINT, request_shutdown)
    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        signal.signal(signal.SIGINT, previous_sigint_handler)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
