from pathlib import Path
import sys

import mujoco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


PROJECT_ROOT = Path.home() / "Home-Robotics"
sys.path.insert(0, str(PROJECT_ROOT))

from simulation.config_loader import ConfigLoader
from simulation.scene_builder import SceneBuilder


class MujocoJointStateBridge(Node):

    def __init__(self):
        super().__init__("mujoco_joint_state_bridge")

        self.publisher_ = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        config = ConfigLoader(PROJECT_ROOT / "config").load()

        self.scene_context = SceneBuilder(config).build()
        self.sim = self.scene_context.__enter__()

        self.model = self.sim.model
        self.data = self.sim.data

        self.joint_names = [
            "panda1_rail_joint",

            "panda1_joint1",
            "panda1_joint2",
            "panda1_joint3",
            "panda1_joint4",
            "panda1_joint5",
            "panda1_joint6",
            "panda1_joint7",

            "panda1_finger_joint1",
            "panda1_finger_joint2",

            "panda2_rail_joint",

            "panda2_joint1",
            "panda2_joint2",
            "panda2_joint3",
            "panda2_joint4",
            "panda2_joint5",
            "panda2_joint6",
            "panda2_joint7",

            "panda2_finger_joint1",
            "panda2_finger_joint2",
        ]

        self.timer = self.create_timer(
            0.02,
            self.update,
        )

        self.get_logger().info(
            f"MuJoCo joint state bridge started with "
            f"{len(self.joint_names)} joints"
        )

    def update(self):
        mujoco.mj_step(self.model, self.data)

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        for joint_name in self.joint_names:

            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )

            if joint_id == -1:
                self.get_logger().error(
                    f"Joint not found: {joint_name}"
                )
                continue

            qpos_address = self.model.jnt_qposadr[joint_id]
            dof_address = self.model.jnt_dofadr[joint_id]

            msg.name.append(joint_name)
            msg.position.append(
                float(self.data.qpos[qpos_address])
            )
            msg.velocity.append(
                float(self.data.qvel[dof_address])
            )

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.scene_context.__exit__(
            None,
            None,
            None,
        )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MujocoJointStateBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()