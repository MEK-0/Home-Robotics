import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RailStatePublisher(Node):
    def __init__(self):
        super().__init__('rail_state_publisher')

        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_joint_state
        )

    def publish_joint_state(self):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['panda1_rail_joint']
        msg.position = [0.0]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = RailStatePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
