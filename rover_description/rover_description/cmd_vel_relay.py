#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    """Forward standard velocity commands to the diff-drive controller."""

    def __init__(self):
        super().__init__('cmd_vel_relay')

        self.publisher = self.create_publisher(
            Twist,
            '/diff_drive_base_controller/cmd_vel_unstamped',
            10,
        )

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.relay_callback,
            10,
        )

        self.get_logger().info(
            'Relaying /cmd_vel to '
            '/diff_drive_base_controller/cmd_vel_unstamped'
        )

    def relay_callback(self, msg: Twist):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()