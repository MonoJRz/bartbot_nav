#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToPlatformTwist(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_platform_twist')

        self.declare_parameter('in_topic', '/cmd_vel')
        self.declare_parameter('out_topic', '/platform/twist')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 4.0)
        self.declare_parameter('max_reverse', 0.4)
        self.declare_parameter('enable_gate', False)
        self.declare_parameter('w_gate', 0.10)
        self.declare_parameter('v_gate', 0.10)

        self.in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.lin_scale = self.get_parameter('linear_scale').value
        self.ang_scale = self.get_parameter('angular_scale').value
        self.max_v = self.get_parameter('max_linear').value
        self.max_w = self.get_parameter('max_angular').value
        self.max_reverse = self.get_parameter('max_reverse').value
        self.enable_gate = self.get_parameter('enable_gate').value
        self.w_gate = self.get_parameter('w_gate').value
        self.v_gate = self.get_parameter('v_gate').value

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.cb, 10)

        self.get_logger().info(
            f'cmd_vel bridge+gate: {self.in_topic} -> {self.out_topic} '
            f'(gate={self.enable_gate}, w_gate={self.w_gate:.3f}, v_gate={self.v_gate:.3f})'
        )

    def cb(self, msg: Twist):
        out = Twist()

        v = msg.linear.x
        w = msg.angular.z

        if self.enable_gate:
            if abs(w) > self.w_gate:
                v = 0.0
            elif abs(v) > self.v_gate:
                w = 0.0
            else:
                v = 0.0
                w = 0.0

        v = v * self.lin_scale
        # w = w * self.ang_scale

        if w < 0:
            w -= 2.75
        elif w > 0:
            w += 2.75

        if v < -self.max_reverse:
            v = -self.max_reverse

        v = max(-self.max_v, min(self.max_v, v))
        w = max(-self.max_w, min(self.max_w, w))

        out.linear.x = v
        out.linear.y = 0.0
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = w

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPlatformTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()