#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToPlatformTwist(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_platform_twist')

        self.declare_parameter('in_topic', '/cmd_vel_muxed')
        self.declare_parameter('out_topic', '/platform/twist')

        self.declare_parameter('linear_scale', 8.0)
        self.declare_parameter('angular_scale', 4.2)

        # Keep 1.0 so:
        #   angular.z < 0 = right turn
        #   angular.z > 0 = left turn
        self.declare_parameter('angular_sign', 1.0)

        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 3.6)
        self.declare_parameter('max_reverse', 0.4)

        self.declare_parameter('linear_deadband', 0.01)
        self.declare_parameter('angular_deadband', 0.03)

        self.declare_parameter('min_forward_cmd', 0.50)
        self.declare_parameter('min_reverse_cmd', 0.30)
        self.declare_parameter('min_turn_cmd', 3.00)

        self.declare_parameter('cmd_timeout', 0.50)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value

        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.angular_sign = float(self.get_parameter('angular_sign').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.max_reverse = float(self.get_parameter('max_reverse').value)

        self.linear_deadband = float(self.get_parameter('linear_deadband').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)

        self.min_forward_cmd = float(self.get_parameter('min_forward_cmd').value)
        self.min_reverse_cmd = float(self.get_parameter('min_reverse_cmd').value)
        self.min_turn_cmd = float(self.get_parameter('min_turn_cmd').value)

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.target_v = 0.0
        self.target_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.cmd_cb, 10)

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.get_logger().info(
            f'Simple platform bridge started: {self.in_topic} -> {self.out_topic}'
        )
        self.get_logger().info(
            f'linear_scale={self.linear_scale:.2f}, '
            f'angular_scale={self.angular_scale:.2f}, '
            f'min_turn_cmd={self.min_turn_cmd:.2f}'
        )
        self.get_logger().info(
            f'Convention: -angular.z = RIGHT, +angular.z = LEFT, '
            f'angular_sign={self.angular_sign:.1f}'
        )

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def deadband(self, x, db):
        if abs(x) < db:
            return 0.0
        return x

    def apply_min_command(self, x, min_cmd):
        if x > 0.0 and abs(x) < min_cmd:
            return min_cmd
        if x < 0.0 and abs(x) < min_cmd:
            return -min_cmd
        return x

    def cmd_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        v = self.deadband(v, self.linear_deadband)
        w = self.deadband(w, self.angular_deadband)

        v = v * self.linear_scale
        w = w * self.angular_scale * self.angular_sign

        if v > 0.0:
            v = self.apply_min_command(v, self.min_forward_cmd)
        elif v < 0.0:
            v = self.apply_min_command(v, self.min_reverse_cmd)

        if w != 0.0:
            w = self.apply_min_command(w, self.min_turn_cmd)

        if v < -self.max_reverse:
            v = -self.max_reverse

        v = self.clamp(v, -self.max_linear, self.max_linear)
        w = self.clamp(w, -self.max_angular, self.max_angular)

        self.target_v = v
        self.target_w = w

    def timer_cb(self):
        now = self.get_clock().now()
        time_since_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9

        if time_since_cmd > self.cmd_timeout:
            self.target_v = 0.0
            self.target_w = 0.0

        out = Twist()
        out.linear.x = self.target_v
        out.linear.y = 0.0
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = self.target_w

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPlatformTwist()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop_msg = Twist()
    node.pub.publish(stop_msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()