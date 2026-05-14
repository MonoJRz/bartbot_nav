#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelMuxSimple(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_simple')

        self.declare_parameter('joy_topic', '/cmd_vel')
        self.declare_parameter('nav_topic', '/cmd_vel_nav_smooth')
        self.declare_parameter('out_topic', '/cmd_vel_muxed')

        self.declare_parameter('publish_rate_hz', 30.0)

        self.declare_parameter('joy_timeout', 0.50)
        self.declare_parameter('nav_timeout', 0.50)

        self.declare_parameter('joy_linear_deadband', 0.005)
        self.declare_parameter('joy_angular_deadband', 0.005)

        self.joy_topic = self.get_parameter('joy_topic').value
        self.nav_topic = self.get_parameter('nav_topic').value
        self.out_topic = self.get_parameter('out_topic').value

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)

        self.joy_linear_deadband = float(self.get_parameter('joy_linear_deadband').value)
        self.joy_angular_deadband = float(self.get_parameter('joy_angular_deadband').value)

        self.last_joy_time = self.get_clock().now()
        self.last_nav_time = self.get_clock().now()

        self.joy_cmd = Twist()
        self.nav_cmd = Twist()

        self.pub = self.create_publisher(Twist, self.out_topic, 10)

        self.joy_sub = self.create_subscription(
            Twist,
            self.joy_topic,
            self.joy_cb,
            10
        )

        self.nav_sub = self.create_subscription(
            Twist,
            self.nav_topic,
            self.nav_cb,
            10
        )

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.get_logger().info(
            f'Simple Twist mux started: joy={self.joy_topic}, '
            f'nav={self.nav_topic}, out={self.out_topic}'
        )
        self.get_logger().info(
            'Priority: joystick > navigation'
        )

    def is_nonzero(self, msg: Twist):
        return (
            abs(msg.linear.x) > self.joy_linear_deadband or
            abs(msg.angular.z) > self.joy_angular_deadband
        )

    def zero_twist(self):
        return Twist()

    def joy_cb(self, msg: Twist):
        self.joy_cmd = msg

        if self.is_nonzero(msg):
            self.last_joy_time = self.get_clock().now()

    def nav_cb(self, msg: Twist):
        self.nav_cmd = msg
        self.last_nav_time = self.get_clock().now()

    def timer_cb(self):
        now = self.get_clock().now()

        joy_age = (now - self.last_joy_time).nanoseconds * 1e-9
        nav_age = (now - self.last_nav_time).nanoseconds * 1e-9

        joy_active = joy_age < self.joy_timeout
        nav_active = nav_age < self.nav_timeout

        if joy_active:
            out = self.joy_cmd
            source = 'joy'
        elif nav_active:
            out = self.nav_cmd
            source = 'nav'
        else:
            out = self.zero_twist()
            source = 'zero'

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxSimple()

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