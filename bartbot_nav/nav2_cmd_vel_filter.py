#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Nav2CmdVelFilter(Node):
    def __init__(self):
        super().__init__('nav2_cmd_vel_filter')

        self.declare_parameter('in_topic', '/cmd_vel_nav')
        self.declare_parameter('out_topic', '/cmd_vel_nav_smooth')

        self.declare_parameter('publish_rate_hz', 30.0)

        self.declare_parameter('linear_deadband', 0.005)
        self.declare_parameter('angular_deadband', 0.03)

        self.declare_parameter('max_linear_x', 0.08)
        self.declare_parameter('max_reverse_x', 0.05)
        self.declare_parameter('max_angular_z', 0.60)

        self.declare_parameter('min_forward_x', 0.04)
        self.declare_parameter('min_reverse_x', 0.03)

        self.declare_parameter('forward_w_limit', 0.18)
        self.declare_parameter('turn_in_place_w', 1.00)
        self.declare_parameter('turn_forward_scale', 0.45)

        self.declare_parameter('max_v_rate', 0.08)
        self.declare_parameter('max_w_rate', 1.00)

        self.declare_parameter('cmd_timeout', 0.50)

        self.in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.linear_deadband = float(self.get_parameter('linear_deadband').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)

        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_reverse_x = float(self.get_parameter('max_reverse_x').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)

        self.min_forward_x = float(self.get_parameter('min_forward_x').value)
        self.min_reverse_x = float(self.get_parameter('min_reverse_x').value)

        self.forward_w_limit = float(self.get_parameter('forward_w_limit').value)
        self.turn_in_place_w = float(self.get_parameter('turn_in_place_w').value)
        self.turn_forward_scale = float(self.get_parameter('turn_forward_scale').value)

        self.max_v_rate = float(self.get_parameter('max_v_rate').value)
        self.max_w_rate = float(self.get_parameter('max_w_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        if self.turn_in_place_w <= self.forward_w_limit:
            self.turn_in_place_w = self.forward_w_limit + 0.01

        self.turn_forward_scale = self.clamp(self.turn_forward_scale, 0.0, 1.0)

        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_timer_time = self.get_clock().now()

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.cmd_cb, 10)

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.get_logger().info(
            f'Nav2 cmd_vel filter started: {self.in_topic} -> {self.out_topic}'
        )

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def deadband(self, x, db):
        if abs(x) < db:
            return 0.0
        return x

    def apply_min_speed(self, x, min_value):
        if x > 0.0 and x < min_value:
            return min_value
        if x < 0.0 and abs(x) < min_value:
            return -min_value
        return x

    def smoothstep(self, x):
        x = self.clamp(x, 0.0, 1.0)
        return x * x * (3.0 - 2.0 * x)

    def rate_limit(self, target, current, max_rate, dt):
        max_step = max_rate * dt
        delta = target - current
        delta = self.clamp(delta, -max_step, max_step)
        return current + delta

    def cmd_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        v = self.deadband(v, self.linear_deadband)
        w = self.deadband(w, self.angular_deadband)

        abs_w = abs(w)

        if abs(v) > 0.0:
            if abs_w < self.forward_w_limit:
                w = 0.0

            elif abs_w < self.turn_in_place_w:
                ratio = (abs_w - self.forward_w_limit) / max(
                    self.turn_in_place_w - self.forward_w_limit,
                    1e-6
                )
                ratio = self.smoothstep(ratio)

                forward_factor = 1.0 - ratio * (1.0 - self.turn_forward_scale)
                v = v * forward_factor

            else:
                v = 0.0

        if v > 0.0:
            v = self.apply_min_speed(v, self.min_forward_x)
        elif v < 0.0:
            v = self.apply_min_speed(v, self.min_reverse_x)

        v = self.clamp(v, -self.max_reverse_x, self.max_linear_x)
        w = self.clamp(w, -self.max_angular_z, self.max_angular_z)

        self.target_v = v
        self.target_w = w

    def timer_cb(self):
        now = self.get_clock().now()

        dt = (now - self.last_timer_time).nanoseconds * 1e-9
        self.last_timer_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.publish_rate_hz

        time_since_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        if time_since_cmd > self.cmd_timeout:
            self.target_v = 0.0
            self.target_w = 0.0

        self.current_v = self.rate_limit(
            self.target_v,
            self.current_v,
            self.max_v_rate,
            dt
        )

        self.current_w = self.rate_limit(
            self.target_w,
            self.current_w,
            self.max_w_rate,
            dt
        )

        out = Twist()
        out.linear.x = self.current_v
        out.angular.z = self.current_w

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CmdVelFilter()

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