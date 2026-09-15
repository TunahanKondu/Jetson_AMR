"""ROS 2 motion controller for the detected color stripe."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from pulsar_color_line_following.line_control import (
    PIDController, calculate_linear_speed, slew_rate_limit,
)


class ColorControllerNode(Node):
    def __init__(self):
        super().__init__('color_line_controller')
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('linear_speed', 0.02)
        self.declare_parameter('minimum_linear_speed', 0.01)
        self.declare_parameter('maximum_angular_speed', 0.20)
        self.declare_parameter('maximum_linear_acceleration', 0.05)
        self.declare_parameter('maximum_angular_acceleration', 0.8)
        self.declare_parameter('slowdown_gain', 0.8)
        self.declare_parameter('detection_timeout', 0.20)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.08)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('linear_direction', -1.0)
        self.declare_parameter('steering_sign', -1.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        parameter = lambda name: self.get_parameter(name).value
        self.frequency = parameter('control_frequency')
        self.max_linear = parameter('linear_speed')
        self.min_linear = parameter('minimum_linear_speed')
        self.max_angular = parameter('maximum_angular_speed')
        self.max_linear_accel = parameter('maximum_linear_acceleration')
        self.max_angular_accel = parameter('maximum_angular_acceleration')
        self.slowdown = parameter('slowdown_gain')
        self.timeout = parameter('detection_timeout')
        self.linear_direction = parameter('linear_direction')
        self.steering_sign = parameter('steering_sign')
        if self.frequency <= 0 or self.timeout <= 0:
            raise ValueError('frequency and timeout must be positive')
        if not 0 <= self.min_linear <= self.max_linear:
            raise ValueError('linear speeds must satisfy 0 <= min <= max')
        if self.max_angular <= 0 or self.max_linear_accel <= 0 or self.max_angular_accel <= 0:
            raise ValueError('speed and acceleration limits must be positive')
        if self.linear_direction not in (-1.0, 1.0):
            raise ValueError('linear_direction must be -1 or 1')
        if self.steering_sign not in (-1.0, 1.0):
            raise ValueError('steering_sign must be -1 or 1')
        self.pid = PIDController(
            parameter('kp'), parameter('ki'), parameter('kd'),
            self.max_angular, parameter('integral_limit'))
        self.active = False
        self.detected = False
        self.error = 0.0
        self.error_time = None
        self.detection_time = None
        self.last_control_time = self.get_clock().now()
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.moving = False
        self.cmd_pub = self.create_publisher(Twist, parameter('cmd_vel_topic'), 10)
        self.create_subscription(String, '/robot_action', self.action_callback, 10)
        self.create_subscription(Bool, '/color_line_detected', self.detected_callback, 10)
        self.create_subscription(Float32, '/color_line_error', self.error_callback, 10)
        self.create_timer(1.0 / self.frequency, self.control_callback)
        self.get_logger().info(
            f'Color line controller ready; waiting for LINE_START | '
            f'linear_direction={self.linear_direction}, '
            f'steering_sign={self.steering_sign}')

    def action_callback(self, msg):
        action = msg.data.strip().upper()
        if action == 'LINE_START':
            self.active = True
            self.detected = False
            self.error_time = None
            self.detection_time = None
            self.pid.reset()
            self.last_control_time = self.get_clock().now()
        elif action == 'LINE_STOP':
            self.active = False
            self.detected = False
            self.pid.reset()
            self.stop(force=True)

    def detected_callback(self, msg):
        self.detected = bool(msg.data)
        self.detection_time = self.get_clock().now()
        if not self.detected:
            self.pid.reset()
            self.stop()

    def error_callback(self, msg):
        self.error = float(msg.data)
        self.error_time = self.get_clock().now()

    def control_callback(self):
        now = self.get_clock().now()
        dt = max((now - self.last_control_time).nanoseconds / 1e9, 1e-6)
        self.last_control_time = now
        fresh = (self.detected and self.error_time is not None
                 and self.detection_time is not None
                 and 0 <= (now - self.error_time).nanoseconds / 1e9 <= self.timeout
                 and 0 <= (now - self.detection_time).nanoseconds / 1e9 <= self.timeout)
        if not self.active or not fresh:
            self.pid.reset()
            self.stop()
            return
        target_angular = self.steering_sign * self.pid.update(self.error, dt)
        target_linear = self.linear_direction * calculate_linear_speed(
            self.error, self.max_linear, self.min_linear, self.slowdown)
        linear = slew_rate_limit(
            target_linear, self.last_linear, self.max_linear_accel, dt)
        angular = slew_rate_limit(
            target_angular, self.last_angular, self.max_angular_accel, dt)
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)
        self.last_linear = linear
        self.last_angular = angular
        self.moving = True

    def stop(self, force=False):
        if not self.moving and not force:
            return
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.moving = False
        if self.context.ok():
            self.cmd_pub.publish(Twist())

    def destroy_node(self):
        self.stop(force=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
