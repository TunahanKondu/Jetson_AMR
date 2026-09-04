from pulsar_line_following.line_control import (
    calculate_linear_speed,
    PIDController,
    slew_rate_limit,
)
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool


class LineControllerNode(Node):

    def __init__(self):
        super().__init__('line_controller_node')

        # Parameters
        self.declare_parameter(
            'control_frequency',
            30.0
        )

        self.declare_parameter(
            'linear_speed',
            0.02
        )

        self.declare_parameter(
            'maximum_angular_speed',
            0.20
        )

        self.declare_parameter(
            'detection_timeout',
            0.20
        )
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('line_error_topic', '/line_error')
        self.declare_parameter('line_detected_topic', '/line_detected')
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.08)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('steering_sign', -1.0)
        self.declare_parameter('enabled_on_start', True)
        self.declare_parameter('minimum_linear_speed', 0.01)
        self.declare_parameter('slowdown_gain', 0.8)
        self.declare_parameter('maximum_linear_acceleration', 0.05)
        self.declare_parameter('maximum_angular_acceleration', 0.8)

        # Read parameters
        self.control_frequency = (
            self.get_parameter('control_frequency')
            .get_parameter_value()
            .double_value
        )

        self.linear_speed = (
            self.get_parameter('linear_speed')
            .get_parameter_value()
            .double_value
        )

        self.maximum_angular_speed = (
            self.get_parameter('maximum_angular_speed')
            .get_parameter_value()
            .double_value
        )

        self.detection_timeout = (
            self.get_parameter('detection_timeout')
            .get_parameter_value()
            .double_value
        )
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.line_error_topic = self.get_parameter('line_error_topic').value
        self.line_detected_topic = self.get_parameter(
            'line_detected_topic'
        ).value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.steering_sign = self.get_parameter('steering_sign').value
        self.enabled = self.get_parameter('enabled_on_start').value
        self.minimum_linear_speed = self.get_parameter(
            'minimum_linear_speed'
        ).value
        self.slowdown_gain = self.get_parameter('slowdown_gain').value
        self.maximum_linear_acceleration = self.get_parameter(
            'maximum_linear_acceleration'
        ).value
        self.maximum_angular_acceleration = self.get_parameter(
            'maximum_angular_acceleration'
        ).value

        if self.control_frequency <= 0.0:
            raise ValueError('control_frequency must be greater than zero')
        if self.detection_timeout <= 0.0:
            raise ValueError('detection_timeout must be greater than zero')
        if not 0.0 <= self.minimum_linear_speed <= self.linear_speed:
            raise ValueError(
                'linear speeds must satisfy '
                '0 <= minimum_linear_speed <= linear_speed'
            )
        if not 0.0 <= self.slowdown_gain <= 1.0:
            raise ValueError('slowdown_gain must be between zero and one')
        if self.maximum_linear_acceleration <= 0.0:
            raise ValueError(
                'maximum_linear_acceleration must be greater than zero'
            )
        if self.maximum_angular_acceleration <= 0.0:
            raise ValueError(
                'maximum_angular_acceleration must be greater than zero'
            )

        self.pid = PIDController(
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            output_limit=self.maximum_angular_speed,
            integral_limit=self.integral_limit,
        )

        self.latest_error = 0.0
        self.line_detected = False
        self.last_detection_time = None
        self.last_control_time = self.get_clock().now()
        self.was_commanding_motion = False
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.error_subscription = self.create_subscription(
            Float32,
            self.line_error_topic,
            self.error_callback,
            10,
        )
        self.detection_subscription = self.create_subscription(
            Bool,
            self.line_detected_topic,
            self.detection_callback,
            10,
        )
        self.enable_service = self.create_service(
            SetBool,
            '/line_following/enable',
            self.enable_callback,
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_callback,
        )
        self.stop_robot(force=True)

        self.get_logger().info('Line controller node started.')

        self.get_logger().info(
            f'control_frequency: {self.control_frequency}'
        )

        self.get_logger().info(
            f'linear_speed: {self.linear_speed}'
        )

        self.get_logger().info(
            f'maximum_angular_speed: {self.maximum_angular_speed}'
        )
        self.get_logger().info(
            'Dynamic speed: '
            f'min={self.minimum_linear_speed}, '
            f'max={self.linear_speed}, gain={self.slowdown_gain}'
        )
        self.get_logger().info(
            'Acceleration limits: '
            f'linear={self.maximum_linear_acceleration}, '
            f'angular={self.maximum_angular_acceleration}'
        )

        self.get_logger().info(
            f'detection_timeout: {self.detection_timeout}'
        )

        self.get_logger().info(
            f'PID: kp={self.kp}, ki={self.ki}, kd={self.kd}'
        )
        self.get_logger().info(
            f'cmd_vel topic: {self.cmd_vel_topic}'
        )
        self.get_logger().info(
            f'Controller enabled: {self.enabled}'
        )

    def error_callback(self, msg):
        self.latest_error = float(msg.data)
        self.last_detection_time = self.get_clock().now()

    def detection_callback(self, msg):
        self.line_detected = bool(msg.data)
        if self.line_detected:
            self.last_detection_time = self.get_clock().now()
        else:
            self.pid.reset()
            self.stop_robot()

    def enable_callback(self, request, response):
        self.enabled = bool(request.data)
        self.pid.reset()
        self.last_control_time = self.get_clock().now()
        if not self.enabled:
            self.stop_robot(force=True)

        state = 'enabled' if self.enabled else 'disabled'
        response.success = True
        response.message = f'Line following controller {state}.'
        self.get_logger().info(response.message)
        return response

    def control_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now

        if not self.enabled or not self.is_detection_fresh(now):
            self.pid.reset()
            self.stop_robot()
            return

        dt = max(dt, 1e-6)
        target_angular_velocity = self.steering_sign * self.pid.update(
            self.latest_error,
            dt,
        )
        target_linear_velocity = calculate_linear_speed(
            error=self.latest_error,
            maximum_speed=self.linear_speed,
            minimum_speed=self.minimum_linear_speed,
            slowdown_gain=self.slowdown_gain,
        )
        linear_velocity = slew_rate_limit(
            target=target_linear_velocity,
            current=self.last_linear_velocity,
            maximum_rate=self.maximum_linear_acceleration,
            dt=dt,
        )
        angular_velocity = slew_rate_limit(
            target=target_angular_velocity,
            current=self.last_angular_velocity,
            maximum_rate=self.maximum_angular_acceleration,
            dt=dt,
        )

        command = Twist()
        command.linear.x = float(linear_velocity)
        command.angular.z = float(angular_velocity)
        self.cmd_vel_publisher.publish(command)
        self.last_linear_velocity = linear_velocity
        self.last_angular_velocity = angular_velocity
        self.was_commanding_motion = True

    def is_detection_fresh(self, now):
        if not self.line_detected or self.last_detection_time is None:
            return False
        age = (now - self.last_detection_time).nanoseconds / 1e9
        return age <= self.detection_timeout

    def stop_robot(self, force=False):
        if not self.was_commanding_motion and not force:
            return
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
        if not self.context.ok():
            self.was_commanding_motion = False
            return
        self.cmd_vel_publisher.publish(Twist())
        self.was_commanding_motion = False

    def destroy_node(self):
        self.stop_robot(force=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LineControllerNode()

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
