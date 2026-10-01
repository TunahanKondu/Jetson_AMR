"""ROS 2 line controller with wheel-tick based dropoff completion."""

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int64MultiArray, String

from pulsar_color_line_following.line_control import (
    PIDController, alignment_gate, calculate_linear_speed,
    calculate_wheel_distance, slew_rate_limit,
)


class ColorControllerNode(Node):
    """Follow the orange line and finish dropoff after a measured distance."""

    DROPOFF_LINE_FOLLOWING_STAGE = 5

    def __init__(self):
        super().__init__('color_line_controller')

        # Existing line controller parameters.
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('linear_speed', 0.02)
        self.declare_parameter('minimum_linear_speed', 0.01)
        self.declare_parameter('maximum_angular_speed', 0.20)
        self.declare_parameter('maximum_linear_acceleration', 0.05)
        self.declare_parameter('maximum_angular_acceleration', 0.8)
        self.declare_parameter('slowdown_gain', 0.8)
        self.declare_parameter('alignment_enter_error', 0.08)
        self.declare_parameter('alignment_exit_error', 0.03)
        self.declare_parameter('alignment_stable_time', 0.20)
        self.declare_parameter('steering_deadband_error', 0.03)
        self.declare_parameter('detection_timeout', 0.20)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.08)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('linear_direction', -1.0)
        self.declare_parameter('steering_sign', -1.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Dropoff completion parameters. Distance is calculated directly from
        # the two cumulative wheel encoder tick counters.
        self.declare_parameter('wheel_ticks_topic', '/wheel_ticks')
        self.declare_parameter(
            'mission_status_topic', '/amr/mission_status_json')
        self.declare_parameter('action_result_topic', '/amr/robot_action_result')
        self.declare_parameter('wheel_radius_m', 0.10)
        self.declare_parameter('ticks_per_revolution', 1100.0)
        self.declare_parameter('dropoff_target_distance_m', 1.5)
        self.declare_parameter('straight_fallback_start_distance_m', 1.0)
        self.declare_parameter('line_lost_straight_speed', 0.01)
        self.declare_parameter('wheel_ticks_timeout', 0.50)
        self.declare_parameter('maximum_tick_jump', 500)

        parameter = lambda name: self.get_parameter(name).value
        self.frequency = float(parameter('control_frequency'))
        self.max_linear = float(parameter('linear_speed'))
        self.min_linear = float(parameter('minimum_linear_speed'))
        self.max_angular = float(parameter('maximum_angular_speed'))
        self.max_linear_accel = float(parameter('maximum_linear_acceleration'))
        self.max_angular_accel = float(parameter('maximum_angular_acceleration'))
        self.slowdown = float(parameter('slowdown_gain'))
        self.alignment_enter_error = float(
            parameter('alignment_enter_error'))
        self.alignment_exit_error = float(
            parameter('alignment_exit_error'))
        self.alignment_stable_time = float(
            parameter('alignment_stable_time'))
        self.steering_deadband_error = float(
            parameter('steering_deadband_error'))
        self.timeout = float(parameter('detection_timeout'))
        self.linear_direction = float(parameter('linear_direction'))
        self.steering_sign = float(parameter('steering_sign'))
        self.wheel_radius = float(parameter('wheel_radius_m'))
        self.ticks_per_revolution = float(parameter('ticks_per_revolution'))
        self.dropoff_target_distance = float(
            parameter('dropoff_target_distance_m'))
        self.straight_fallback_start_distance = float(
            parameter('straight_fallback_start_distance_m'))
        self.line_lost_straight_speed = float(
            parameter('line_lost_straight_speed'))
        self.wheel_ticks_timeout = float(parameter('wheel_ticks_timeout'))
        self.maximum_tick_jump = int(parameter('maximum_tick_jump'))
        self._validate_parameters()

        self.pid = PIDController(
            parameter('kp'), parameter('ki'), parameter('kd'),
            self.max_angular, parameter('integral_limit'))

        # Normal line tracking state.
        self.active = False
        self.line_start_detector_ready = False
        self.detected = False
        self.error = 0.0
        self.error_time = None
        self.detection_time = None
        self.last_control_time = self.get_clock().now()
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.moving = False
        # Every run starts by centering the line. While this flag is true the
        # robot may rotate, but linear.x is forced to exactly zero.
        self.alignment_only = True
        self.alignment_stable_since = None

        # Mission and dropoff state.
        self.mission_stage = 0
        self.carrying_load = False
        self.current_ticks = None
        self.previous_ticks = None
        self.last_ticks_time = None
        self.total_travel_distance = 0.0
        self.has_seen_line = False
        self.result_sent = False
        self.last_tick_warning_time = 0.0
        self.last_distance_log_time = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, str(parameter('cmd_vel_topic')), 10)
        self.result_pub = self.create_publisher(
            String, str(parameter('action_result_topic')), 10)

        self.create_subscription(
            String, '/robot_action', self.action_callback, 10)
        self.create_subscription(
            Bool, '/color_line_detected', self.detected_callback, 10)
        self.create_subscription(
            Float32, '/color_line_error', self.error_callback, 10)
        self.create_subscription(
            Int64MultiArray, str(parameter('wheel_ticks_topic')),
            self.wheel_ticks_callback, 20)
        self.create_subscription(
            String, str(parameter('mission_status_topic')),
            self.mission_status_callback, 10)

        self.create_timer(1.0 / self.frequency, self.control_callback)

        self.get_logger().info(
            'Color line controller ready; waiting for LINE_START | '
            f'linear_direction={self.linear_direction}, '
            f'steering_sign={self.steering_sign}, '
            f'alignment_enter={self.alignment_enter_error:.3f}, '
            f'alignment_exit={self.alignment_exit_error:.3f}, '
            f'alignment_stable={self.alignment_stable_time:.2f} s, '
            f'steering_deadband={self.steering_deadband_error:.3f}, '
            f'target_distance={self.dropoff_target_distance:.2f} m, '
            f'straight_fallback_after='
            f'{self.straight_fallback_start_distance:.2f} m, '
            f'wheel_radius={self.wheel_radius:.3f} m, '
            f'ticks_per_revolution={self.ticks_per_revolution:.1f}')

    def _validate_parameters(self):
        if self.frequency <= 0 or self.timeout <= 0:
            raise ValueError('frequency and timeout must be positive')
        if not 0 <= self.min_linear <= self.max_linear:
            raise ValueError('linear speeds must satisfy 0 <= min <= max')
        if (self.max_angular <= 0 or self.max_linear_accel <= 0
                or self.max_angular_accel <= 0):
            raise ValueError('speed and acceleration limits must be positive')
        if not 0 <= self.alignment_exit_error < self.alignment_enter_error <= 1:
            raise ValueError(
                'alignment errors must satisfy 0 <= exit < enter <= 1')
        if self.alignment_stable_time < 0:
            raise ValueError('alignment_stable_time must not be negative')
        if not 0 <= self.steering_deadband_error <= self.alignment_exit_error:
            raise ValueError(
                'steering_deadband_error must satisfy '
                '0 <= deadband <= alignment_exit_error')
        if self.linear_direction not in (-1.0, 1.0):
            raise ValueError('linear_direction must be -1 or 1')
        if self.steering_sign not in (-1.0, 1.0):
            raise ValueError('steering_sign must be -1 or 1')
        if self.wheel_radius <= 0 or self.ticks_per_revolution <= 0:
            raise ValueError('wheel radius and ticks per revolution must be positive')
        if self.dropoff_target_distance <= 0:
            raise ValueError('dropoff_target_distance_m must be positive')
        if not 0 <= self.straight_fallback_start_distance < (
                self.dropoff_target_distance):
            raise ValueError(
                'straight_fallback_start_distance_m must satisfy '
                '0 <= fallback < dropoff target distance')
        if not 0 < self.line_lost_straight_speed <= self.max_linear:
            raise ValueError(
                'line_lost_straight_speed must be in (0, linear_speed]')
        if self.wheel_ticks_timeout <= 0 or self.maximum_tick_jump <= 0:
            raise ValueError('tick timeout and maximum jump must be positive')

    def action_callback(self, msg):
        action = msg.data.strip().upper()
        if action == 'LINE_START':
            # Mission Controller retries LINE_START until it receives an ACK.
            # Never reset PID/encoder distance for a duplicate command.
            if self.active:
                if self.line_start_detector_ready:
                    self.publish_line_start_accepted(repeated=True)
                return

            self.active = True
            self.line_start_detector_ready = False
            self.detected = False
            self.error_time = None
            self.detection_time = None
            self.pid.reset()
            self.last_control_time = self.get_clock().now()
            self.total_travel_distance = 0.0
            self.has_seen_line = False
            self.result_sent = False
            self.last_distance_log_time = 0.0
            self.alignment_only = True
            self.alignment_stable_since = None
            self.previous_ticks = self.current_ticks
            self.get_logger().info(
                'LINE_START received; distance reset and alignment-only mode enabled')
        elif action == 'LINE_STOP':
            self.active = False
            self.line_start_detector_ready = False
            self.detected = False
            self.pid.reset()
            self.alignment_only = True
            self.alignment_stable_since = None
            self.stop(force=True)
            result = String()
            result.data = 'LINE_CONTROLLER_STOPPED'
            self.result_pub.publish(result)
            self.get_logger().info(
                'LINE_STOP received; motion stopped and '
                'LINE_CONTROLLER_STOPPED published')

    def detected_callback(self, msg):
        self.detected = bool(msg.data)
        self.detection_time = self.get_clock().now()

        # Receiving even a False detection proves that the detector accepted
        # LINE_START, received a camera frame and completed one processing
        # cycle.  A single ACK therefore confirms both line-following nodes.
        if self.active and not self.line_start_detector_ready:
            self.line_start_detector_ready = True
            self.publish_line_start_accepted()

        if self.detected:
            self.has_seen_line = True
        elif not self.dropoff_mode():
            self.pid.reset()
            self.alignment_only = True
            self.alignment_stable_since = None
            self.stop()

    def publish_line_start_accepted(self, repeated=False):
        result = String()
        result.data = 'LINE_START_ACCEPTED'
        self.result_pub.publish(result)
        if repeated:
            self.get_logger().info(
                'Duplicate LINE_START received; state preserved and '
                'LINE_START_ACCEPTED re-published')
        else:
            self.get_logger().info(
                'Detector produced its first result; '
                'LINE_START_ACCEPTED published')

    def error_callback(self, msg):
        self.error = float(msg.data)
        self.error_time = self.get_clock().now()

    def mission_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
            self.mission_stage = int(status.get('stage', 0))
            self.carrying_load = bool(status.get('carryingLoad', False))
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'Mission status could not be read: {error}')

    def wheel_ticks_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning(
                '/wheel_ticks must contain [left_ticks, right_ticks]')
            return

        ticks = (int(msg.data[0]), int(msg.data[1]))
        self.current_ticks = ticks
        self.last_ticks_time = self.get_clock().now()

        if not self.active or self.previous_ticks is None:
            self.previous_ticks = ticks
            return

        left_delta = abs(ticks[0] - self.previous_ticks[0])
        right_delta = abs(ticks[1] - self.previous_ticks[1])
        self.previous_ticks = ticks

        if (left_delta > self.maximum_tick_jump
                or right_delta > self.maximum_tick_jump):
            now = time.monotonic()
            if now - self.last_tick_warning_time >= 2.0:
                self.get_logger().warning(
                    'Ignoring encoder jump: '
                    f'left={left_delta}, right={right_delta}')
                self.last_tick_warning_time = now
            return

        # In alignment-only mode wheel motion is rotation, not useful forward
        # travel. Counting it would make encoder-based dropoff finish early.
        if not self.alignment_only:
            self.total_travel_distance += calculate_wheel_distance(
                left_delta,
                right_delta,
                self.wheel_radius,
                self.ticks_per_revolution,
            )

        now = time.monotonic()
        if self.dropoff_mode() and now - self.last_distance_log_time >= 1.0:
            self.get_logger().info(
                'Dropoff encoder distance: '
                f'{self.total_travel_distance:.3f} / '
                f'{self.dropoff_target_distance:.3f} m')
            self.last_distance_log_time = now

    def dropoff_mode(self):
        return (
            self.mission_stage == self.DROPOFF_LINE_FOLLOWING_STAGE
            and self.carrying_load
        )

    def line_data_is_fresh(self, now):
        return (
            self.detected
            and self.error_time is not None
            and self.detection_time is not None
            and 0 <= (now - self.error_time).nanoseconds / 1e9 <= self.timeout
            and 0 <= (now - self.detection_time).nanoseconds / 1e9 <= self.timeout
        )

    def wheel_ticks_are_fresh(self, now):
        return (
            self.last_ticks_time is not None
            and 0 <= (now - self.last_ticks_time).nanoseconds / 1e9
            <= self.wheel_ticks_timeout
        )

    def control_callback(self):
        now = self.get_clock().now()
        dt = max((now - self.last_control_time).nanoseconds / 1e9, 1e-6)
        self.last_control_time = now

        if not self.active:
            self.stop()
            return

        if self.dropoff_mode():
            if not self.wheel_ticks_are_fresh(now):
                self.pid.reset()
                self.stop()
                return

            if self.total_travel_distance >= self.dropoff_target_distance:
                self.complete_dropoff()
                return

        if not self.line_data_is_fresh(now):
            self.pid.reset()
            self.alignment_stable_since = None
            # Blind motion is allowed only after the line has been seen at
            # least once in this dropoff run. This covers the final part where
            # the painted line ends before the configured encoder distance.
            if (self.dropoff_mode() and self.has_seen_line
                    and not self.alignment_only
                    and self.total_travel_distance
                    >= self.straight_fallback_start_distance):
                self.publish_motion(
                    self.linear_direction * self.line_lost_straight_speed,
                    0.0,
                    dt,
                )
            else:
                self.stop()
            return

        # Ignore tiny centre noise completely. Resetting the PID and the last
        # angular command prevents integral/derivative residue from making the
        # robot continuously rock left and right while already centred.
        if abs(self.error) <= self.steering_deadband_error:
            self.pid.reset()
            self.last_angular = 0.0
            target_angular = 0.0
        else:
            target_angular = (
                self.steering_sign * self.pid.update(self.error, dt))

        was_alignment_only = self.alignment_only
        gate_requests_alignment = alignment_gate(
            self.error,
            self.alignment_only,
            self.alignment_enter_error,
            self.alignment_exit_error,
        )

        if gate_requests_alignment:
            self.alignment_only = True
            self.alignment_stable_since = None
        elif self.alignment_only:
            # Require the tighter error band to remain valid briefly. One
            # noisy frame can therefore never release forward motion.
            if self.alignment_stable_since is None:
                self.alignment_stable_since = now
            stable_for = (
                now - self.alignment_stable_since).nanoseconds / 1e9
            if stable_for >= self.alignment_stable_time:
                self.alignment_only = False
                self.alignment_stable_since = None

        if self.alignment_only:
            if not was_alignment_only:
                self.get_logger().info(
                    f'Alignment-only mode: error={self.error:+.3f}; '
                    'linear motion blocked')
            self.publish_motion(
                0.0, target_angular, dt, hard_stop_linear=True)
            return

        if was_alignment_only:
            self.get_logger().info(
                f'Alignment complete: error={self.error:+.3f}; '
                'forward motion enabled')

        target_linear = self.linear_direction * calculate_linear_speed(
            self.error, self.max_linear, self.min_linear, self.slowdown)
        self.publish_motion(target_linear, target_angular, dt)

    def publish_motion(
            self, target_linear, target_angular, dt, hard_stop_linear=False):
        # Safety gate: when alignment is outside tolerance, do not let the
        # acceleration limiter carry a residual forward command into the turn.
        linear = (0.0 if hard_stop_linear else slew_rate_limit(
            target_linear, self.last_linear, self.max_linear_accel, dt))
        angular = slew_rate_limit(
            target_angular, self.last_angular, self.max_angular_accel, dt)
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)
        self.last_linear = linear
        self.last_angular = angular
        self.moving = True

    def complete_dropoff(self):
        if self.result_sent:
            return
        self.stop(force=True)
        self.active = False
        self.result_sent = True
        result = String()
        result.data = 'LINE_COMPLETE'
        self.result_pub.publish(result)
        self.get_logger().info(
            f'Dropoff distance {self.total_travel_distance:.3f} m reached; '
            'LINE_COMPLETE published')

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

