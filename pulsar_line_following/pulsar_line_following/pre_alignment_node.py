import math
from statistics import median

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool


class PreAlignmentNode(Node):
    """Align the AMR to the line before normal line following starts.

    Sequence:
      LINE_START
        -> camera angle control until angle ~= 0 deg
        -> stop and collect lateral-offset samples
        -> 90 deg in-place turn
        -> straight lateral correction distance
        -> opposite 90 deg in-place turn
        -> camera angle re-check
        -> offset verification
        -> enable the existing line-following PID controller

    During pre-alignment, linear.x and angular.z are NEVER commanded together.
    """

    IDLE = 'IDLE'
    ANGLE_ALIGN = 'ANGLE_ALIGN'
    OFFSET_MEASURE = 'OFFSET_MEASURE'
    SHIFT_TURN_1 = 'SHIFT_TURN_1'
    SHIFT_DRIVE = 'SHIFT_DRIVE'
    SHIFT_TURN_2 = 'SHIFT_TURN_2'
    FINAL_ANGLE_ALIGN = 'FINAL_ANGLE_ALIGN'
    FINAL_OFFSET_MEASURE = 'FINAL_OFFSET_MEASURE'
    MEASUREMENT_READY = 'MEASUREMENT_READY'
    LINE_FOLLOW = 'LINE_FOLLOW'

    def __init__(self):
        super().__init__('pre_alignment_node')

        # ---------------------------
        # Topics / control frequency
        # ---------------------------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('robot_action_topic', '/robot_action')
        self.declare_parameter('line_detected_topic', '/line_detected')
        self.declare_parameter('line_angle_topic', '/line_angle_error')
        self.declare_parameter('line_offset_px_topic', '/line_offset_px')
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('detection_timeout', 0.30)

        # ---------------------------
        # Phase 1: angle alignment
        # ---------------------------
        self.declare_parameter('angle_tolerance_deg', 1.0)
        self.declare_parameter('angle_stable_frames', 8)
        self.declare_parameter('angle_kp', 1.5)
        self.declare_parameter('angle_steering_sign', -1.0)
        # Camera is mounted at the rear. This multiplier lets us correct the
        # camera-angle -> robot-yaw sign without changing the vision math.
        # Keep +1.0 if the current angle-align test turns the correct way;
        # change to -1.0 if the physical robot rotates away from zero.
        self.declare_parameter('rear_camera_angle_multiplier', -1.0)
        self.declare_parameter('maximum_angle_speed', 0.20)
        self.declare_parameter('minimum_angle_speed', 0.04)

        # ---------------------------
        # Phase 2: offset measurement
        # ---------------------------
        self.declare_parameter('offset_sample_count', 15)

        # IMPORTANT:
        # This must be calibrated for the actual camera geometry.
        # Leave 0.0 to TEST angle alignment + pixel measurement only.
        self.declare_parameter('meters_per_pixel', 0.0)
        self.declare_parameter('lateral_measurement_sign', 1.0)
        self.declare_parameter('lateral_tolerance_m', 0.01)
        self.declare_parameter('max_lateral_corrections', 2)

        # ---------------------------
        # Lateral rectangular maneuver
        # ---------------------------
        self.declare_parameter('shift_turn_angle_deg', 90.0)
        self.declare_parameter('turn_speed', 0.20)
        self.declare_parameter('minimum_turn_speed', 0.05)
        self.declare_parameter('turn_kp', 1.5)
        self.declare_parameter('turn_tolerance_deg', 1.0)
        self.declare_parameter('shift_linear_speed', 0.08)
        self.declare_parameter('minimum_shift_linear_speed', 0.025)
        self.declare_parameter('shift_linear_kp', 0.8)
        self.declare_parameter('distance_tolerance_m', 0.005)

        # Read parameters.
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.robot_action_topic = self.get_parameter('robot_action_topic').value
        self.line_detected_topic = self.get_parameter('line_detected_topic').value
        self.line_angle_topic = self.get_parameter('line_angle_topic').value
        self.line_offset_px_topic = self.get_parameter('line_offset_px_topic').value
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)

        self.angle_tolerance_deg = float(self.get_parameter('angle_tolerance_deg').value)
        self.angle_stable_frames = int(self.get_parameter('angle_stable_frames').value)
        self.angle_kp = float(self.get_parameter('angle_kp').value)
        self.angle_steering_sign = float(self.get_parameter('angle_steering_sign').value)
        self.rear_camera_angle_multiplier = float(
            self.get_parameter('rear_camera_angle_multiplier').value
        )
        self.maximum_angle_speed = abs(float(self.get_parameter('maximum_angle_speed').value))
        self.minimum_angle_speed = abs(float(self.get_parameter('minimum_angle_speed').value))

        self.offset_sample_count = int(self.get_parameter('offset_sample_count').value)
        self.meters_per_pixel = float(self.get_parameter('meters_per_pixel').value)
        self.lateral_measurement_sign = float(self.get_parameter('lateral_measurement_sign').value)
        self.lateral_tolerance_m = abs(float(self.get_parameter('lateral_tolerance_m').value))
        self.max_lateral_corrections = int(self.get_parameter('max_lateral_corrections').value)

        self.shift_turn_angle_rad = math.radians(
            float(self.get_parameter('shift_turn_angle_deg').value)
        )
        self.turn_speed = abs(float(self.get_parameter('turn_speed').value))
        self.minimum_turn_speed = abs(float(self.get_parameter('minimum_turn_speed').value))
        self.turn_kp = float(self.get_parameter('turn_kp').value)
        self.turn_tolerance_rad = math.radians(
            abs(float(self.get_parameter('turn_tolerance_deg').value))
        )
        self.shift_linear_speed = abs(float(self.get_parameter('shift_linear_speed').value))
        self.minimum_shift_linear_speed = abs(
            float(self.get_parameter('minimum_shift_linear_speed').value)
        )
        self.shift_linear_kp = float(self.get_parameter('shift_linear_kp').value)
        self.distance_tolerance_m = abs(
            float(self.get_parameter('distance_tolerance_m').value)
        )

        if self.control_frequency <= 0.0:
            raise ValueError('control_frequency must be > 0')
        if self.angle_stable_frames < 1:
            raise ValueError('angle_stable_frames must be >= 1')
        if self.offset_sample_count < 1:
            raise ValueError('offset_sample_count must be >= 1')
        if self.max_lateral_corrections < 0:
            raise ValueError('max_lateral_corrections must be >= 0')

        # ---------------------------
        # ROS interfaces
        # ---------------------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, '/line_alignment/state', 10)
        self.measured_offset_px_pub = self.create_publisher(
            Float32, '/line_alignment/measured_offset_px', 10
        )
        self.measured_offset_m_pub = self.create_publisher(
            Float32, '/line_alignment/measured_offset_m', 10
        )

        self.create_subscription(
            String,
            self.robot_action_topic,
            self.robot_action_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.line_detected_topic,
            self.line_detected_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self.line_angle_topic,
            self.line_angle_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self.line_offset_px_topic,
            self.line_offset_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20,
        )

        self.line_follow_enable_client = self.create_client(
            SetBool,
            '/line_following/enable',
        )

        # ---------------------------
        # State
        # ---------------------------
        self.state = self.IDLE
        self.line_detected = False
        self.latest_angle_deg = None
        self.latest_offset_px = None
        self.last_vision_time = None

        self.angle_stable_count = 0
        self.offset_samples = []

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.turn_start_yaw = None
        self.drive_start_x = None
        self.drive_start_y = None

        self.shift_turn_direction = 0.0
        self.shift_distance_m = 0.0
        self.lateral_correction_count = 0

        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_callback,
        )

        self.stop_robot()
        self.publish_state()

        self.get_logger().info('Pre-alignment node started.')
        self.get_logger().info(
            'Flow: ANGLE -> OFFSET -> 90deg -> STRAIGHT -> -90deg -> VERIFY -> REVERSE LINE FOLLOW'
        )
        self.get_logger().info(
            f'rear_camera_angle_multiplier: {self.rear_camera_angle_multiplier}'
        )
        if self.meters_per_pixel <= 0.0:
            self.get_logger().warning(
                'meters_per_pixel is 0.0. The node will align the angle and measure '
                'the lateral offset in pixels, but it will NOT execute the lateral '
                'shift until camera distance calibration is configured.'
            )

    # =====================================================
    # ROS callbacks
    # =====================================================

    def robot_action_callback(self, msg):
        action = msg.data.strip().lower()

        if action == 'line_start':
            self.get_logger().info('LINE_START -> starting pre-alignment.')
            self.call_line_follow_enable(False)
            self.lateral_correction_count = 0
            self.start_angle_alignment(final=False)

        elif action == 'line_stop':
            self.get_logger().info('LINE_STOP -> stopping pre-alignment and line follow.')
            self.call_line_follow_enable(False)
            self.stop_robot()
            self.set_state(self.IDLE)

    def line_detected_callback(self, msg):
        self.line_detected = bool(msg.data)
        if not self.line_detected:
            self.angle_stable_count = 0

    def line_angle_callback(self, msg):
        self.latest_angle_deg = float(msg.data)
        self.last_vision_time = self.get_clock().now()

        if self.state in (self.ANGLE_ALIGN, self.FINAL_ANGLE_ALIGN):
            if abs(self.latest_angle_deg) <= self.angle_tolerance_deg:
                self.angle_stable_count += 1
            else:
                self.angle_stable_count = 0

    def line_offset_callback(self, msg):
        self.latest_offset_px = float(msg.data)
        self.last_vision_time = self.get_clock().now()

        if self.state in (self.OFFSET_MEASURE, self.FINAL_OFFSET_MEASURE):
            # Only accept offset frames while the line is still angularly aligned.
            if (
                self.latest_angle_deg is not None
                and abs(self.latest_angle_deg) <= self.angle_tolerance_deg
            ):
                self.offset_samples.append(self.latest_offset_px)

    def odom_callback(self, msg):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    # =====================================================
    # State machine
    # =====================================================

    def control_callback(self):
        if self.state == self.IDLE or self.state == self.LINE_FOLLOW:
            return

        if self.state in (
            self.ANGLE_ALIGN,
            self.OFFSET_MEASURE,
            self.FINAL_ANGLE_ALIGN,
            self.FINAL_OFFSET_MEASURE,
        ):
            if not self.vision_is_fresh():
                self.stop_robot()
                return

        if self.state == self.ANGLE_ALIGN:
            self.control_angle_alignment(final=False)

        elif self.state == self.OFFSET_MEASURE:
            self.control_offset_measurement(final=False)

        elif self.state == self.SHIFT_TURN_1:
            self.control_turn(first_turn=True)

        elif self.state == self.SHIFT_DRIVE:
            self.control_shift_drive()

        elif self.state == self.SHIFT_TURN_2:
            self.control_turn(first_turn=False)

        elif self.state == self.FINAL_ANGLE_ALIGN:
            self.control_angle_alignment(final=True)

        elif self.state == self.FINAL_OFFSET_MEASURE:
            self.control_offset_measurement(final=True)

        elif self.state == self.MEASUREMENT_READY:
            self.stop_robot()

    # =====================================================
    # Phase 1 / final phase: camera angle -> zero
    # =====================================================

    def start_angle_alignment(self, final=False):
        self.stop_robot()
        self.angle_stable_count = 0
        self.offset_samples = []
        self.set_state(self.FINAL_ANGLE_ALIGN if final else self.ANGLE_ALIGN)

    def control_angle_alignment(self, final=False):
        if not self.line_detected or self.latest_angle_deg is None:
            self.stop_robot()
            return

        if self.angle_stable_count >= self.angle_stable_frames:
            self.stop_robot()
            self.get_logger().info(
                f'Angle aligned: {self.latest_angle_deg:+.2f} deg '
                f'for {self.angle_stable_count} frames.'
            )
            self.offset_samples = []
            self.set_state(
                self.FINAL_OFFSET_MEASURE if final else self.OFFSET_MEASURE
            )
            return

        error_rad = math.radians(self.latest_angle_deg)
        angular = (
            self.angle_steering_sign
            * self.rear_camera_angle_multiplier
            * self.angle_kp
            * error_rad
        )
        angular = self.clamp(
            angular,
            -self.maximum_angle_speed,
            self.maximum_angle_speed,
        )

        if abs(angular) < self.minimum_angle_speed and abs(self.latest_angle_deg) > self.angle_tolerance_deg:
            angular = math.copysign(self.minimum_angle_speed, angular)

        command = Twist()
        command.linear.x = 0.0
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)

    # =====================================================
    # Phase 2: stationary offset sampling
    # =====================================================

    def control_offset_measurement(self, final=False):
        self.stop_robot()

        if len(self.offset_samples) < self.offset_sample_count:
            return

        samples = self.offset_samples[:self.offset_sample_count]
        self.offset_samples = []
        offset_px = float(median(samples))

        px_msg = Float32()
        px_msg.data = offset_px
        self.measured_offset_px_pub.publish(px_msg)

        side = 'RIGHT' if offset_px > 0.0 else 'LEFT'
        if abs(offset_px) < 0.5:
            side = 'CENTER'

        self.get_logger().info(
            f'Lateral measurement: {offset_px:+.2f} px -> line is {side}.'
        )

        if self.meters_per_pixel <= 0.0:
            self.get_logger().warning(
                'Measurement finished, but meters_per_pixel is not calibrated. '
                'Robot remains stopped. Configure meters_per_pixel to execute shift.'
            )
            self.set_state(self.MEASUREMENT_READY)
            return

        offset_m = (
            offset_px
            * self.meters_per_pixel
            * self.lateral_measurement_sign
        )

        m_msg = Float32()
        m_msg.data = float(offset_m)
        self.measured_offset_m_pub.publish(m_msg)

        self.get_logger().info(
            f'Calibrated lateral offset: {offset_m:+.4f} m.'
        )

        if abs(offset_m) <= self.lateral_tolerance_m:
            self.get_logger().info('Lateral position is inside tolerance.')
            self.finish_alignment()
            return

        if final and self.lateral_correction_count >= self.max_lateral_corrections:
            self.get_logger().warning(
                'Maximum lateral correction count reached. Robot stays stopped.'
            )
            self.set_state(self.MEASUREMENT_READY)
            return

        self.prepare_lateral_shift(offset_m)

    # =====================================================
    # Rectangular lateral shift
    # =====================================================

    def prepare_lateral_shift(self, offset_m):
        if self.odom_yaw is None:
            self.get_logger().warning('No /odom yet. Cannot execute lateral shift.')
            self.set_state(self.MEASUREMENT_READY)
            return

        # offset_m > 0: line is to robot/camera RIGHT -> first turn RIGHT (-Z)
        # offset_m < 0: line is to LEFT -> first turn LEFT (+Z)
        self.shift_turn_direction = -1.0 if offset_m > 0.0 else 1.0
        self.shift_distance_m = abs(float(offset_m))
        self.lateral_correction_count += 1

        turn_text = 'RIGHT' if self.shift_turn_direction < 0.0 else 'LEFT'
        self.get_logger().info(
            'Calculated pre-alignment plan: '
            f'TURN {turn_text} {math.degrees(self.shift_turn_angle_rad):.1f} deg -> '
            f'STRAIGHT {self.shift_distance_m:.3f} m -> '
            f'TURN back {math.degrees(self.shift_turn_angle_rad):.1f} deg.'
        )

        self.turn_start_yaw = self.odom_yaw
        self.set_state(self.SHIFT_TURN_1)

    def control_turn(self, first_turn):
        if self.odom_yaw is None or self.turn_start_yaw is None:
            self.stop_robot()
            return

        direction = self.shift_turn_direction if first_turn else -self.shift_turn_direction
        target_delta = direction * self.shift_turn_angle_rad
        travelled = self.normalize_angle(self.odom_yaw - self.turn_start_yaw)
        error = self.normalize_angle(target_delta - travelled)

        if abs(error) <= self.turn_tolerance_rad:
            self.stop_robot()

            if first_turn:
                self.drive_start_x = self.odom_x
                self.drive_start_y = self.odom_y
                self.set_state(self.SHIFT_DRIVE)
            else:
                self.start_angle_alignment(final=True)
            return

        angular = self.turn_kp * error
        angular = self.clamp(angular, -self.turn_speed, self.turn_speed)
        if abs(angular) < self.minimum_turn_speed:
            angular = math.copysign(self.minimum_turn_speed, angular)

        command = Twist()
        command.linear.x = 0.0
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)

    def control_shift_drive(self):
        if (
            self.odom_x is None
            or self.odom_y is None
            or self.drive_start_x is None
            or self.drive_start_y is None
        ):
            self.stop_robot()
            return

        travelled = math.hypot(
            self.odom_x - self.drive_start_x,
            self.odom_y - self.drive_start_y,
        )
        remaining = self.shift_distance_m - travelled

        if remaining <= self.distance_tolerance_m:
            self.stop_robot()
            self.turn_start_yaw = self.odom_yaw
            self.set_state(self.SHIFT_TURN_2)
            return

        linear = self.shift_linear_kp * remaining
        linear = self.clamp(
            linear,
            self.minimum_shift_linear_speed,
            self.shift_linear_speed,
        )

        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = 0.0
        self.cmd_pub.publish(command)

    # =====================================================
    # Handoff to normal line following
    # =====================================================

    def finish_alignment(self):
        self.stop_robot()
        self.get_logger().info(
            'PRE-ALIGNMENT COMPLETE -> enabling reverse line-follow controller.'
        )
        self.call_line_follow_enable(True)
        self.set_state(self.LINE_FOLLOW)

    def call_line_follow_enable(self, enabled):
        if not self.line_follow_enable_client.service_is_ready():
            self.get_logger().warning(
                '/line_following/enable service is not ready yet.'
            )
            return

        request = SetBool.Request()
        request.data = bool(enabled)
        self.line_follow_enable_client.call_async(request)

    # =====================================================
    # Helpers
    # =====================================================

    def vision_is_fresh(self):
        if not self.line_detected or self.last_vision_time is None:
            return False
        age = (
            self.get_clock().now() - self.last_vision_time
        ).nanoseconds / 1e9
        return age <= self.detection_timeout

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.publish_state()
            self.get_logger().info(f'Alignment state -> {state}')

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(float(value), maximum))

    def destroy_node(self):
        self.stop_robot()
        self.call_line_follow_enable(False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PreAlignmentNode()

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
