"""ROS 2 camera node for the orange center stripe."""

import os
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, String

from pulsar_color_line_following.color_vision import detect_orange_line, draw_detection


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_line_detector')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('roi_start_ratio', 0.45)
        self.declare_parameter('roi_end_ratio', 0.88)
        self.declare_parameter('hsv_lower', [5, 80, 70])
        self.declare_parameter('hsv_upper', [25, 255, 255])
        self.declare_parameter('min_area_ratio', 0.0005)
        self.declare_parameter('minimum_height_ratio', 0.18)
        self.declare_parameter('maximum_center_jump_ratio', 0.25)
        self.declare_parameter('tracking_reset_frames', 5)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_image', False)
        self.declare_parameter('jpeg_quality', 65)
        self.active = False
        self.previous_center_x = None
        self.misses = 0
        self.last_frame_time = None
        self.measured_fps = 0.0
        self.last_log_time = time.monotonic()
        self.frame_count = 0
        self.bridge = CvBridge()
        image_qos = QoSProfile(depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.image_callback, image_qos)
        self.create_subscription(String, '/robot_action', self.action_callback, 10)
        self.detected_pub = self.create_publisher(Bool, '/color_line_detected', 10)
        self.error_pub = self.create_publisher(Float32, '/color_line_error', 10)
        self.debug_pub = self.create_publisher(
            CompressedImage, '/color_line_debug_image/compressed', image_qos)
        cv2.setNumThreads(1)
        self.window_name = 'Pulsar Color Line Following'
        self.show_image = self.get_parameter('show_image').value
        if self.show_image:
            if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            else:
                self.get_logger().warning(
                    'show_image requested, but no graphical display is available')
                self.show_image = False
        self.get_logger().info('Color line detector ready; waiting for line_start')

    def action_callback(self, msg):
        action = msg.data.strip().upper()
        if action == 'LINE_START':
            self.active = True
            self.previous_center_x = None
            self.misses = 0
            self.last_frame_time = None
            self.measured_fps = 0.0
        elif action == 'LINE_STOP':
            self.active = False
            self.previous_center_x = None
            self.detected_pub.publish(Bool(data=False))

    def image_callback(self, msg):
        if not self.active:
            return
        started = time.perf_counter()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detection = detect_orange_line(
                frame,
                roi_start_ratio=self.get_parameter('roi_start_ratio').value,
                roi_end_ratio=self.get_parameter('roi_end_ratio').value,
                hsv_lower=self.get_parameter('hsv_lower').value,
                hsv_upper=self.get_parameter('hsv_upper').value,
                min_area_ratio=self.get_parameter('min_area_ratio').value,
                minimum_height_ratio=self.get_parameter('minimum_height_ratio').value,
                previous_center_x=self.previous_center_x,
                maximum_center_jump_ratio=self.get_parameter(
                    'maximum_center_jump_ratio').value,
            )
        except Exception as error:
            self.get_logger().error(f'Camera processing failed: {error}')
            self.detected_pub.publish(Bool(data=False))
            return
        if detection.center is None:
            self.misses += 1
            if self.misses >= self.get_parameter('tracking_reset_frames').value:
                self.previous_center_x = None
        else:
            self.previous_center_x = detection.center[0]
            self.misses = 0
        self.detected_pub.publish(Bool(data=detection.center is not None))
        if detection.error is not None:
            self.error_pub.publish(Float32(data=detection.error))
        now = time.perf_counter()
        if self.last_frame_time is not None and now > self.last_frame_time:
            fps = 1.0 / (now - self.last_frame_time)
            self.measured_fps = (fps if self.measured_fps == 0
                                 else 0.9 * self.measured_fps + 0.1 * fps)
        self.last_frame_time = now
        has_debug_subscribers = (
            self.get_parameter('publish_debug_image').value
            and self.debug_pub.get_subscription_count() > 0)
        if self.show_image or has_debug_subscribers:
            image = draw_detection(frame, detection)
            cv2.putText(image, f'FPS {self.measured_fps:.1f}', (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if self.show_image:
                cv2.imshow(self.window_name, image)
                cv2.waitKey(1)
        if has_debug_subscribers:
            quality = max(1, min(100, self.get_parameter('jpeg_quality').value))
            ok, buffer = cv2.imencode(
                '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok:
                debug = CompressedImage()
                debug.header = msg.header
                debug.format = 'jpeg'
                debug.data = buffer.tobytes()
                self.debug_pub.publish(debug)
        self.frame_count += 1
        if time.monotonic() - self.last_log_time >= 5:
            self.get_logger().info(
                f'Color detection: {self.measured_fps:.1f} FPS | '
                f'last callback={1000 * (time.perf_counter() - started):.1f} ms')
            self.last_log_time = time.monotonic()

    def destroy_node(self):
        if self.show_image:
            cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
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
