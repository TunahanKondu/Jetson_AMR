"""ROS 2 camera node for the orange center stripe with optional GStreamer UDP streaming."""

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

        # Camera / vision parameters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('roi_start_ratio', 0.45)
        self.declare_parameter('roi_end_ratio', 0.88)
        self.declare_parameter('hsv_lower', [5, 80, 70])
        self.declare_parameter('hsv_upper', [25, 255, 255])
        self.declare_parameter('min_area_ratio', 0.0005)
        self.declare_parameter('minimum_height_ratio', 0.18)
        self.declare_parameter('maximum_center_jump_ratio', 0.25)
        self.declare_parameter('tracking_reset_frames', 5)

        # Debug image parameters
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_image', False)
        self.declare_parameter('jpeg_quality', 65)

        # GStreamer parameters
        self.declare_parameter('gstreamer_enabled', True)
        self.declare_parameter('gstreamer_host', '192.168.68.119')
        self.declare_parameter('gstreamer_port', 5000)
        self.declare_parameter('gstreamer_bitrate', 2000000)
        self.declare_parameter('gstreamer_fps', 30)

        self.active = False
        self.previous_center_x = None
        self.misses = 0
        self.last_frame_time = None
        self.measured_fps = 0.0
        self.last_log_time = time.monotonic()
        self.frame_count = 0

        self.bridge = CvBridge()

        self.gstreamer_enabled = bool(
            self.get_parameter('gstreamer_enabled').value
        )
        self.gstreamer_host = str(
            self.get_parameter('gstreamer_host').value
        )
        self.gstreamer_port = int(
            self.get_parameter('gstreamer_port').value
        )
        self.gstreamer_bitrate = int(
            self.get_parameter('gstreamer_bitrate').value
        )
        self.gstreamer_fps = int(
            self.get_parameter('gstreamer_fps').value
        )
        self.gstreamer_writer = None

        if self.gstreamer_port <= 0:
            raise ValueError('gstreamer_port must be greater than zero')
        if self.gstreamer_bitrate <= 0:
            raise ValueError('gstreamer_bitrate must be greater than zero')
        if self.gstreamer_fps <= 0:
            raise ValueError('gstreamer_fps must be greater than zero')

        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            image_qos,
        )

        self.create_subscription(
            String,
            '/robot_action',
            self.action_callback,
            10,
        )

        self.detected_pub = self.create_publisher(
            Bool,
            '/color_line_detected',
            10,
        )

        self.error_pub = self.create_publisher(
            Float32,
            '/color_line_error',
            10,
        )

        self.debug_pub = self.create_publisher(
            CompressedImage,
            '/color_line_debug_image/compressed',
            image_qos,
        )

        cv2.setNumThreads(1)

        self.window_name = 'Pulsar Color Line Following'
        self.show_image = self.get_parameter('show_image').value

        if self.show_image:
            if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            else:
                self.get_logger().warning(
                    'show_image requested, but no graphical display is available'
                )
                self.show_image = False

        self.get_logger().info(
            'Color line detector ready; waiting for LINE_START'
        )

        self.get_logger().info(
            f'GStreamer enabled: {self.gstreamer_enabled}'
        )

        if self.gstreamer_enabled:
            self.get_logger().info(
                f'GStreamer destination: '
                f'{self.gstreamer_host}:{self.gstreamer_port}'
            )

    def action_callback(self, msg):
        action = msg.data.strip().upper()

        if action == 'LINE_START':
            self.active = True
            self.previous_center_x = None
            self.misses = 0
            self.last_frame_time = None
            self.measured_fps = 0.0

            self.get_logger().info(
                'LINE_START received; color detection active'
            )

        elif action == 'LINE_STOP':
            self.active = False
            self.previous_center_x = None
            self.misses = 0
            self.detected_pub.publish(Bool(data=False))

            self.stop_gstreamer()

            self.get_logger().info(
                'LINE_STOP received; color detection inactive'
            )

    def initialize_gstreamer(self, width, height):
        pipeline = (
            'appsrc '
            'is-live=true '
            'block=false '
            'format=time ! '
            f'video/x-raw,format=BGR,width={width},height={height},'
            f'framerate={self.gstreamer_fps}/1 ! '
            'videoconvert ! '
            'nvvidconv ! '
            'video/x-raw(memory:NVMM),format=NV12 ! '
            f'nvv4l2h264enc bitrate={self.gstreamer_bitrate} ! '
            'h264parse ! '
            'rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={self.gstreamer_host} '
            f'port={self.gstreamer_port} '
            'sync=false async=false'
        )

        self.get_logger().info(
            f'Opening GStreamer stream -> '
            f'{self.gstreamer_host}:{self.gstreamer_port}'
        )

        writer = cv2.VideoWriter(
            pipeline,
            cv2.CAP_GSTREAMER,
            0,
            self.gstreamer_fps,
            (width, height),
            True,
        )

        if not writer.isOpened():
            self.get_logger().error(
                'Could not open GStreamer stream.'
            )
            return False

        self.gstreamer_writer = writer

        self.get_logger().info(
            f'GStreamer stream started -> '
            f'{self.gstreamer_host}:{self.gstreamer_port}'
        )

        return True

    def stop_gstreamer(self):
        if self.gstreamer_writer is not None:
            self.get_logger().info('Stopping GStreamer stream.')
            self.gstreamer_writer.release()
            self.gstreamer_writer = None

    def image_callback(self, msg):
        if not self.active:
            return

        started = time.perf_counter()

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )

            detection = detect_orange_line(
                frame,
                roi_start_ratio=self.get_parameter(
                    'roi_start_ratio'
                ).value,
                roi_end_ratio=self.get_parameter(
                    'roi_end_ratio'
                ).value,
                hsv_lower=self.get_parameter(
                    'hsv_lower'
                ).value,
                hsv_upper=self.get_parameter(
                    'hsv_upper'
                ).value,
                min_area_ratio=self.get_parameter(
                    'min_area_ratio'
                ).value,
                minimum_height_ratio=self.get_parameter(
                    'minimum_height_ratio'
                ).value,
                previous_center_x=self.previous_center_x,
                maximum_center_jump_ratio=self.get_parameter(
                    'maximum_center_jump_ratio'
                ).value,
            )

        except Exception as error:
            self.get_logger().error(
                f'Camera processing failed: {error}'
            )
            self.detected_pub.publish(Bool(data=False))
            return

        if detection.center is None:
            self.misses += 1

            if self.misses >= self.get_parameter(
                'tracking_reset_frames'
            ).value:
                self.previous_center_x = None

        else:
            self.previous_center_x = detection.center[0]
            self.misses = 0

        self.detected_pub.publish(
            Bool(data=detection.center is not None)
        )

        if detection.error is not None:
            self.error_pub.publish(
                Float32(data=detection.error)
            )

        now = time.perf_counter()

        if (
            self.last_frame_time is not None
            and now > self.last_frame_time
        ):
            fps = 1.0 / (now - self.last_frame_time)

            self.measured_fps = (
                fps
                if self.measured_fps == 0
                else 0.9 * self.measured_fps + 0.1 * fps
            )

        self.last_frame_time = now

        has_debug_subscribers = (
            self.get_parameter(
                'publish_debug_image'
            ).value
            and self.debug_pub.get_subscription_count() > 0
        )

        needs_debug_frame = (
            self.show_image
            or self.gstreamer_enabled
            or has_debug_subscribers
        )

        image = None

        if needs_debug_frame:
            image = draw_detection(
                frame,
                detection,
            )

            cv2.putText(
                image,
                f'FPS {self.measured_fps:.1f}',
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

            if self.gstreamer_enabled:
                if self.gstreamer_writer is None:
                    height, width = image.shape[:2]

                    self.initialize_gstreamer(
                        width,
                        height,
                    )

                if self.gstreamer_writer is not None:
                    self.gstreamer_writer.write(
                        image
                    )

            if self.show_image:
                cv2.imshow(
                    self.window_name,
                    image,
                )
                cv2.waitKey(1)

        if (
            has_debug_subscribers
            and image is not None
        ):
            quality = max(
                1,
                min(
                    100,
                    self.get_parameter(
                        'jpeg_quality'
                    ).value,
                ),
            )

            ok, buffer = cv2.imencode(
                '.jpg',
                image,
                [cv2.IMWRITE_JPEG_QUALITY, quality],
            )

            if ok:
                debug = CompressedImage()
                debug.header = msg.header
                debug.format = 'jpeg'
                debug.data = buffer.tobytes()
                self.debug_pub.publish(debug)

        self.frame_count += 1

        if time.monotonic() - self.last_log_time >= 5:
            self.get_logger().info(
                f'Color detection: '
                f'{self.measured_fps:.1f} FPS | '
                f'last callback='
                f'{1000 * (time.perf_counter() - started):.1f} ms'
            )
            self.last_log_time = time.monotonic()

    def destroy_node(self):
        self.stop_gstreamer()

        if self.show_image:
            cv2.destroyWindow(
                self.window_name
            )

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
