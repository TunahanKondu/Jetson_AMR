import threading
import time

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, String

from pulsar_line_following.vision_processing import (
    detect_line,
    draw_detection,
)


class LineDetectorNode(Node):

    def __init__(self):
        super().__init__('line_detector_node')

        # =================================================
        # Parameters
        # =================================================

        self.declare_parameter(
            'image_topic',
            '/camera/camera/color/image_raw',
        )

        self.declare_parameter(
            'show_image',
            True,
        )

        self.declare_parameter(
            'publish_debug_image',
            True,
        )

        self.declare_parameter(
            'jpeg_quality',
            75,
        )

        self.declare_parameter(
            'show_mask',
            False,
        )

        self.declare_parameter(
            'display_width',
            1280,
        )

        self.declare_parameter(
            'display_height',
            720,
        )

        self.declare_parameter(
            'roi_start_ratio',
            0.55,
        )

        self.declare_parameter(
            'threshold',
            80,
        )

        self.declare_parameter(
            'invert_threshold',
            True,
        )

        self.declare_parameter(
            'blur_kernel_size',
            5,
        )

        self.declare_parameter(
            'morph_kernel_size',
            5,
        )

        self.declare_parameter(
            'min_contour_area',
            500.0,
        )

        self.declare_parameter(
            'roi_left_ratio',
            0.10,
        )

        self.declare_parameter(
            'roi_right_ratio',
            0.90,
        )

        self.declare_parameter(
            'minimum_line_width_ratio',
            0.015,
        )

        self.declare_parameter(
            'maximum_line_width_ratio',
            0.14,
        )

        self.declare_parameter(
            'minimum_line_height_ratio',
            0.20,
        )

        self.declare_parameter(
            'expected_line_width_ratio',
            0.06,
        )

        self.declare_parameter(
            'maximum_center_jump_ratio',
            0.20,
        )

        self.declare_parameter(
            'tracking_reset_frames',
            5,
        )

        # =================================================
        # GStreamer parameters
        # =================================================

        self.declare_parameter(
            'gstreamer_enabled',
            True,
        )

        self.declare_parameter(
            'gstreamer_host',
            '192.168.68.119',
        )

        self.declare_parameter(
            'gstreamer_port',
            5000,
        )

        self.declare_parameter(
            'gstreamer_bitrate',
            2000000,
        )

        self.declare_parameter(
            'gstreamer_fps',
            30,
        )

        # =================================================
        # Read parameters
        # =================================================

        self.image_topic = self.get_parameter(
            'image_topic'
        ).value

        self.show_image = self.get_parameter(
            'show_image'
        ).value

        self.publish_debug_image = self.get_parameter(
            'publish_debug_image'
        ).value

        self.jpeg_quality = int(
            self.get_parameter(
                'jpeg_quality'
            ).value
        )

        self.show_mask = self.get_parameter(
            'show_mask'
        ).value

        self.display_width = int(
            self.get_parameter(
                'display_width'
            ).value
        )

        self.display_height = int(
            self.get_parameter(
                'display_height'
            ).value
        )

        self.roi_start_ratio = self.get_parameter(
            'roi_start_ratio'
        ).value

        self.threshold = self.get_parameter(
            'threshold'
        ).value

        self.invert_threshold = self.get_parameter(
            'invert_threshold'
        ).value

        self.blur_kernel_size = self.get_parameter(
            'blur_kernel_size'
        ).value

        self.morph_kernel_size = self.get_parameter(
            'morph_kernel_size'
        ).value

        self.min_contour_area = self.get_parameter(
            'min_contour_area'
        ).value

        self.roi_left_ratio = self.get_parameter(
            'roi_left_ratio'
        ).value

        self.roi_right_ratio = self.get_parameter(
            'roi_right_ratio'
        ).value

        self.minimum_line_width_ratio = (
            self.get_parameter(
                'minimum_line_width_ratio'
            ).value
        )

        self.maximum_line_width_ratio = (
            self.get_parameter(
                'maximum_line_width_ratio'
            ).value
        )

        self.minimum_line_height_ratio = (
            self.get_parameter(
                'minimum_line_height_ratio'
            ).value
        )

        self.expected_line_width_ratio = (
            self.get_parameter(
                'expected_line_width_ratio'
            ).value
        )

        self.maximum_center_jump_ratio = (
            self.get_parameter(
                'maximum_center_jump_ratio'
            ).value
        )

        self.tracking_reset_frames = int(
            self.get_parameter(
                'tracking_reset_frames'
            ).value
        )

        # =================================================
        # Read GStreamer parameters
        # =================================================

        self.gstreamer_enabled = self.get_parameter(
            'gstreamer_enabled'
        ).value

        self.gstreamer_host = self.get_parameter(
            'gstreamer_host'
        ).value

        self.gstreamer_port = int(
            self.get_parameter(
                'gstreamer_port'
            ).value
        )

        self.gstreamer_bitrate = int(
            self.get_parameter(
                'gstreamer_bitrate'
            ).value
        )

        self.gstreamer_fps = int(
            self.get_parameter(
                'gstreamer_fps'
            ).value
        )

        # =================================================
        # Validation
        # =================================================

        if self.tracking_reset_frames < 1:
            raise ValueError(
                'tracking_reset_frames must be at least one'
            )

        if self.gstreamer_port <= 0:
            raise ValueError(
                'gstreamer_port must be greater than zero'
            )

        if self.gstreamer_bitrate <= 0:
            raise ValueError(
                'gstreamer_bitrate must be greater than zero'
            )

        if self.gstreamer_fps <= 0:
            raise ValueError(
                'gstreamer_fps must be greater than zero'
            )

        # =================================================
        # Main activation state
        # =================================================

        # Node starts inactive.
        # No OpenCV processing happens until "line_start".
        self.line_mode_active = False

        # =================================================
        # CvBridge
        # =================================================

        self.bridge = CvBridge()

        # =================================================
        # QoS
        # =================================================

        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # =================================================
        # Subscribers
        # =================================================

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos,
        )

        self.robot_action_subscription = self.create_subscription(
            String,
            '/robot_action',
            self.robot_action_callback,
            10,
        )

        # =================================================
        # Publishers
        # =================================================

        self.detection_publisher = self.create_publisher(
            Bool,
            'line_detected',
            10,
        )

        self.error_publisher = self.create_publisher(
            Float32,
            'line_error',
            10,
        )

        self.compressed_image_publisher = self.create_publisher(
            CompressedImage,
            'debug_image/compressed',
            debug_qos,
        )

        # =================================================
        # Internal state
        # =================================================

        self.frame_count = 0
        self.last_frame_time = None
        self.measured_fps = 0.0

        self.timing_sample_count = 0
        self.conversion_time_total = 0.0
        self.detection_time_total = 0.0
        self.display_time_total = 0.0

        self.latest_debug_frame = None
        self.latest_mask = None

        self.display_lock = threading.Lock()
        self.display_stop_event = threading.Event()
        self.display_frame_event = threading.Event()

        self.display_thread = None

        self.previous_line_center_x = None
        self.tracking_miss_count = 0

        self.gstreamer_writer = None

        cv2.setNumThreads(1)

        # =================================================
        # Local OpenCV display
        # =================================================

        if self.show_image:
            self.display_thread = threading.Thread(
                target=self.display_loop,
                name='line_detector_display',
                daemon=True,
            )

            self.display_thread.start()

        # =================================================
        # Logs
        # =================================================

        self.get_logger().info(
            'Line detector node started.'
        )

        self.get_logger().info(
            f'Image topic: {self.image_topic}'
        )

        self.get_logger().info(
            'Line mode starts DISABLED.'
        )

        self.get_logger().info(
            'Waiting for /robot_action = line_start'
        )

        self.get_logger().info(
            f'GStreamer enabled: {self.gstreamer_enabled}'
        )

        if self.gstreamer_enabled:
            self.get_logger().info(
                'GStreamer destination: '
                f'{self.gstreamer_host}:'
                f'{self.gstreamer_port}'
            )

    # =====================================================
    # Robot action callback
    # =====================================================

    def robot_action_callback(self, msg):

        action = msg.data.strip().lower()

        if action == 'line_start':

            if self.line_mode_active:
                return

            self.line_mode_active = True

            # Reset tracking state when a new line-following
            # operation begins.
            self.previous_line_center_x = None
            self.tracking_miss_count = 0

            self.frame_count = 0
            self.last_frame_time = None
            self.measured_fps = 0.0

            self.get_logger().info(
                'LINE_START received.'
            )

            self.get_logger().info(
                'Line detection is now ACTIVE.'
            )

        elif action == 'line_stop':

            if not self.line_mode_active:
                return

            self.line_mode_active = False

            self.previous_line_center_x = None
            self.tracking_miss_count = 0

            self.get_logger().info(
                'LINE_STOP received.'
            )

            self.get_logger().info(
                'Line detection is now INACTIVE.'
            )

            # Stop the network stream when line mode stops.
            if self.gstreamer_writer is not None:

                self.get_logger().info(
                    'Stopping GStreamer stream.'
                )

                self.gstreamer_writer.release()
                self.gstreamer_writer = None

        else:

            self.get_logger().debug(
                f'Ignoring robot action: {msg.data}'
            )

    # =====================================================
    # GStreamer initialization
    # =====================================================

    def initialize_gstreamer(
        self,
        width,
        height,
    ):

        pipeline = (
            'appsrc '
            'is-live=true '
            'block=false '
            'format=time ! '

            f'video/x-raw,'
            f'format=BGR,'
            f'width={width},'
            f'height={height},'
            f'framerate={self.gstreamer_fps}/1 ! '

            'videoconvert ! '

            'nvvidconv ! '

            'video/x-raw('
            'memory:NVMM'
            '),'
            'format=NV12 ! '

            f'nvv4l2h264enc '
            f'bitrate={self.gstreamer_bitrate} ! '

            'h264parse ! '

            'rtph264pay '
            'config-interval=1 '
            'pt=96 ! '

            f'udpsink '
            f'host={self.gstreamer_host} '
            f'port={self.gstreamer_port} '
            'sync=false '
            'async=false'
        )

        self.get_logger().info(
            'Opening GStreamer stream...'
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
            'GStreamer stream started -> '
            f'{self.gstreamer_host}:'
            f'{self.gstreamer_port}'
        )

        return True

    # =====================================================
    # Camera callback
    # =====================================================

    def image_callback(self, msg):

        # -------------------------------------------------
        # VERY IMPORTANT
        #
        # Before line_start:
        #   - no CvBridge conversion
        #   - no OpenCV processing
        #   - no line detection
        #   - no debug drawing
        #   - no GStreamer encoding
        #
        # Callback returns immediately.
        # -------------------------------------------------

        if not self.line_mode_active:
            return

        callback_start = time.perf_counter()

        # -------------------------------------------------
        # ROS image -> OpenCV BGR
        # -------------------------------------------------

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )

        except Exception as error:

            self.get_logger().error(
                f'CvBridge error: {error}'
            )

            return

        conversion_finished = time.perf_counter()

        # -------------------------------------------------
        # FPS
        # -------------------------------------------------

        self.frame_count += 1

        current_time = time.perf_counter()

        if self.last_frame_time is not None:

            frame_interval = (
                current_time
                - self.last_frame_time
            )

            if frame_interval > 0.0:

                instantaneous_fps = (
                    1.0 / frame_interval
                )

                if self.measured_fps == 0.0:

                    self.measured_fps = (
                        instantaneous_fps
                    )

                else:

                    self.measured_fps = (
                        0.9 * self.measured_fps
                        + 0.1 * instantaneous_fps
                    )

        self.last_frame_time = current_time

        # -------------------------------------------------
        # First frame
        # -------------------------------------------------

        if self.frame_count == 1:

            height, width = frame.shape[:2]

            self.get_logger().info(
                'First active image received: '
                f'{width}x{height}'
            )

            self.get_logger().info(
                f'ROS image encoding: {msg.encoding}'
            )

        # -------------------------------------------------
        # LINE DETECTION
        # -------------------------------------------------

        detection = detect_line(
            frame,
            roi_start_ratio=self.roi_start_ratio,
            threshold=self.threshold,
            invert_threshold=self.invert_threshold,
            blur_kernel_size=self.blur_kernel_size,
            morph_kernel_size=self.morph_kernel_size,
            min_contour_area=self.min_contour_area,
            roi_left_ratio=self.roi_left_ratio,
            roi_right_ratio=self.roi_right_ratio,
            minimum_line_width_ratio=(
                self.minimum_line_width_ratio
            ),
            maximum_line_width_ratio=(
                self.maximum_line_width_ratio
            ),
            minimum_line_height_ratio=(
                self.minimum_line_height_ratio
            ),
            expected_line_width_ratio=(
                self.expected_line_width_ratio
            ),
            previous_center_x=(
                self.previous_line_center_x
            ),
            maximum_center_jump_ratio=(
                self.maximum_center_jump_ratio
            ),
        )

        detection_finished = time.perf_counter()

        # -------------------------------------------------
        # Tracking state
        # -------------------------------------------------

        if detection.center is not None:

            self.previous_line_center_x = (
                detection.center[0]
            )

            self.tracking_miss_count = 0

        else:

            self.tracking_miss_count += 1

            if (
                self.tracking_miss_count
                >= self.tracking_reset_frames
            ):

                self.previous_line_center_x = None

        # -------------------------------------------------
        # Publish whether line exists
        # -------------------------------------------------

        detected_message = Bool()

        detected_message.data = (
            detection.center is not None
        )

        self.detection_publisher.publish(
            detected_message
        )

        # -------------------------------------------------
        # Publish line error
        # -------------------------------------------------

        if detection.error is not None:

            error_message = Float32()

            error_message.data = float(
                detection.error
            )

            self.error_publisher.publish(
                error_message
            )

        # -------------------------------------------------
        # Does an annotated frame need to exist?
        # -------------------------------------------------

        has_debug_subscribers = (
            self.compressed_image_publisher
            .get_subscription_count()
            > 0
        )

        needs_debug_frame = (
            self.show_image
            or self.gstreamer_enabled
            or (
                self.publish_debug_image
                and has_debug_subscribers
            )
        )

        debug_frame = None

        # -------------------------------------------------
        # Create annotated frame
        # -------------------------------------------------

        if needs_debug_frame:

            debug_frame = draw_detection(
                frame,
                detection,
            )

            cv2.putText(
                debug_frame,
                f'FPS {self.measured_fps:.1f}',
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

            # ---------------------------------------------
            # GStreamer processed image
            # ---------------------------------------------

            if self.gstreamer_enabled:

                if self.gstreamer_writer is None:

                    height, width = (
                        debug_frame.shape[:2]
                    )

                    self.initialize_gstreamer(
                        width,
                        height,
                    )

                if self.gstreamer_writer is not None:

                    self.gstreamer_writer.write(
                        debug_frame
                    )

        # -------------------------------------------------
        # ROS compressed debug image
        # -------------------------------------------------

        if (
            self.publish_debug_image
            and has_debug_subscribers
            and debug_frame is not None
        ):

            jpeg_quality = max(
                1,
                min(
                    self.jpeg_quality,
                    100,
                ),
            )

            encoded, jpeg_buffer = cv2.imencode(
                '.jpg',
                debug_frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    jpeg_quality,
                ],
            )

            if encoded:

                debug_message = CompressedImage()

                debug_message.header = msg.header
                debug_message.format = 'jpeg'

                debug_message.data = (
                    jpeg_buffer.tobytes()
                )

                self.compressed_image_publisher.publish(
                    debug_message
                )

            else:

                self.get_logger().warning(
                    'JPEG encoding failed.'
                )

        # -------------------------------------------------
        # Local OpenCV display
        # -------------------------------------------------

        if (
            self.show_image
            and debug_frame is not None
        ):

            with self.display_lock:

                self.latest_debug_frame = (
                    debug_frame
                )

                if self.show_mask:

                    self.latest_mask = (
                        detection.mask
                    )

            self.display_frame_event.set()

        # -------------------------------------------------
        # Performance statistics
        # -------------------------------------------------

        display_finished = time.perf_counter()

        self.conversion_time_total += (
            conversion_finished
            - callback_start
        )

        self.detection_time_total += (
            detection_finished
            - conversion_finished
        )

        self.display_time_total += (
            display_finished
            - detection_finished
        )

        self.timing_sample_count += 1

        if self.timing_sample_count >= 150:

            samples = float(
                self.timing_sample_count
            )

            self.get_logger().info(
                'Performance: '
                f'{self.measured_fps:.1f} FPS | '
                f'convert='
                f'{1000.0 * self.conversion_time_total / samples:.2f} ms, '
                f'detect='
                f'{1000.0 * self.detection_time_total / samples:.2f} ms, '
                f'output='
                f'{1000.0 * self.display_time_total / samples:.2f} ms'
            )

            self.timing_sample_count = 0

            self.conversion_time_total = 0.0
            self.detection_time_total = 0.0
            self.display_time_total = 0.0

    # =====================================================
    # Local display thread
    # =====================================================

    def display_loop(self):

        cv2.namedWindow(
            'AC Line Following - Detection',
            cv2.WINDOW_NORMAL
            | cv2.WINDOW_KEEPRATIO,
        )

        cv2.resizeWindow(
            'AC Line Following - Detection',
            self.display_width,
            self.display_height,
        )

        if self.show_mask:

            cv2.namedWindow(
                'AC Line Following - Mask',
                cv2.WINDOW_AUTOSIZE,
            )

        while (
            not self.display_stop_event.is_set()
        ):

            if not self.display_frame_event.wait(
                timeout=0.1
            ):

                cv2.waitKey(1)
                continue

            self.display_frame_event.clear()

            with self.display_lock:

                debug_frame = (
                    self.latest_debug_frame
                )

                mask = (
                    self.latest_mask
                )

            if debug_frame is not None:

                cv2.imshow(
                    'AC Line Following - Detection',
                    debug_frame,
                )

                if (
                    self.show_mask
                    and mask is not None
                ):

                    cv2.imshow(
                        'AC Line Following - Mask',
                        mask,
                    )

            cv2.waitKey(1)

        cv2.destroyAllWindows()

    # =====================================================
    # Shutdown
    # =====================================================

    def destroy_node(self):

        self.line_mode_active = False

        self.display_stop_event.set()
        self.display_frame_event.set()

        if self.display_thread is not None:

            self.display_thread.join(
                timeout=2.0
            )

        if self.gstreamer_writer is not None:

            self.get_logger().info(
                'Stopping GStreamer stream.'
            )

            self.gstreamer_writer.release()

            self.gstreamer_writer = None

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = LineDetectorNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    except RuntimeError:

        if rclpy.ok():
            raise

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
