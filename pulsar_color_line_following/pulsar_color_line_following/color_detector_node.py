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

try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
except (ImportError, ValueError):
    Gst = None

from pulsar_color_line_following.color_vision import (
    detect_orange_line,
    draw_detection,
)


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
        self.declare_parameter(
            'action_result_topic', '/amr/robot_action_result')

        # Optional low-latency RTP/H.264 stream for the QML GUI or a
        # gst-launch receiver.  The camera remains owned by realsense2_camera;
        # this node streams the ROS image instead of opening /dev/video*.
        self.declare_parameter('gstreamer_enabled', False)
        self.declare_parameter('gstreamer_host', '127.0.0.1')
        self.declare_parameter('gstreamer_port', 5000)
        self.declare_parameter('gstreamer_bitrate_kbps', 2000)
        self.declare_parameter('gstreamer_fps', 30.0)
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
        self.action_result_pub = self.create_publisher(
            String, self.get_parameter('action_result_topic').value, 10)
        self.debug_pub = self.create_publisher(
            CompressedImage, '/color_line_debug_image/compressed', image_qos)
        cv2.setNumThreads(1)
        self.window_name = 'Pulsar Color Line Following'
        self.show_image = self.get_parameter('show_image').value
        self.gstreamer_enabled = bool(
            self.get_parameter('gstreamer_enabled').value)
        self.gstreamer_host = str(
            self.get_parameter('gstreamer_host').value)
        self.gstreamer_port = int(
            self.get_parameter('gstreamer_port').value)
        self.gstreamer_bitrate_kbps = int(
            self.get_parameter('gstreamer_bitrate_kbps').value)
        self.gstreamer_fps = float(
            self.get_parameter('gstreamer_fps').value)
        self.gstreamer_writer = None
        self.gstreamer_pipeline = None
        self.gstreamer_appsrc = None
        self.gstreamer_backend = None
        self.gstreamer_frame_index = 0
        self.gstreamer_frame_size = None
        self.gstreamer_failure_logged = False
        self.gstreamer_last_open_attempt = 0.0
        self._validate_gstreamer_parameters()
        if self.show_image:
            if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            else:
                self.get_logger().warning(
                    'show_image requested, but no graphical display is available')
                self.show_image = False
        stream_status = (
            f'enabled -> udp://{self.gstreamer_host}:{self.gstreamer_port}'
            if self.gstreamer_enabled else 'disabled')
        self.get_logger().info(
            'Color line detector ready; waiting for LINE_START | '
            f'GStreamer {stream_status}')
        if self.gstreamer_enabled:
            self.get_logger().info(
                'GStreamer will start after LINE_START and stop after LINE_STOP')

    def _validate_gstreamer_parameters(self):
        if not self.gstreamer_host:
            raise ValueError('gstreamer_host must not be empty')
        if not 1 <= self.gstreamer_port <= 65535:
            raise ValueError('gstreamer_port must be between 1 and 65535')
        if self.gstreamer_bitrate_kbps <= 0:
            raise ValueError('gstreamer_bitrate_kbps must be positive')
        if self.gstreamer_fps <= 0:
            raise ValueError('gstreamer_fps must be positive')

    def _gstreamer_pipeline(self):
        return (
            'appsrc is-live=true do-timestamp=true format=time '
            '! queue max-size-buffers=2 leaky=downstream '
            '! videoconvert '
            '! video/x-raw,format=I420 '
            f'! x264enc tune=zerolatency speed-preset=ultrafast '
            f'bitrate={self.gstreamer_bitrate_kbps} '
            f'key-int-max={max(1, int(round(self.gstreamer_fps)))} '
            '! rtph264pay config-interval=1 pt=96 '
            f'! udpsink host={self.gstreamer_host} '
            f'port={self.gstreamer_port} sync=false async=false'
        )

    def _ensure_gstreamer_writer(self, frame):
        frame_size = (int(frame.shape[1]), int(frame.shape[0]))
        if ((self.gstreamer_writer is not None
                or self.gstreamer_appsrc is not None)
                and self.gstreamer_frame_size == frame_size):
            return True

        now = time.monotonic()
        if (self.gstreamer_failure_logged
                and now - self.gstreamer_last_open_attempt < 2.0):
            return False
        self.gstreamer_last_open_attempt = now

        self._close_gstreamer_writer()

        writer = cv2.VideoWriter(
            self._gstreamer_pipeline(),
            cv2.CAP_GSTREAMER,
            0,
            self.gstreamer_fps,
            frame_size,
            True,
        )
        if writer.isOpened():
            self.gstreamer_writer = writer
            self.gstreamer_backend = 'OpenCV/GStreamer'
            return self._finish_gstreamer_open(frame_size)
        writer.release()

        # Some Jetson OpenCV builds report GStreamer=NO even though the
        # system GStreamer installation works. Use the native Python binding
        # as a fallback so gst-launch compatibility is sufficient.
        if Gst is not None:
            try:
                Gst.init(None)
                width, height = frame_size
                fps_numerator = max(1, int(round(self.gstreamer_fps)))
                pipeline_text = (
                    'appsrc name=source is-live=true block=false format=time '
                    f'caps=video/x-raw,format=BGR,width={width},height={height},'
                    f'framerate={fps_numerator}/1 '
                    '! queue max-size-buffers=2 leaky=downstream '
                    '! videoconvert ! video/x-raw,format=I420 '
                    '! x264enc tune=zerolatency speed-preset=ultrafast '
                    f'bitrate={self.gstreamer_bitrate_kbps} '
                    f'key-int-max={fps_numerator} '
                    '! rtph264pay config-interval=1 pt=96 '
                    f'! udpsink host={self.gstreamer_host} '
                    f'port={self.gstreamer_port} sync=false async=false'
                )
                pipeline = Gst.parse_launch(pipeline_text)
                appsrc = pipeline.get_by_name('source')
                result = pipeline.set_state(Gst.State.PLAYING)
                if appsrc is None or result == Gst.StateChangeReturn.FAILURE:
                    pipeline.set_state(Gst.State.NULL)
                    raise RuntimeError('native GStreamer pipeline rejected')
                self.gstreamer_pipeline = pipeline
                self.gstreamer_appsrc = appsrc
                self.gstreamer_backend = 'native GStreamer'
                return self._finish_gstreamer_open(frame_size)
            except Exception as error:
                self.get_logger().error(
                    f'Native GStreamer fallback failed: {error}')

        if not self.gstreamer_failure_logged:
            self.get_logger().error(
                'GStreamer stream could not be opened by OpenCV or the '
                'native Python GStreamer binding')
            self.gstreamer_failure_logged = True
        return False

    def _finish_gstreamer_open(self, frame_size):
        self.gstreamer_frame_size = frame_size
        self.gstreamer_frame_index = 0
        self.gstreamer_failure_logged = False
        self.get_logger().info(
            'GStreamer stream started using '
            f'{self.gstreamer_backend}: {frame_size[0]}x{frame_size[1]} '
            f'@ {self.gstreamer_fps:.1f} FPS -> '
            f'udp://{self.gstreamer_host}:{self.gstreamer_port}')
        return True

    def _write_gstreamer_frame(self, frame):
        if self.gstreamer_writer is not None:
            self.gstreamer_writer.write(frame)
            return
        if self.gstreamer_appsrc is None or Gst is None:
            return
        if not frame.flags['C_CONTIGUOUS']:
            frame = frame.copy()
        data = frame.tobytes()
        buffer = Gst.Buffer.new_allocate(None, len(data), None)
        buffer.fill(0, data)
        duration = int(Gst.SECOND / self.gstreamer_fps)
        buffer.pts = self.gstreamer_frame_index * duration
        buffer.dts = buffer.pts
        buffer.duration = duration
        self.gstreamer_frame_index += 1
        flow = self.gstreamer_appsrc.emit('push-buffer', buffer)
        if (flow != Gst.FlowReturn.OK
                and not self.gstreamer_failure_logged):
            self.get_logger().error(
                f'GStreamer rejected a video frame: {flow}')
            self.gstreamer_failure_logged = True

    def _close_gstreamer_writer(self):
        if self.gstreamer_writer is not None:
            self.gstreamer_writer.release()
            self.gstreamer_writer = None
        if self.gstreamer_appsrc is not None:
            try:
                self.gstreamer_appsrc.emit('end-of-stream')
            except Exception:
                pass
            self.gstreamer_appsrc = None
        if self.gstreamer_pipeline is not None and Gst is not None:
            self.gstreamer_pipeline.set_state(Gst.State.NULL)
            self.gstreamer_pipeline = None
        self.gstreamer_backend = None
        self.gstreamer_frame_size = None

    def action_callback(self, msg):
        action = msg.data.strip().upper()
        if action == 'LINE_START':
            # LINE_START may be retried until the controller acknowledges it.
            # Do not reset tracking/GStreamer state after the first command.
            if self.active:
                return
            self.active = True
            self.previous_center_x = None
            self.misses = 0
            self.last_frame_time = None
            self.measured_fps = 0.0
        elif action == 'LINE_STOP':
            self.active = False
            self.previous_center_x = None
            self.detected_pub.publish(Bool(data=False))
            self._close_gstreamer_writer()
            result = String()
            result.data = 'LINE_DETECTOR_STOPPED'
            self.action_result_pub.publish(result)
            self.get_logger().info(
                'LINE_STOP received; line detection and GStreamer stopped; '
                'LINE_DETECTOR_STOPPED published')

    def image_callback(self, msg):
        started = time.perf_counter()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'Camera conversion failed: {error}')
            return

        if not self.active:
            if self.show_image:
                cv2.imshow(self.window_name, frame)
                cv2.waitKey(1)
            return

        try:
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
            self.get_logger().error(f'Line detection failed: {error}')
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
        needs_visual_output = (
            self.show_image
            or has_debug_subscribers
            or self.gstreamer_enabled)
        if needs_visual_output:
            image = draw_detection(frame, detection)
            cv2.putText(image, f'FPS {self.measured_fps:.1f}', (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if self.show_image:
                cv2.imshow(self.window_name, image)
                cv2.waitKey(1)
        if (self.gstreamer_enabled
                and self._ensure_gstreamer_writer(image)):
            self._write_gstreamer_frame(image)
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
        self._close_gstreamer_writer()
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

