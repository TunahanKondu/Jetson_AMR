#!/usr/bin/env python3

import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import UInt8
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Bool

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException


class PlcUdpBridge(Node):

    def __init__(self):
        super().__init__('plc_udp_bridge')

        # -------------------------------------------------
        # Parametreler
        # -------------------------------------------------
        self.declare_parameter('local_ip', '0.0.0.0')
        self.declare_parameter('local_port', 0)

        self.declare_parameter('plc_ip', '127.0.0.1')
        self.declare_parameter('plc_port', 1515)

        self.declare_parameter('send_period_ms', 1000)
        self.declare_parameter('connection_timeout_ms', 2500)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        self.local_ip = self.get_parameter(
            'local_ip'
        ).get_parameter_value().string_value

        self.local_port = self.get_parameter(
            'local_port'
        ).get_parameter_value().integer_value

        self.plc_ip = self.get_parameter(
            'plc_ip'
        ).get_parameter_value().string_value

        self.plc_port = self.get_parameter(
            'plc_port'
        ).get_parameter_value().integer_value

        self.send_period_ms = self.get_parameter(
            'send_period_ms'
        ).get_parameter_value().integer_value

        self.connection_timeout_ms = self.get_parameter(
            'connection_timeout_ms'
        ).get_parameter_value().integer_value

        self.map_frame = self.get_parameter(
            'map_frame'
        ).get_parameter_value().string_value

        self.robot_frame = self.get_parameter(
            'robot_frame'
        ).get_parameter_value().string_value

        # -------------------------------------------------
        # PLC'ye gönderilecek güncel bilgiler
        # -------------------------------------------------
        self.robot_status = 1
        self.pickup_station = 0
        self.dropoff_station = 0

        self.last_rx_time = None
        self.connection_state = False

        self.tx_packet_count = 0
        self.rx_packet_count = 0

        # -------------------------------------------------
        # TF
        # -------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # -------------------------------------------------
        # ROS subscriber'ları
        # -------------------------------------------------
        self.status_sub = self.create_subscription(
            UInt8,
            '/plc/tx_status',
            self.status_callback,
            10
        )

        self.pickup_sub = self.create_subscription(
            UInt8,
            '/plc/tx_pickup',
            self.pickup_callback,
            10
        )

        self.dropoff_sub = self.create_subscription(
            UInt8,
            '/plc/tx_dropoff',
            self.dropoff_callback,
            10
        )

        # -------------------------------------------------
        # ROS publisher'ları
        # -------------------------------------------------
        self.mission_pub = self.create_publisher(
            UInt8MultiArray,
            '/plc/mission_command',
            10
        )

        self.connected_pub = self.create_publisher(
            Bool,
            '/plc/connected',
            10
        )

        self.raw_rx_pub = self.create_publisher(
            UInt8MultiArray,
            '/plc/raw_rx',
            10
        )

        # -------------------------------------------------
        # UDP socket
        # -------------------------------------------------
        self.socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.socket.setblocking(False)

        self.socket.bind((
            self.local_ip,
            self.local_port
        ))

        self.socket.connect((
            self.plc_ip,
            self.plc_port
        ))

        # Her 1 saniyede TX
        self.send_timer = self.create_timer(
            self.send_period_ms / 1000.0,
            self.send_packet
        )

        # Gelen UDP paketlerini sık kontrol et
        self.receive_timer = self.create_timer(
            0.02,
            self.receive_packets
        )

        # Bağlantı zaman aşımı kontrolü
        self.connection_timer = self.create_timer(
            0.2,
            self.check_connection
        )

        self.get_logger().info(
            'PLC UDP bridge başlatıldı. '
            f'Local={self.local_ip}:{self.local_port}, '
            f'PLC={self.plc_ip}:{self.plc_port}'
        )

    # =====================================================
    # ROS CALLBACK'LERİ
    # =====================================================

    def status_callback(self, msg):
        if 1 <= msg.data <= 8:
            self.robot_status = msg.data
        else:
            self.get_logger().warning(
                f'Geçersiz robot durum kodu: {msg.data}'
            )

    def pickup_callback(self, msg):
        if 0 <= msg.data <= 3:
            self.pickup_station = msg.data
        else:
            self.get_logger().warning(
                f'Geçersiz yük alma istasyonu: {msg.data}'
            )

    def dropoff_callback(self, msg):
        if 0 <= msg.data <= 3:
            self.dropoff_station = msg.data
        else:
            self.get_logger().warning(
                f'Geçersiz yük bırakma noktası: {msg.data}'
            )

    # =====================================================
    # ROBOT POZİSYONU
    # =====================================================

    def get_robot_position_cm(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time()
            )

            x_m = transform.transform.translation.x
            y_m = transform.transform.translation.y

            # Şartname: integer(metre * 100)
            x_cm = int(x_m * 100.0)
            y_cm = int(y_m * 100.0)

            # Int16 sınırı
            x_cm = max(-32768, min(32767, x_cm))
            y_cm = max(-32768, min(32767, y_cm))

            return x_cm, y_cm

        except TransformException:
            return 0, 0

    # =====================================================
    # PLC'YE 7 BYTE GÖNDER
    # =====================================================

    def send_packet(self):
        x_cm, y_cm = self.get_robot_position_cm()

        try:
            packet = struct.pack(
                '<BBBhh',
                self.robot_status,
                self.pickup_station,
                self.dropoff_station,
                x_cm,
                y_cm
            )

            if len(packet) != 7:
                self.get_logger().error(
                    f'TX paket boyutu hatalı: {len(packet)}'
                )
                return

            self.socket.send(packet)
            self.tx_packet_count += 1

            self.get_logger().info(
                'PLC TX | '
                f'durum={self.robot_status}, '
                f'alım={self.pickup_station}, '
                f'bırakma={self.dropoff_station}, '
                f'x={x_cm}, y={y_cm}, '
                f'sayaç={self.tx_packet_count}'
            )

        except OSError as error:
            self.get_logger().error(
                f'UDP gönderme hatası: {error}'
            )

    # =====================================================
    # PLC'DEN 3 BYTE AL
    # =====================================================

    def receive_packets(self):
        while True:
            try:
                data = self.socket.recv(64)

            except BlockingIOError:
                break

            except OSError as error:
                self.get_logger().error(
                    f'UDP alma hatası: {error}'
                )
                break

            if len(data) != 3:
                self.get_logger().warning(
                    f'Hatalı RX paket boyutu: {len(data)} byte'
                )
                continue

            pickup, dropoff, control = struct.unpack(
                '<BBB',
                data
            )

            if pickup not in (1, 2, 3):
                self.get_logger().warning(
                    f'Geçersiz RX alım istasyonu: {pickup}'
                )
                continue

            if dropoff not in (1, 2, 3):
                self.get_logger().warning(
                    f'Geçersiz RX bırakma noktası: {dropoff}'
                )
                continue

            if control not in (1, 2):
                self.get_logger().warning(
                    f'Geçersiz RX kontrol bilgisi: {control}'
                )
                continue

            self.last_rx_time = time.monotonic()
            self.rx_packet_count += 1

            mission_msg = UInt8MultiArray()
            mission_msg.data = [
                pickup,
                dropoff,
                control
            ]
            self.mission_pub.publish(mission_msg)

            raw_msg = UInt8MultiArray()
            raw_msg.data = list(data)
            self.raw_rx_pub.publish(raw_msg)

            self.set_connection_state(True)

            control_text = (
                'BEKLE'
                if control == 1
                else 'BAŞLA/DEVAM ET'
            )

            self.get_logger().info(
                'PLC RX | '
                f'A{pickup} -> B{dropoff}, '
                f'kontrol={control_text}, '
                f'sayaç={self.rx_packet_count}'
            )

    # =====================================================
    # BAĞLANTI KONTROLÜ
    # =====================================================

    def check_connection(self):
        if self.last_rx_time is None:
            self.set_connection_state(False)
            return

        elapsed_ms = (
            time.monotonic() - self.last_rx_time
        ) * 1000.0

        connected = (
            elapsed_ms <= self.connection_timeout_ms
        )

        self.set_connection_state(connected)

    def set_connection_state(self, connected):
        if self.connection_state == connected:
            return

        self.connection_state = connected

        msg = Bool()
        msg.data = connected
        self.connected_pub.publish(msg)

        if connected:
            self.get_logger().info(
                'PLC bağlantısı kuruldu.'
            )
        else:
            self.get_logger().warning(
                'PLC bağlantısı zaman aşımına uğradı.'
            )

    def destroy_node(self):
        try:
            self.socket.close()
        except OSError:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PlcUdpBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
