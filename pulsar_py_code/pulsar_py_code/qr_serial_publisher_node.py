#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import serial


class QrSerialPublisher(Node):

    def __init__(self):
        super().__init__('qr_serial_publisher')

        self.declare_parameter(
            'port',
            '/dev/ttyACM1'
        )

        self.declare_parameter(
            'baudrate',
            115200
        )

        self.declare_parameter(
            'topic',
            '/qr_code'
        )

        self.port = (
            self.get_parameter('port')
            .get_parameter_value()
            .string_value
        )

        self.baudrate = (
            self.get_parameter('baudrate')
            .get_parameter_value()
            .integer_value
        )

        self.topic = (
            self.get_parameter('topic')
            .get_parameter_value()
            .string_value
        )

        self.publisher_ = self.create_publisher(
            String,
            self.topic,
            10
        )

        self.serial_port = None
        self.receive_buffer = bytearray()

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.01
            )

            self.get_logger().info(
                f'QR scanner opened: '
                f'{self.port} @ {self.baudrate}'
            )

            self.get_logger().info(
                f'Publishing QR data to: '
                f'{self.topic}'
            )

        except serial.SerialException as error:
            self.get_logger().error(
                f'Could not open QR scanner: {error}'
            )

        self.timer = self.create_timer(
            0.01,
            self.read_serial
        )

    def read_serial(self):

        if self.serial_port is None:
            return

        if not self.serial_port.is_open:
            return

        try:
            waiting = self.serial_port.in_waiting

            if waiting <= 0:
                return

            data = self.serial_port.read(
                waiting
            )

            self.receive_buffer.extend(data)

            while b'\n' in self.receive_buffer:

                line, _, remaining = (
                    self.receive_buffer.partition(
                        b'\n'
                    )
                )

                self.receive_buffer = bytearray(
                    remaining
                )

                qr_text = line.decode(
                    'utf-8',
                    errors='ignore'
                ).strip().upper()

                if not qr_text:
                    continue

                self.publish_qr(qr_text)

        except serial.SerialException as error:
            self.get_logger().error(
                f'Serial read error: {error}'
            )

    def publish_qr(self, qr_text):

        message = String()
        message.data = qr_text

        self.publisher_.publish(message)

        self.get_logger().info(
            f'Published QR: {qr_text}'
        )

    def destroy_node(self):

        if (
            self.serial_port is not None
            and self.serial_port.is_open
        ):
            self.serial_port.close()

            self.get_logger().info(
                'QR scanner serial port closed.'
            )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = QrSerialPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
