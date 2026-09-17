#!/usr/bin/env python3

import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy

from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import UInt8MultiArray


class PlcMissionAdapter(Node):

    def __init__(self):
        super().__init__('plc_mission_adapter')

        # PLC'den gelen görev
        self.plc_mission_sub = self.create_subscription(
            UInt8MultiArray,
            '/plc/mission_command',
            self.plc_mission_callback,
            10
        )

        # MissionManager komut çıkışı
        self.mission_command_pub = self.create_publisher(
            String,
            '/amr/mission_command',
            10
        )

        # PLC TX bilgileri
        self.plc_status_pub = self.create_publisher(
            UInt8,
            '/plc/tx_status',
            10
        )

        self.plc_pickup_pub = self.create_publisher(
            UInt8,
            '/plc/tx_pickup',
            10
        )

        self.plc_dropoff_pub = self.create_publisher(
            UInt8,
            '/plc/tx_dropoff',
            10
        )

        # Mission status topic'i transient-local
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.mission_status_sub = self.create_subscription(
            String,
            '/amr/mission_status_json',
            self.mission_status_callback,
            status_qos
        )

        self.active_task = None
        self.last_control = None

        self.mission_state = 0
        self.mission_started = False
        self.mission_paused = False

        self.command_queue = deque()

        # Komutları aralarında 300 ms bırakarak gönder
        self.command_timer = self.create_timer(
            0.3,
            self.process_command_queue
        )

        self.publish_plc_status(1)

        self.get_logger().info(
            'PLC mission adapter hazır.'
        )

    # =====================================================
    # PLC GÖREV PAKETİ
    # =====================================================

    def plc_mission_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warning(
                'PLC görev mesajı 3 byte olmalıdır.'
            )
            return

        pickup = int(msg.data[0])
        dropoff = int(msg.data[1])
        control = int(msg.data[2])

        if pickup not in (1, 2, 3):
            self.get_logger().warning(
                f'Geçersiz pickup: {pickup}'
            )
            return

        if dropoff not in (1, 2, 3):
            self.get_logger().warning(
                f'Geçersiz dropoff: {dropoff}'
            )
            return

        if control not in (1, 2):
            self.get_logger().warning(
                f'Geçersiz control: {control}'
            )
            return

        task = (pickup, dropoff)

        # ---------------------------------------------
        # Yeni görev
        # ---------------------------------------------
        if self.active_task is None:
            if self.mission_state != 0:
                self.get_logger().warning(
                    'MissionManager boş durumda değil. '
                    f'state={self.mission_state}'
                )
                return

            self.active_task = task
            self.last_control = control
            self.mission_started = False
            self.mission_paused = False

            self.publish_station_information(
                pickup,
                dropoff
            )

            self.enqueue_command(
                f'CREATE A{pickup} B{dropoff}'
            )
            self.enqueue_command('APPROVE')

            if control == 2:
                self.enqueue_command('START')
                self.publish_plc_status(2)
            else:
                self.publish_plc_status(5)

            self.get_logger().info(
                f'Yeni PLC görevi: '
                f'A{pickup} -> B{dropoff}, '
                f'control={control}'
            )
            return

        # ---------------------------------------------
        # Çalışan görevden farklı görev geldi
        # ---------------------------------------------
        if task != self.active_task:
            self.get_logger().warning(
                'Aktif görev bitmeden farklı görev geldi. '
                f'Aktif=A{self.active_task[0]}'
                f'->B{self.active_task[1]}, '
                f'Gelen=A{pickup}->B{dropoff}'
            )
            return

        # Aynı paket her saniye geliyorsa hiçbir şey yapma
        if control == self.last_control:
            return

        # ---------------------------------------------
        # PLC: BEKLE
        # ---------------------------------------------
        if control == 1:
            # START henüz gönderilmediyse kuyruktan çıkar
            self.remove_queued_command('START')

            if self.mission_started and not self.mission_paused:
                self.enqueue_command('PAUSE')

            self.publish_plc_status(5)

        # ---------------------------------------------
        # PLC: BAŞLA / DEVAM ET
        # ---------------------------------------------
        elif control == 2:
            if not self.mission_started:
                self.enqueue_command('START')

            elif self.mission_paused:
                self.enqueue_command('RESUME')

            self.publish_plc_status(2)

        self.last_control = control

    # =====================================================
    # MISSION MANAGER DURUMU
    # =====================================================

    def mission_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(
                f'Mission JSON okunamadı: {error}'
            )
            return

        self.mission_state = int(
            status.get('state', 0)
        )

        navigation_active = bool(
            status.get('navigationActive', False)
        )

        navigation_paused = bool(
            status.get('navigationPaused', False)
        )

        current_target = str(
            status.get('currentTargetName', '')
        )

        operation = str(
            status.get('operation', '')
        )

        status_text = str(
            status.get('statusText', '')
        )

        # MissionManager tamamen boş
        if self.mission_state == 0:
            if self.active_task is None:
                self.publish_plc_status(1)

        # Görev oluşturuldu
        elif self.mission_state == 1:
            self.publish_plc_status(2)

        # Görev onaylandı
        elif self.mission_state == 2:
            if self.last_control == 1:
                self.publish_plc_status(5)
            else:
                self.publish_plc_status(2)

        # Hata
        elif self.mission_state == 7:
            self.publish_plc_status(7)

        # Pause
        elif navigation_paused:
            self.mission_paused = True
            self.publish_plc_status(5)

        # Aktif navigasyon
        elif navigation_active:
            self.mission_paused = False

            if current_target.startswith('A'):
                # Yüksüz olarak alım noktasına gidiyor
                self.publish_plc_status(3)

            elif current_target.startswith('B'):
                # Yüklü olarak bırakma noktasına gidiyor
                self.publish_plc_status(4)

        self.get_logger().info(
            'Mission status | '
            f'state={self.mission_state}, '
            f'target={current_target}, '
            f'operation={operation}, '
            f'status={status_text}'
        )

    # =====================================================
    # KOMUT KUYRUĞU
    # =====================================================

    def enqueue_command(self, command):
        if command in self.command_queue:
            return

        self.command_queue.append(command)

    def remove_queued_command(self, command):
        self.command_queue = deque(
            item
            for item in self.command_queue
            if item != command
        )

    def process_command_queue(self):
        if not self.command_queue:
            return

        command = self.command_queue.popleft()

        msg = String()
        msg.data = command
        self.mission_command_pub.publish(msg)

        if command == 'START':
            self.mission_started = True
            self.mission_paused = False

        elif command == 'PAUSE':
            self.mission_paused = True

        elif command == 'RESUME':
            self.mission_paused = False

        elif command == 'CANCEL':
            self.mission_started = False
            self.mission_paused = False

        self.get_logger().info(
            f'Mission komutu gönderildi: {command}'
        )

    # =====================================================
    # PLC TX YARDIMCILARI
    # =====================================================

    def publish_plc_status(self, status):
        msg = UInt8()
        msg.data = int(status)
        self.plc_status_pub.publish(msg)

    def publish_station_information(
        self,
        pickup,
        dropoff
    ):
        pickup_msg = UInt8()
        pickup_msg.data = int(pickup)
        self.plc_pickup_pub.publish(pickup_msg)

        dropoff_msg = UInt8()
        dropoff_msg.data = int(dropoff)
        self.plc_dropoff_pub.publish(dropoff_msg)


def main(args=None):
    rclpy.init(args=args)

    node = PlcMissionAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
