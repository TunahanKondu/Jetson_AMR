#!/usr/bin/env python3
"""Translate the PLC's three-byte command into acknowledged mission commands."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String, UInt8, UInt8MultiArray


class PlcMissionAdapter(Node):
    """State-driven adapter between the PLC protocol and MissionManager."""

    NO_MISSION = 0
    PENDING_APPROVAL = 1
    APPROVED = 2
    RUNNING = 3
    PAUSED = 4
    COMPLETED = 5
    CANCELLED = 6
    MISSION_ERROR = 7

    TERMINAL_STATES = {COMPLETED, CANCELLED, MISSION_ERROR}
    CREATABLE_STATES = {NO_MISSION, COMPLETED, CANCELLED, MISSION_ERROR}

    def __init__(self):
        super().__init__('plc_mission_adapter')

        self.declare_parameter('command_timeout_sec', 2.0)
        self.declare_parameter('max_command_retries', 3)
        self.declare_parameter('emergency_stop_topic', '/amr/emergency_stop')

        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value)
        self.max_command_retries = int(
            self.get_parameter('max_command_retries').value)
        emergency_stop_topic = str(
            self.get_parameter('emergency_stop_topic').value)

        if self.command_timeout_sec <= 0.0:
            raise ValueError('command_timeout_sec must be positive')
        if self.max_command_retries < 0:
            raise ValueError('max_command_retries cannot be negative')

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.plc_mission_sub = self.create_subscription(
            UInt8MultiArray,
            '/plc/mission_command',
            self.plc_mission_callback,
            10,
        )
        self.mission_status_sub = self.create_subscription(
            String,
            '/amr/mission_status_json',
            self.mission_status_callback,
            transient_qos,
        )
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            emergency_stop_topic,
            self.emergency_stop_callback,
            10,
        )

        self.mission_command_pub = self.create_publisher(
            String, '/amr/mission_command', 10)
        self.plc_status_pub = self.create_publisher(
            UInt8, '/plc/tx_status', transient_qos)
        self.plc_pickup_pub = self.create_publisher(
            UInt8, '/plc/tx_pickup', transient_qos)
        self.plc_dropoff_pub = self.create_publisher(
            UInt8, '/plc/tx_dropoff', transient_qos)
        self.adapter_state_pub = self.create_publisher(
            String, '/plc/adapter_state', transient_qos)
        self.automation_continue_pub = self.create_publisher(
            Bool, '/amr/automation_continue', 10)

        self.have_mission_status = False
        self.mission_state = self.NO_MISSION
        self.mission_stage = 0
        self.navigation_active = False
        self.navigation_paused = False
        self.current_target = ''
        self.automation_waiting = False
        self.waiting_door = ''
        self.door_wait_armed = False

        self.active_task = None
        self.last_terminal_task = None
        self.desired_control = 1
        self.emergency_stop_active = False

        self.phase = 'IDLE'
        self.pending_command = None
        self.pending_expected_states = set()
        self.pending_sent_at = None
        self.pending_retries = 0

        self.last_plc_status = None
        self.last_pickup = None
        self.last_dropoff = None

        self.watchdog_timer = self.create_timer(0.1, self.command_watchdog)

        self.publish_plc_status(1, force=True)
        self.publish_adapter_state()
        self.get_logger().info('PLC mission adapter hazır; durum onayları bekleniyor.')

    # ------------------------------------------------------------------
    # PLC command input
    # ------------------------------------------------------------------
    def plc_mission_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warning('PLC görev mesajı tam olarak 3 byte olmalıdır.')
            return

        pickup, dropoff, control = (int(value) for value in msg.data)
        if pickup not in (1, 2, 3):
            self.get_logger().warning(f'Geçersiz pickup: {pickup}')
            return
        if dropoff not in (1, 2, 3):
            self.get_logger().warning(f'Geçersiz dropoff: {dropoff}')
            return
        if control not in (1, 2):
            self.get_logger().warning(f'Geçersiz control: {control}')
            return

        task = (pickup, dropoff)

        if self.emergency_stop_active:
            self.publish_plc_status(8)
            return

        if not self.have_mission_status:
            self.get_logger().warning(
                'Mission status henüz alınmadı; PLC görevi bekletiliyor.')
            return

        if self.active_task is None:
            if self.mission_state not in self.CREATABLE_STATES:
                self.get_logger().warning(
                    'MissionManager başka bir görevle meşgul; '
                    f'state={self.mission_state}')
                return

            # The PLC packet has no mission sequence number. After a terminal
            # state, an unchanged control=2 packet must not restart the same
            # task forever. A control=1 packet rearms that same task safely.
            if task == self.last_terminal_task and control == 2:
                self.publish_adapter_state('SAME_TASK_WAITING_FOR_REARM')
                return

            self.start_new_task(task, control)
            return

        if task != self.active_task:
            self.get_logger().warning(
                'Aktif görev bitmeden farklı görev reddedildi: '
                f'aktif=A{self.active_task[0]}->B{self.active_task[1]}, '
                f'gelen=A{pickup}->B{dropoff}')
            return

        # Door waiting is deliberately separate from mission PAUSE/RESUME.
        # The robot remains in RUNNING state while NavigationManager withholds
        # the next Nav2 goal. A fresh PLC 1 packet arms the door wait; only a
        # later PLC 2 packet releases it.
        if self.automation_waiting:
            self.desired_control = control
            if control == 1:
                if not self.door_wait_armed:
                    self.get_logger().info(
                        f'PLC kapı beklemesini onayladı: {self.waiting_door}')
                self.door_wait_armed = True
                self.phase = 'WAIT_DOOR_RELEASE'
            elif self.door_wait_armed:
                release = Bool()
                release.data = True
                self.automation_continue_pub.publish(release)
                self.door_wait_armed = False
                self.phase = 'WAIT_DOOR_CLEAR'
                self.get_logger().info(
                    f'PLC kapıyı açtı; devam bildirildi: {self.waiting_door}')
            else:
                self.get_logger().warning(
                    'Kapı beklemesi PLC 1 ile onaylanmadan PLC 2 geldi; '
                    'güvenlik için hareket başlatılmadı.')

            self.publish_status_from_mission()
            self.publish_adapter_state()
            return

        if control == self.desired_control:
            return

        self.desired_control = control
        self.get_logger().info(
            f'PLC kontrol değişti: {"BEKLE" if control == 1 else "BAŞLA/DEVAM"}')
        self.apply_desired_control()

    def start_new_task(self, task, control):
        self.active_task = task
        self.desired_control = control
        self.phase = 'WAIT_CREATE_ACK'
        self.publish_station_information(*task)
        self.send_tracked_command(
            f'CREATE A{task[0]} B{task[1]}',
            {self.PENDING_APPROVAL},
        )
        self.publish_plc_status(2)
        self.publish_adapter_state()
        self.get_logger().info(
            f'Yeni PLC görevi kabul edildi: A{task[0]} -> B{task[1]}, '
            f'control={control}')

    # ------------------------------------------------------------------
    # Mission status and state machine
    # ------------------------------------------------------------------
    def mission_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as error:
            self.get_logger().error(f'Mission JSON okunamadı: {error}')
            return

        previous_state = self.mission_state
        previous_automation_waiting = self.automation_waiting
        self.have_mission_status = True
        self.mission_state = int(status.get('state', self.NO_MISSION))
        self.mission_stage = int(status.get('stage', 0))
        self.navigation_active = bool(status.get('navigationActive', False))
        self.navigation_paused = bool(status.get('navigationPaused', False))
        self.current_target = str(status.get('currentTargetName', ''))
        self.automation_waiting = bool(status.get('automationWaiting', False))
        self.waiting_door = str(status.get('waitingDoor', ''))

        if self.automation_waiting and not previous_automation_waiting:
            self.door_wait_armed = False
            self.phase = 'WAIT_DOOR_PLC_1'
            self.get_logger().info(
                f'Kapı beklemesi başladı: {self.waiting_door}; PLC 1 bekleniyor.')
        elif previous_automation_waiting and not self.automation_waiting:
            self.door_wait_armed = False
            if self.mission_state == self.RUNNING:
                self.phase = 'ACTIVE'
            self.get_logger().info('Kapı beklemesi tamamlandı.')

        # Preserve replay protection if this node restarts while MissionManager
        # still exposes the previous terminal mission through transient-local.
        if (self.active_task is None
                and self.last_terminal_task is None
                and self.mission_state in self.TERMINAL_STATES):
            pickup_number = self.station_number(status.get('pickup', ''), 'A')
            dropoff_number = self.station_number(status.get('dropoff', ''), 'B')
            if pickup_number is not None and dropoff_number is not None:
                self.last_terminal_task = (pickup_number, dropoff_number)

        if self.pending_command and self.mission_state in self.pending_expected_states:
            acknowledged = self.pending_command
            self.clear_pending_command()
            self.get_logger().info(
                f'Mission komutu onaylandı: {acknowledged}, state={self.mission_state}')

        if self.active_task is not None:
            pickup_text = str(status.get('pickup', ''))
            dropoff_text = str(status.get('dropoff', ''))
            expected_pickup = f'A{self.active_task[0]}'
            expected_dropoff = f'B{self.active_task[1]}'
            if (self.mission_state in {self.PENDING_APPROVAL, self.APPROVED,
                                      self.RUNNING, self.PAUSED}
                    and (pickup_text != expected_pickup
                         or dropoff_text != expected_dropoff)):
                self.protocol_error(
                    'MissionManager görev bilgisi PLC göreviyle eşleşmiyor')
                return

        if self.active_task is not None:
            if self.mission_state == self.PENDING_APPROVAL:
                if self.phase == 'WAIT_CREATE_ACK':
                    self.phase = 'WAIT_APPROVE_ACK'
                    self.send_tracked_command('APPROVE', {self.APPROVED})

            elif self.mission_state == self.APPROVED:
                if self.phase in {'WAIT_APPROVE_ACK', 'WAIT_START'}:
                    if self.desired_control == 2:
                        self.phase = 'WAIT_START_ACK'
                        self.send_tracked_command('START', {self.RUNNING})
                    else:
                        self.phase = 'WAIT_START'

            elif self.mission_state == self.RUNNING:
                if self.phase in {'WAIT_START_ACK', 'WAIT_RESUME_ACK'}:
                    self.phase = 'ACTIVE'
                if (not self.automation_waiting
                        and self.desired_control == 1
                        and self.phase != 'WAIT_PAUSE_ACK'):
                    self.phase = 'WAIT_PAUSE_ACK'
                    self.send_tracked_command('PAUSE', {self.PAUSED})

            elif self.mission_state == self.PAUSED:
                self.phase = 'PAUSED'
                if self.desired_control == 2:
                    self.phase = 'WAIT_RESUME_ACK'
                    self.send_tracked_command('RESUME', {self.RUNNING})

            elif self.mission_state in self.TERMINAL_STATES:
                self.finish_active_task(self.mission_state)

        self.publish_status_from_mission()

        if previous_state != self.mission_state:
            self.get_logger().info(
                f'Mission state: {previous_state} -> {self.mission_state}, '
                f'stage={self.mission_stage}, phase={self.phase}')
            self.publish_adapter_state()

    def apply_desired_control(self):
        if self.active_task is None:
            return

        if self.automation_waiting:
            return

        if self.desired_control == 1:
            if self.phase == 'WAIT_START_ACK' and self.mission_state == self.APPROVED:
                self.clear_pending_command()
                self.phase = 'WAIT_START'
            elif self.mission_state == self.RUNNING:
                self.phase = 'WAIT_PAUSE_ACK'
                self.send_tracked_command('PAUSE', {self.PAUSED})
            elif self.mission_state in {self.APPROVED, self.PAUSED}:
                self.phase = 'WAIT_START' if self.mission_state == self.APPROVED else 'PAUSED'

        elif self.desired_control == 2:
            if self.mission_state == self.APPROVED:
                self.phase = 'WAIT_START_ACK'
                self.send_tracked_command('START', {self.RUNNING})
            elif self.mission_state == self.PAUSED:
                self.phase = 'WAIT_RESUME_ACK'
                self.send_tracked_command('RESUME', {self.RUNNING})

        self.publish_status_from_mission()
        self.publish_adapter_state()

    def finish_active_task(self, terminal_state):
        finished_task = self.active_task
        self.clear_pending_command()
        self.active_task = None
        self.phase = 'IDLE'
        self.last_terminal_task = finished_task

        if terminal_state == self.COMPLETED:
            self.publish_plc_status(6)
        elif terminal_state == self.CANCELLED:
            self.publish_plc_status(1)
        else:
            self.publish_plc_status(7)

        self.get_logger().info(
            f'PLC görevi sonlandı: A{finished_task[0]}->B{finished_task[1]}, '
            f'state={terminal_state}')
        self.publish_adapter_state()

    # ------------------------------------------------------------------
    # Reliable command delivery
    # ------------------------------------------------------------------
    def send_tracked_command(self, command, expected_states):
        self.pending_command = command
        self.pending_expected_states = set(expected_states)
        self.pending_retries = 0
        self.publish_mission_command(command)
        self.pending_sent_at = time.monotonic()

    def publish_mission_command(self, command):
        msg = String()
        msg.data = command
        self.mission_command_pub.publish(msg)
        self.get_logger().info(f'Mission komutu gönderildi: {command}')

    def clear_pending_command(self):
        self.pending_command = None
        self.pending_expected_states.clear()
        self.pending_sent_at = None
        self.pending_retries = 0

    def command_watchdog(self):
        if self.pending_command is None or self.pending_sent_at is None:
            return
        if time.monotonic() - self.pending_sent_at < self.command_timeout_sec:
            return

        if self.pending_retries >= self.max_command_retries:
            command = self.pending_command
            self.clear_pending_command()
            self.protocol_error(
                f'{command} komutu {self.max_command_retries} tekrar sonrası onaylanmadı')
            return

        self.pending_retries += 1
        self.get_logger().warning(
            f'Mission komutu tekrar gönderiliyor: {self.pending_command} '
            f'({self.pending_retries}/{self.max_command_retries})')
        self.publish_mission_command(self.pending_command)
        self.pending_sent_at = time.monotonic()

    def protocol_error(self, reason):
        failed_task = self.active_task
        self.clear_pending_command()
        self.phase = 'ERROR'
        if self.mission_state in {
                self.PENDING_APPROVAL, self.APPROVED, self.RUNNING, self.PAUSED}:
            self.publish_mission_command('CANCEL')
        self.active_task = None
        if failed_task is not None:
            self.last_terminal_task = failed_task
        self.publish_plc_status(7)
        self.publish_adapter_state(reason)
        self.get_logger().error(reason)

    # ------------------------------------------------------------------
    # PLC TX values and diagnostics
    # ------------------------------------------------------------------
    def publish_status_from_mission(self):
        if self.phase == 'ERROR':
            self.publish_plc_status(7)
        elif self.emergency_stop_active:
            self.publish_plc_status(8)
        elif self.automation_waiting:
            self.publish_plc_status(5)
        elif self.mission_state == self.NO_MISSION:
            self.publish_plc_status(1)
        elif self.mission_state == self.PENDING_APPROVAL:
            self.publish_plc_status(2)
        elif self.mission_state == self.APPROVED:
            self.publish_plc_status(5 if self.desired_control == 1 else 2)
        elif self.mission_state == self.RUNNING:
            if self.mission_stage in (1, 2, 3):
                self.publish_plc_status(3)
            elif self.mission_stage in (4, 5, 6):
                self.publish_plc_status(4)
            elif self.mission_stage == 7:
                # Load has been dropped and the empty robot is returning
                # to the canonical START station.
                self.publish_plc_status(6)
            else:
                self.publish_plc_status(2)
        elif self.mission_state == self.PAUSED:
            self.publish_plc_status(5)
        elif self.mission_state == self.COMPLETED:
            self.publish_plc_status(6)
        elif self.mission_state == self.CANCELLED:
            self.publish_plc_status(1)
        elif self.mission_state == self.MISSION_ERROR:
            self.publish_plc_status(7)

    def publish_plc_status(self, status, force=False):
        status = int(status)
        if self.last_plc_status == status and not force:
            return
        self.last_plc_status = status
        msg = UInt8()
        msg.data = status
        self.plc_status_pub.publish(msg)
        self.get_logger().info(f'PLC TX durum kodu: {status}')

    def publish_station_information(self, pickup, dropoff):
        if self.last_pickup != pickup:
            pickup_msg = UInt8()
            pickup_msg.data = int(pickup)
            self.plc_pickup_pub.publish(pickup_msg)
            self.last_pickup = pickup
        if self.last_dropoff != dropoff:
            dropoff_msg = UInt8()
            dropoff_msg.data = int(dropoff)
            self.plc_dropoff_pub.publish(dropoff_msg)
            self.last_dropoff = dropoff

    def publish_adapter_state(self, detail=''):
        msg = String()
        task_text = (
            'none' if self.active_task is None
            else f'A{self.active_task[0]}->B{self.active_task[1]}')
        msg.data = json.dumps({
            'phase': self.phase,
            'task': task_text,
            'control': self.desired_control,
            'missionState': self.mission_state,
            'missionStage': self.mission_stage,
            'automationWaiting': self.automation_waiting,
            'waitingDoor': self.waiting_door,
            'doorWaitArmed': self.door_wait_armed,
            'pendingCommand': self.pending_command or '',
            'detail': detail,
        }, separators=(',', ':'))
        self.adapter_state_pub.publish(msg)

    @staticmethod
    def station_number(value, prefix):
        text = str(value)
        if len(text) != 2 or text[0] != prefix or text[1] not in '123':
            return None
        return int(text[1])

    def emergency_stop_callback(self, msg):
        active = bool(msg.data)
        if active == self.emergency_stop_active:
            return
        self.emergency_stop_active = active
        if active:
            self.clear_pending_command()
            self.publish_mission_command('CANCEL')
            self.phase = 'EMERGENCY_STOP'
            self.publish_plc_status(8, force=True)
        else:
            self.phase = 'IDLE' if self.active_task is None else self.phase
            self.publish_status_from_mission()
        self.publish_adapter_state()


def main(args=None):
    rclpy.init(args=args)
    node = PlcMissionAdapter()
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
