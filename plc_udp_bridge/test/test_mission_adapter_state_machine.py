"""State-machine regression test without requiring a running ROS graph."""

import importlib
import json
import sys
import types


class _Message:
    def __init__(self):
        self.data = None


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, _name):
        self.parameters = {}
        self.publishers = {}

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def create_subscription(self, *_args):
        return object()

    def create_publisher(self, _type, topic, _qos):
        publisher = _Publisher()
        self.publishers[topic] = publisher
        return publisher

    def create_timer(self, *_args):
        return object()

    def get_logger(self):
        return _Logger()

    def destroy_node(self):
        pass


class _QoSProfile:
    def __init__(self, **_kwargs):
        pass


def _install_ros_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda **_kwargs: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda _node: None

    node_module = types.ModuleType('rclpy.node')
    node_module.Node = _Node
    qos_module = types.ModuleType('rclpy.qos')
    qos_module.QoSProfile = _QoSProfile
    qos_module.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    qos_module.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Bool = _Message
    std_msgs_msg.String = _Message
    std_msgs_msg.UInt8 = _Message
    std_msgs_msg.UInt8MultiArray = _Message

    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_module
    sys.modules['rclpy.qos'] = qos_module
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg


def _status_message(
        state, stage=0, pickup='', dropoff='',
        automation_waiting=False, waiting_door=''):
    message = _Message()
    message.data = json.dumps({
        'state': state,
        'stage': stage,
        'pickup': pickup,
        'dropoff': dropoff,
        'navigationActive': state == 3,
        'navigationPaused': state == 4,
        'currentTargetName': pickup if stage <= 3 else dropoff,
        'automationWaiting': automation_waiting,
        'waitingDoor': waiting_door,
    })
    return message


def _plc_message(pickup, dropoff, control):
    message = _Message()
    message.data = [pickup, dropoff, control]
    return message


def test_acknowledged_start_and_same_task_rearm():
    _install_ros_stubs()
    module = importlib.import_module(
        'plc_udp_bridge.plc_mission_adapter_node')
    adapter = module.PlcMissionAdapter()
    command_output = adapter.publishers['/amr/mission_command'].messages

    adapter.mission_status_callback(_status_message(0))
    adapter.plc_mission_callback(_plc_message(1, 2, 2))
    assert command_output == ['CREATE A1 B2']

    adapter.mission_status_callback(_status_message(1, pickup='A1', dropoff='B2'))
    assert command_output[-1] == 'APPROVE'

    adapter.mission_status_callback(_status_message(2, pickup='A1', dropoff='B2'))
    assert command_output[-1] == 'START'

    adapter.mission_status_callback(
        _status_message(3, stage=1, pickup='A1', dropoff='B2'))
    assert adapter.phase == 'ACTIVE'
    assert adapter.last_plc_status == 3

    adapter.mission_status_callback(
        _status_message(5, pickup='A1', dropoff='B2'))
    assert adapter.active_task is None
    assert adapter.last_plc_status == 6

    command_count = len(command_output)
    adapter.plc_mission_callback(_plc_message(1, 2, 2))
    assert len(command_output) == command_count

    adapter.plc_mission_callback(_plc_message(1, 2, 1))
    assert command_output[-1] == 'CREATE A1 B2'


def test_control_one_waits_after_approval():
    _install_ros_stubs()
    sys.modules.pop('plc_udp_bridge.plc_mission_adapter_node', None)
    module = importlib.import_module(
        'plc_udp_bridge.plc_mission_adapter_node')
    adapter = module.PlcMissionAdapter()
    command_output = adapter.publishers['/amr/mission_command'].messages

    adapter.mission_status_callback(_status_message(0))
    adapter.plc_mission_callback(_plc_message(2, 3, 1))
    adapter.mission_status_callback(_status_message(1, pickup='A2', dropoff='B3'))
    adapter.mission_status_callback(_status_message(2, pickup='A2', dropoff='B3'))

    assert command_output == ['CREATE A2 B3', 'APPROVE']
    assert adapter.phase == 'WAIT_START'
    assert adapter.last_plc_status == 5

    adapter.plc_mission_callback(_plc_message(2, 3, 2))
    assert command_output[-1] == 'START'
    assert adapter.phase == 'WAIT_START_ACK'


def test_returning_to_start_publishes_status_six():
    _install_ros_stubs()
    sys.modules.pop('plc_udp_bridge.plc_mission_adapter_node', None)
    module = importlib.import_module(
        'plc_udp_bridge.plc_mission_adapter_node')
    adapter = module.PlcMissionAdapter()

    adapter.mission_status_callback(_status_message(0))
    adapter.plc_mission_callback(_plc_message(1, 2, 2))
    adapter.mission_status_callback(_status_message(1, pickup='A1', dropoff='B2'))
    adapter.mission_status_callback(_status_message(2, pickup='A1', dropoff='B2'))
    adapter.mission_status_callback(
        _status_message(3, stage=7, pickup='A1', dropoff='B2'))

    assert adapter.last_plc_status == 6


def test_door_wait_requires_fresh_one_then_two_handshake():
    _install_ros_stubs()
    sys.modules.pop('plc_udp_bridge.plc_mission_adapter_node', None)
    module = importlib.import_module(
        'plc_udp_bridge.plc_mission_adapter_node')
    adapter = module.PlcMissionAdapter()

    adapter.mission_status_callback(_status_message(0))
    adapter.plc_mission_callback(_plc_message(1, 2, 2))
    adapter.mission_status_callback(_status_message(1, pickup='A1', dropoff='B2'))
    adapter.mission_status_callback(_status_message(2, pickup='A1', dropoff='B2'))
    adapter.mission_status_callback(
        _status_message(3, stage=4, pickup='A1', dropoff='B2'))

    adapter.mission_status_callback(_status_message(
        3, stage=4, pickup='A1', dropoff='B2',
        automation_waiting=True, waiting_door='K1'))
    release_output = adapter.publishers['/amr/automation_continue'].messages

    assert adapter.last_plc_status == 5
    assert adapter.phase == 'WAIT_DOOR_PLC_1'

    # A stale/early 2 cannot release the robot.
    adapter.plc_mission_callback(_plc_message(1, 2, 2))
    assert release_output == []

    adapter.plc_mission_callback(_plc_message(1, 2, 1))
    assert adapter.door_wait_armed is True
    assert release_output == []

    adapter.plc_mission_callback(_plc_message(1, 2, 2))
    assert release_output == [True]
    assert adapter.door_wait_armed is False
