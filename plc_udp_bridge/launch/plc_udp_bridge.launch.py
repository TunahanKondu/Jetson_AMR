"""Start the PLC UDP bridge and, optionally, its mission adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Load installed network parameters and start the bridge."""
    parameters = os.path.join(
        get_package_share_directory('plc_udp_bridge'),
        'config', 'plc_udp_bridge.yaml',
    )
    local_ip = LaunchConfiguration('local_ip')
    plc_ip = LaunchConfiguration('plc_ip')
    start_adapter = LaunchConfiguration('start_adapter')

    return LaunchDescription([
        DeclareLaunchArgument(
            'local_ip',
            default_value='0.0.0.0',
            description='Local address used by the robot UDP socket',
        ),
        DeclareLaunchArgument(
            'plc_ip',
            default_value='192.168.100.100',
            description='PLC simulator/server IPv4 address',
        ),
        DeclareLaunchArgument(
            'start_adapter',
            default_value='true',
            description='Start PLC-to-mission state machine',
        ),
        Node(
            package='plc_udp_bridge',
            executable='plc_udp_bridge_node',
            name='plc_udp_bridge',
            output='screen',
            parameters=[parameters, {
                'local_ip': local_ip,
                'plc_ip': plc_ip,
            }],
        ),
        Node(
            package='plc_udp_bridge',
            executable='plc_mission_adapter_node',
            name='plc_mission_adapter',
            output='screen',
            parameters=[parameters],
            condition=IfCondition(start_adapter),
        ),
    ])
