"""Start only the raw PLC UDP bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Load installed network parameters and start the bridge."""
    parameters = os.path.join(
        get_package_share_directory('plc_udp_bridge'),
        'config', 'plc_udp_bridge.yaml',
    )
    return LaunchDescription([
        Node(
            package='plc_udp_bridge',
            executable='plc_udp_bridge_node',
            name='plc_udp_bridge',
            output='screen',
            parameters=[parameters],
        ),
    ])
