"""Launch the detector, controller and optional GStreamer UDP stream."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory(
        'pulsar_color_line_following')
    default_config = os.path.join(
        package_share, 'config', 'color_line_following.yaml')

    show_image = LaunchConfiguration('show_image')
    gstreamer_enabled = LaunchConfiguration('gstreamer_enabled')
    gstreamer_host = LaunchConfiguration('gstreamer_host')
    gstreamer_port = LaunchConfiguration('gstreamer_port')
    gstreamer_bitrate_kbps = LaunchConfiguration(
        'gstreamer_bitrate_kbps')
    gstreamer_fps = LaunchConfiguration('gstreamer_fps')

    return LaunchDescription([
        DeclareLaunchArgument('show_image', default_value='false'),
        DeclareLaunchArgument('gstreamer_enabled', default_value='false'),
        DeclareLaunchArgument('gstreamer_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('gstreamer_port', default_value='5000'),
        DeclareLaunchArgument('gstreamer_bitrate_kbps', default_value='2000'),
        DeclareLaunchArgument('gstreamer_fps', default_value='30.0'),
        Node(
            package='pulsar_color_line_following',
            executable='color_line_detector',
            name='color_line_detector',
            output='screen',
            parameters=[
                default_config,
                {
                    'show_image': ParameterValue(
                        show_image, value_type=bool),
                    'gstreamer_enabled': ParameterValue(
                        gstreamer_enabled, value_type=bool),
                    'gstreamer_host': ParameterValue(
                        gstreamer_host, value_type=str),
                    'gstreamer_port': ParameterValue(
                        gstreamer_port, value_type=int),
                    'gstreamer_bitrate_kbps': ParameterValue(
                        gstreamer_bitrate_kbps, value_type=int),
                    'gstreamer_fps': ParameterValue(
                        gstreamer_fps, value_type=float),
                },
            ],
        ),
        Node(
            package='pulsar_color_line_following',
            executable='color_line_controller',
            name='color_line_controller',
            output='screen',
            parameters=[default_config],
        ),
    ])
