"""Start the color detector and its controller together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'show_image',
            default_value='false',
            description='Open a local OpenCV preview window'
        ),

        DeclareLaunchArgument(
            'linear_direction',
            default_value='-1.0',
            description='Use -1 to command reverse motion'
        ),

        DeclareLaunchArgument(
            'steering_sign',
            default_value='-1.0',
            description='Choose the angular correction sign'
        ),

        # -------------------------------------------------
        # GStreamer
        # -------------------------------------------------

        DeclareLaunchArgument(
            'gstreamer_enabled',
            default_value='true',
            description='Enable H264 RTP/UDP GStreamer streaming'
        ),

        DeclareLaunchArgument(
            'gstreamer_host',
            default_value='192.168.68.121',
            description='Destination GUI PC IP address'
        ),

        DeclareLaunchArgument(
            'gstreamer_port',
            default_value='5000',
            description='Destination UDP port'
        ),

        DeclareLaunchArgument(
            'gstreamer_bitrate',
            default_value='2000000',
            description='H264 encoder bitrate'
        ),

        DeclareLaunchArgument(
            'gstreamer_fps',
            default_value='30',
            description='GStreamer output FPS'
        ),

        # -------------------------------------------------
        # COLOR DETECTOR
        # -------------------------------------------------

        Node(
            package='pulsar_color_line_following',
            executable='color_line_detector',
            output='screen',
            parameters=[{
                'show_image': ParameterValue(
                    LaunchConfiguration('show_image'),
                    value_type=bool
                ),

                'gstreamer_enabled': ParameterValue(
                    LaunchConfiguration('gstreamer_enabled'),
                    value_type=bool
                ),

                'gstreamer_host': LaunchConfiguration(
                    'gstreamer_host'
                ),

                'gstreamer_port': ParameterValue(
                    LaunchConfiguration('gstreamer_port'),
                    value_type=int
                ),

                'gstreamer_bitrate': ParameterValue(
                    LaunchConfiguration('gstreamer_bitrate'),
                    value_type=int
                ),

                'gstreamer_fps': ParameterValue(
                    LaunchConfiguration('gstreamer_fps'),
                    value_type=int
                ),
            }]
        ),

        # -------------------------------------------------
        # COLOR CONTROLLER
        # -------------------------------------------------

        Node(
            package='pulsar_color_line_following',
            executable='color_line_controller',
            output='screen',
            parameters=[{
                'linear_direction': ParameterValue(
                    LaunchConfiguration('linear_direction'),
                    value_type=float
                ),

                'steering_sign': ParameterValue(
                    LaunchConfiguration('steering_sign'),
                    value_type=float
                ),
            }]
        ),
    ])

