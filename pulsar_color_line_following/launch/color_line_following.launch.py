"""Start the color detector and its controller together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('show_image', default_value='false',
                              description='Open a local OpenCV preview window'),
        DeclareLaunchArgument('linear_direction', default_value='-1.0',
                              description='Use -1 to command reverse motion'),
        DeclareLaunchArgument('steering_sign', default_value='-1.0',
                              description='Choose the angular correction sign'),
        Node(package='pulsar_color_line_following',
             executable='color_line_detector', output='screen',
             parameters=[{'show_image': ParameterValue(
                 LaunchConfiguration('show_image'), value_type=bool)}]),
        Node(package='pulsar_color_line_following',
             executable='color_line_controller', output='screen',
             parameters=[{
                 'linear_direction': ParameterValue(
                     LaunchConfiguration('linear_direction'), value_type=float),
                 'steering_sign': ParameterValue(
                     LaunchConfiguration('steering_sign'), value_type=float),
             }]),
    ])
