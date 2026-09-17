import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================================================
    # DOSYA YOLLARI
    # =========================================================

    laser_filter_params = os.path.expanduser(
        "~/ros2_ws/src/bringup/config/laser_filter.yaml"
    )

    collision_monitor_params = (
        "/opt/ros/humble/share/nav2_collision_monitor/"
        "params/collision_monitor_params.yaml"
    )

    nav2_params = (
        "/opt/ros/humble/share/nav2_bringup/"
        "params/nav2_params.yaml"
    )

    map_file = (
        "/home/pulsar_robotic/ros2_ws/src/"
        "bringup/map/test_map.yaml"
    )

    nav2_bringup_dir = get_package_share_directory(
        "nav2_bringup"
    )


    # =========================================================
    # 1) LASER FILTER
    #
    # /scan
    #    ↓
    # laser_filters
    #    ↓
    # /scan_filtered
    # =========================================================

    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_filter",
        output="screen",

        parameters=[
            laser_filter_params
        ],

        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
    )


    # =========================================================
    # 2) COLLISION MONITOR
    #
    # /cmd_vel
    #    ↓
    # collision_monitor
    #    ↓
    # /cmd_vel_safe
    #
    # Sensör:
    # /scan_filtered
    # =========================================================

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",

        parameters=[
            collision_monitor_params
        ],
    )


    # =========================================================
    # COLLISION MONITOR LIFECYCLE MANAGER
    #
    # configure + activate işlemlerini otomatik yapar.
    # Artık:
    #
    # ros2 lifecycle set /collision_monitor configure
    # ros2 lifecycle set /collision_monitor activate
    #
    # yazmana gerek kalmaz.
    # =========================================================

    collision_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",

        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["collision_monitor"],
            }
        ],
    )


    # =========================================================
    # 3) NAV2
    # =========================================================

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                "launch",
                "bringup_launch.py",
            )
        ),

        launch_arguments={
            "map": map_file,
            "use_sim_time": "true",
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )


    # =========================================================
    # BAŞLATMA SIRASI
    #
    # 0 sn  : Laser filter
    # 1 sn  : Collision Monitor
    # 2 sn  : Collision lifecycle manager
    # 3 sn  : Nav2
    #
    # Böylece /scan_filtered önce oluşmuş olur.
    # =========================================================

    delayed_collision_monitor = TimerAction(
        period=1.0,
        actions=[
            collision_monitor
        ],
    )

    delayed_collision_lifecycle = TimerAction(
        period=2.0,
        actions=[
            collision_lifecycle_manager
        ],
    )

    delayed_nav2 = TimerAction(
        period=3.0,
        actions=[
            nav2
        ],
    )


    return LaunchDescription([
        laser_filter,
        delayed_collision_monitor,
        delayed_collision_lifecycle,
        delayed_nav2,
    ])
