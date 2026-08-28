#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port')
    base_port = LaunchConfiguration('base_port')

    args = [       
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('base_port', default_value='/dev/ttyUSB1'),
    ]

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    odom = Node(
        package='room3d',
        executable='odom_node',
        name='odom_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'port': base_port,
            'baud': 115200,

            # KALIBRASI ODOM
            'wheel_radius': 0.03,
            'wheel_separation': 0.135,
            'ticks_per_rev': 1152,
            'invert_left': False,
            'invert_right': False,

            'publish_tf': True,
            'publish_laser_tf': True,
            'publish_path': True,
            'laser_frame': 'laser',

            # JARAK LIDAR
            'pivot_x': 0.05,
            'pivot_z': 0.20,
            'arm_x': 0.0,
            'arm_z': 0.07,
            'tilt_sign': 1.0,
            'tilt_offset_deg': 0.0,
            'tilt_tf_rate': 100.0,
            'start_sweep': True,
        }],
    )

    return LaunchDescription(args + [lidar, odom])
