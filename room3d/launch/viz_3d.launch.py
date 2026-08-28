#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('room3d')
    urdf_path = os.path.join(share, 'urdf', 'robot3d.urdf')
    rviz_path = os.path.join(share, 'rviz', 'room3d.rviz')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    args = [
        DeclareLaunchArgument('gate_on_motion', default_value='true',
                             description='true = point cloud TIDAK diunggah saat robot bergerak'),
        DeclareLaunchArgument('voxel_size', default_value='0.02'),
        DeclareLaunchArgument('range_max', default_value='4.0'),
        DeclareLaunchArgument('settle_time', default_value='0.6'),
        DeclareLaunchArgument('max_points', default_value='250000'),
        DeclareLaunchArgument('save_path', default_value='~/room3d_scan.pcd'),
    ]

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    mapper = Node(
        package='room3d',
        executable='cloud_mapper',
        name='cloud_mapper',
        output='screen',
        parameters=[{
            'fixed_frame': 'odom',
            'publish_period': 1.5,
            'range_min': 0.15,
            'range_max': LaunchConfiguration('range_max'),
            'min_z': -0.30,
            'max_z': 2.50,
            'voxel_size': LaunchConfiguration('voxel_size'),
            'max_points': LaunchConfiguration('max_points'),
            'deskew_chunks': 1,

            # ---- inti permintaan: gating gerak ----
            'gate_on_motion': LaunchConfiguration('gate_on_motion'),
            'lin_thresh': 0.010,
            'ang_thresh': 0.030,
            'settle_time': LaunchConfiguration('settle_time'),
            'cmd_hold_time': 0.4,
            'odom_timeout': 1.0,
            'require_odom': True,
            'publish_while_moving': False,

            'save_path': LaunchConfiguration('save_path'),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )

    return LaunchDescription(args + [rsp, mapper, rviz])