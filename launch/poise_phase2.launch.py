"""
POISE Phase 2 Launch File.

Starts all four nodes, loading every parameter from config/sim_config.yaml.
Topics use global names (leading slash) so nodes find each other regardless
of namespace.

New in Phase 2:
  - /poise/reset service available on integrity_aggregator
  - QoS profiles applied from poise.qos (no inline QoS decisions)
  - GNSS dropout detection active (gnss_dropout_timeout_s)
  - Revalidation-based auto-recovery for environmental faults
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('poise'),
        'config',
        'sim_config.yaml'
    )

    return LaunchDescription([

        Node(
            package='poise',
            executable='gnss_publisher',
            name='gnss_publisher',
            parameters=[config],
            output='screen',
        ),

        Node(
            package='poise',
            executable='imu_publisher',
            name='imu_publisher',
            parameters=[config],
            output='screen',
        ),

        Node(
            package='poise',
            executable='gnss_imu_checker',
            name='gnss_imu_checker',
            parameters=[config],
            output='screen',
        ),

        Node(
            package='poise',
            executable='integrity_aggregator',
            name='integrity_aggregator',
            parameters=[config],
            output='screen',
        ),

    ])
