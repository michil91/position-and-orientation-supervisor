"""
POISE Phase 1 Launch File.

Starts all four nodes, loading every parameter from config/sim_config.yaml.
Each node is given its own namespace-scoped parameter block as defined in the
YAML file.  Topics use global names (leading slash) so nodes find each other
regardless of namespace.
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
