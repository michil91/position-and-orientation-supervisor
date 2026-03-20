"""
POISE Combined Launch File — Phase 4 + wheel odometry cross-check.

Starts all ten POISE nodes plus RViz2 pre-configured with poise.rviz.
Parameters are loaded from config/sim_config.yaml (base) overlaid with
the selected scenario file from config/scenarios/<scenario>.yaml.

Usage
-----
  ros2 launch poise poise.launch.py                      # nominal (no faults)
  ros2 launch poise poise.launch.py scenario:=gnss_drift
  ros2 launch poise poise.launch.py scenario:=gnss_jump
  ros2 launch poise poise.launch.py scenario:=gnss_dropout
  ros2 launch poise poise.launch.py scenario:=imu_bias
  ros2 launch poise poise.launch.py scenario:=imu_spike
  ros2 launch poise poise.launch.py scenario:=imu_extrinsic
  ros2 launch poise poise.launch.py scenario:=odom_slip
  ros2 launch poise poise.launch.py scenario:=odom_dropout

Topics
------
Sensor simulation:   /sim/gnss, /sim/imu, /sim/vehicle_state, /sim/odometry
Integrity outputs:   /poise/integrity_status, /poise/system_integrity
Visualization:       /poise/viz/* (consumed by RViz2)
DR position:         /poise/dr_position (gnss_imu_checker → status_visualizer)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_VALID_SCENARIOS = [
    'nominal',
    'gnss_drift',
    'gnss_jump',
    'gnss_dropout',
    'gnss_slow_drift',
    'imu_extrinsic_warn_only',
    'imu_bias',
    'imu_spike',
    'imu_extrinsic',
    'odom_slip',
    'odom_dropout',
]


def _launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration('scenario').perform(context)

    if scenario not in _VALID_SCENARIOS:
        valid = ', '.join(_VALID_SCENARIOS)
        raise RuntimeError(
            f'\n[poise] ERROR: Unknown scenario "{scenario}"\n'
            f'[poise] Valid scenarios: {valid}\n'
        )

    print(f'\n[poise] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
    print(f'[poise] Active scenario: {scenario}')
    print(f'[poise] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n')

    pkg_share   = get_package_share_directory('poise')
    base_cfg    = os.path.join(pkg_share, 'config', 'sim_config.yaml')
    scenario_cfg = os.path.join(pkg_share, 'config', 'scenarios', f'{scenario}.yaml')
    rviz_cfg    = os.path.join(pkg_share, 'config', 'poise.rviz')

    # Both files passed to each node; scenario overrides base for any shared key.
    params = [base_cfg, scenario_cfg]

    return [

        # ── Simulation nodes ─────────────────────────────────────────────────
        Node(
            package='poise',
            executable='gnss_publisher',
            name='gnss_publisher',
            parameters=params,
            output='screen',
        ),

        Node(
            package='poise',
            executable='imu_publisher',
            name='imu_publisher',
            parameters=params,
            output='screen',
        ),

        Node(
            package='poise',
            executable='vehicle_state_publisher',
            name='vehicle_state_publisher',
            parameters=params,
            output='screen',
        ),

        Node(
            package='poise',
            executable='odometry_publisher',
            name='odometry_publisher',
            parameters=params,
            output='screen',
        ),

        # ── Integrity checker nodes ───────────────────────────────────────────
        Node(
            package='poise',
            executable='gnss_imu_checker',
            name='gnss_imu_checker',
            parameters=params,
            output='screen',
        ),

        Node(
            package='poise',
            executable='calibration_validator',
            name='calibration_validator',
            parameters=params,
            output='screen',
        ),

        Node(
            package='poise',
            executable='extrinsic_validator',
            name='extrinsic_validator',
            parameters=params,
            output='screen',
        ),

        Node(
            package='poise',
            executable='odometry_checker',
            name='odometry_checker',
            parameters=params,
            output='screen',
        ),

        # ── Aggregator ───────────────────────────────────────────────────────
        Node(
            package='poise',
            executable='integrity_aggregator',
            name='integrity_aggregator',
            parameters=params,
            output='screen',
        ),

        # ── Visualization ────────────────────────────────────────────────────
        Node(
            package='poise',
            executable='status_visualizer',
            name='status_visualizer',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),

    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='nominal',
            description=(
                f'Fault scenario to run. Valid values: {", ".join(_VALID_SCENARIOS)}'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
