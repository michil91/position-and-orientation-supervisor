#!/usr/bin/env python3
"""
IMU Simulator Node — publishes sensor_msgs/Imu on /sim/imu.

Models constant-velocity straight-line motion with configurable noise.
All parameters are read from sim_config.yaml via ROS2 parameter server.
Supports fault modes: none, bias, spike, dropout.
"""

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class ImuPublisher(Node):
    """Simulates a 6-DOF IMU with configurable fault injection."""

    def __init__(self):
        super().__init__('imu_publisher')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('accel_noise_stddev_mps2', 0.02)
        self.declare_parameter('gyro_noise_stddev_radps', 0.001)
        self.declare_parameter('linear_velocity_mps', 0.0)
        self.declare_parameter('turn_rate_radps', 0.0)

        self.declare_parameter('fault_mode', 'none')

        # bias
        self.declare_parameter('bias_axis', 'x')
        self.declare_parameter('bias_magnitude_mps2', 0.5)

        # spike
        self.declare_parameter('spike_time_s', 10.0)
        self.declare_parameter('spike_duration_s', 0.1)
        self.declare_parameter('spike_magnitude_mps2', 10.0)

        # dropout
        self.declare_parameter('dropout_start_s', 15.0)
        self.declare_parameter('dropout_duration_s', 3.0)

        # ── Read parameters ──────────────────────────────────────────────────
        self.rate_hz        = self.get_parameter('publish_rate_hz').value
        self.accel_noise    = self.get_parameter('accel_noise_stddev_mps2').value
        self.gyro_noise     = self.get_parameter('gyro_noise_stddev_radps').value
        self.velocity       = self.get_parameter('linear_velocity_mps').value
        self.turn_rate      = self.get_parameter('turn_rate_radps').value
        self.fault_mode     = self.get_parameter('fault_mode').value

        self.bias_axis      = self.get_parameter('bias_axis').value
        self.bias_mag       = self.get_parameter('bias_magnitude_mps2').value

        self.spike_time     = self.get_parameter('spike_time_s').value
        self.spike_duration = self.get_parameter('spike_duration_s').value
        self.spike_mag      = self.get_parameter('spike_magnitude_mps2').value

        self.dropout_start  = self.get_parameter('dropout_start_s').value
        self.dropout_dur    = self.get_parameter('dropout_duration_s').value

        # ── State ────────────────────────────────────────────────────────────
        self._elapsed_s = 0.0

        # Covariance matrices (diagonal, 9-element row-major — all must be float)
        a_var = self.accel_noise ** 2
        g_var = self.gyro_noise ** 2
        self._accel_cov = [a_var, 0.0, 0.0, 0.0, a_var, 0.0, 0.0, 0.0, a_var]
        self._gyro_cov  = [g_var, 0.0, 0.0, 0.0, g_var, 0.0, 0.0, 0.0, g_var]
        # Orientation unknown — fill with -1 to indicate not estimated
        self._orient_cov = [-1.0] + [0.0] * 8

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Imu, '/sim/imu', 10)

        period = 1.0 / self.rate_hz
        self._timer = self.create_timer(period, self._publish_cb)

        self.get_logger().info(
            f'ImuPublisher started | rate={self.rate_hz} Hz | '
            f'velocity={self.velocity} m/s | fault_mode={self.fault_mode}'
        )

    # ── helpers ──────────────────────────────────────────────────────────────

    def _gaussian(self, std: float) -> float:
        u1 = random.random()
        u2 = random.random()
        return math.sqrt(-2.0 * math.log(max(u1, 1e-10))) * math.cos(2.0 * math.pi * u2) * std

    # ── main callback ─────────────────────────────────────────────────────────

    def _publish_cb(self):
        dt = 1.0 / self.rate_hz
        self._elapsed_s += dt

        # ── Dropout ──────────────────────────────────────────────────────
        if self.fault_mode == 'dropout':
            t = self._elapsed_s
            if self.dropout_start <= t <= self.dropout_start + self.dropout_dur:
                self.get_logger().debug('IMU dropout active — suppressing message')
                return

        # ── Base accelerations ────────────────────────────────────────────
        # Straight-line constant-velocity → centripetal accel on y, no forward accel
        # Gravity is NOT included (body frame, gravity is internal to INS integration)
        accel_x = self._gaussian(self.accel_noise)
        accel_y = self.velocity * self.turn_rate + self._gaussian(self.accel_noise)
        accel_z = self._gaussian(self.accel_noise)

        gyro_x  = self._gaussian(self.gyro_noise)
        gyro_y  = self._gaussian(self.gyro_noise)
        gyro_z  = self.turn_rate + self._gaussian(self.gyro_noise)

        # ── Bias fault ────────────────────────────────────────────────────
        if self.fault_mode == 'bias':
            if self.bias_axis == 'x':
                accel_x += self.bias_mag
            elif self.bias_axis == 'y':
                accel_y += self.bias_mag
            elif self.bias_axis == 'z':
                accel_z += self.bias_mag

        # ── Spike fault ───────────────────────────────────────────────────
        if self.fault_mode == 'spike':
            t = self._elapsed_s
            if self.spike_time <= t <= self.spike_time + self.spike_duration:
                accel_x += self.spike_mag

        # ── Build message ─────────────────────────────────────────────────
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu'

        msg.orientation_covariance = self._orient_cov

        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z
        msg.angular_velocity_covariance = self._gyro_cov

        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z
        msg.linear_acceleration_covariance = self._accel_cov

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
