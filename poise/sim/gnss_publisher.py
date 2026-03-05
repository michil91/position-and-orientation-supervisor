#!/usr/bin/env python3
"""
GNSS Simulator Node — publishes sensor_msgs/NavSatFix on /sim/gnss.

All parameters are read from sim_config.yaml via ROS2 parameter server.
Supports fault modes: none, drift, jump, dropout, covariance_inflation.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from builtin_interfaces.msg import Time


class GnssPublisher(Node):
    """Simulates a GNSS receiver with configurable fault injection."""

    def __init__(self):
        super().__init__('gnss_publisher')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('initial_latitude', 35.6812)
        self.declare_parameter('initial_longitude', 139.7671)
        self.declare_parameter('initial_altitude', 40.0)
        self.declare_parameter('position_noise_stddev_m', 0.5)
        self.declare_parameter('fix_type', 3)
        self.declare_parameter('num_satellites', 12)
        self.declare_parameter('hdop', 0.9)
        self.declare_parameter('position_covariance_m2', 0.25)

        self.declare_parameter('fault_mode', 'none')

        # drift
        self.declare_parameter('drift_rate_m_per_s', 0.05)
        self.declare_parameter('drift_direction_deg', 0.0)

        # jump
        self.declare_parameter('jump_time_s', 20.0)
        self.declare_parameter('jump_north_m', 5.0)
        self.declare_parameter('jump_east_m', 0.0)

        # dropout
        self.declare_parameter('dropout_start_s', 15.0)
        self.declare_parameter('dropout_duration_s', 5.0)

        # covariance_inflation
        self.declare_parameter('inflated_covariance_m2', 50.0)
        self.declare_parameter('inflation_start_s', 10.0)
        self.declare_parameter('inflation_duration_s', 10.0)

        # ── Read parameters ──────────────────────────────────────────────────
        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.lat0 = self.get_parameter('initial_latitude').value
        self.lon0 = self.get_parameter('initial_longitude').value
        self.alt0 = self.get_parameter('initial_altitude').value
        self.noise_std = self.get_parameter('position_noise_stddev_m').value
        self.fix_type = self.get_parameter('fix_type').value
        self.num_sats = self.get_parameter('num_satellites').value
        self.hdop = self.get_parameter('hdop').value
        self.cov_m2 = self.get_parameter('position_covariance_m2').value
        self.fault_mode = self.get_parameter('fault_mode').value

        self.drift_rate = self.get_parameter('drift_rate_m_per_s').value
        self.drift_dir_deg = self.get_parameter('drift_direction_deg').value

        self.jump_time = self.get_parameter('jump_time_s').value
        self.jump_north = self.get_parameter('jump_north_m').value
        self.jump_east = self.get_parameter('jump_east_m').value

        self.dropout_start = self.get_parameter('dropout_start_s').value
        self.dropout_duration = self.get_parameter('dropout_duration_s').value

        self.inflated_cov = self.get_parameter('inflated_covariance_m2').value
        self.inflation_start = self.get_parameter('inflation_start_s').value
        self.inflation_duration = self.get_parameter('inflation_duration_s').value

        # ── State ────────────────────────────────────────────────────────────
        self._elapsed_s = 0.0
        self._jump_applied = False
        self._lat_offset_deg = 0.0  # accumulated drift in degrees
        self._lon_offset_deg = 0.0

        # Earth radius for degree conversion
        self._R = 6_378_137.0

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(NavSatFix, '/sim/gnss', 10)

        period = 1.0 / self.rate_hz
        self._timer = self.create_timer(period, self._publish_cb)

        self.get_logger().info(
            f'GnssPublisher started | rate={self.rate_hz} Hz | '
            f'fault_mode={self.fault_mode}'
        )

    # ── helpers ──────────────────────────────────────────────────────────────

    def _metres_to_lat_deg(self, metres: float) -> float:
        return metres / self._R * (180.0 / math.pi)

    def _metres_to_lon_deg(self, metres: float, lat_deg: float) -> float:
        return metres / (self._R * math.cos(math.radians(lat_deg))) * (180.0 / math.pi)

    def _gaussian_noise(self) -> float:
        """Return a single Gaussian sample (Box-Muller)."""
        import random
        import math
        u1 = random.random()
        u2 = random.random()
        return math.sqrt(-2.0 * math.log(max(u1, 1e-10))) * math.cos(2.0 * math.pi * u2)

    # ── main callback ─────────────────────────────────────────────────────────

    def _publish_cb(self):
        dt = 1.0 / self.rate_hz
        self._elapsed_s += dt

        # ── Dropout: suppress publication ─────────────────────────────────
        if self.fault_mode == 'dropout':
            t = self._elapsed_s
            if self.dropout_start <= t <= self.dropout_start + self.dropout_duration:
                self.get_logger().debug('GNSS dropout active — suppressing message')
                return

        # ── Drift accumulation ────────────────────────────────────────────
        if self.fault_mode == 'drift':
            drift_north = self.drift_rate * dt * math.cos(math.radians(self.drift_dir_deg))
            drift_east  = self.drift_rate * dt * math.sin(math.radians(self.drift_dir_deg))
            self._lat_offset_deg += self._metres_to_lat_deg(drift_north)
            self._lon_offset_deg += self._metres_to_lon_deg(drift_east, self.lat0)

        # ── Jump: one-shot displacement ───────────────────────────────────
        if self.fault_mode == 'jump' and not self._jump_applied:
            if self._elapsed_s >= self.jump_time:
                self._lat_offset_deg += self._metres_to_lat_deg(self.jump_north)
                self._lon_offset_deg += self._metres_to_lon_deg(self.jump_east, self.lat0)
                self._jump_applied = True
                self.get_logger().warn(
                    f'GNSS jump applied at t={self._elapsed_s:.1f}s | '
                    f'N={self.jump_north} m, E={self.jump_east} m'
                )

        # ── Compose position with noise ───────────────────────────────────
        noise_n = self._gaussian_noise() * self.noise_std
        noise_e = self._gaussian_noise() * self.noise_std

        lat = (self.lat0
               + self._lat_offset_deg
               + self._metres_to_lat_deg(noise_n))
        lon = (self.lon0
               + self._lon_offset_deg
               + self._metres_to_lon_deg(noise_e, self.lat0))
        alt = self.alt0

        # ── Covariance ────────────────────────────────────────────────────
        active_cov = self.cov_m2
        if self.fault_mode == 'covariance_inflation':
            t = self._elapsed_s
            if self.inflation_start <= t <= self.inflation_start + self.inflation_duration:
                active_cov = self.inflated_cov

        cov_diag = [active_cov, 0.0, 0.0,
                    0.0, active_cov, 0.0,
                    0.0, 0.0, active_cov * 4.0]

        # ── Build message ─────────────────────────────────────────────────
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss'

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        msg.position_covariance = cov_diag
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GnssPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
