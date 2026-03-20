#!/usr/bin/env python3
"""
Calibration Envelope Validator — Phase 3.

Subscribes to /sim/imu (sensor_msgs/Imu) and /sim/gnss (sensor_msgs/NavSatFix).
Validates that sensor readings lie within their physical calibration envelopes.
Publishes poise/IntegrityStatus on /poise/integrity_status using INTEGRITY_QOS.
All thresholds are loaded from sim_config.yaml at startup — no hardcoded values.

Fault codes
-----------
IMU_ACCEL_OUT_OF_ENVELOPE    Non-recoverable  Linear acceleration magnitude exceeds calibrated range
IMU_GYRO_OUT_OF_ENVELOPE     Non-recoverable  Angular velocity magnitude exceeds calibrated range
IMU_GRAVITY_OUT_OF_ENVELOPE  Non-recoverable  Z-axis acceleration outside expected gravity range
GNSS_OUT_OF_GEOFENCE         Recoverable      Position outside expected operating area

IMU faults are non-recoverable because they indicate sensor failure — a physically
impossible reading cannot be explained by environmental conditions.

GNSS_OUT_OF_GEOFENCE is recoverable because it is positional/environmental — the
vehicle may re-enter the geofence as it moves.

Publishing cadence
------------------
Individual check faults: published on every IMU/GNSS callback while active
    (rate-limited to one publish per status change to avoid bus flooding).
Heartbeat: STATUS_OK published at 1 Hz on check_name='calibration_validator'
    when all checks pass — confirms the node is alive and monitoring.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

from poise.msg import IntegrityStatus
from poise.qos import SENSOR_QOS, INTEGRITY_QOS


class CalibrationValidator(Node):
    """Validates IMU and GNSS readings against their physical calibration envelopes."""

    def __init__(self):
        super().__init__('calibration_validator')

        # ── Parameters ──────────────────────────────────────────────────────
        # IMU envelope
        self.declare_parameter('imu_max_linear_accel',    49.0)
        self.declare_parameter('imu_max_angular_velocity', 10.0)
        self.declare_parameter('imu_min_accel_z',         -15.0)
        self.declare_parameter('imu_max_accel_z',          -5.0)

        # GNSS envelope
        self.declare_parameter('gnss_max_covariance',      25.0)
        self.declare_parameter('gnss_geofence_lat_min',    35.0)
        self.declare_parameter('gnss_geofence_lat_max',    36.5)
        self.declare_parameter('gnss_geofence_lon_min',   139.0)
        self.declare_parameter('gnss_geofence_lon_max',   140.5)

        self._imu_max_accel   = self.get_parameter('imu_max_linear_accel').value
        self._imu_max_gyro    = self.get_parameter('imu_max_angular_velocity').value
        self._imu_min_accel_z = self.get_parameter('imu_min_accel_z').value
        self._imu_max_accel_z = self.get_parameter('imu_max_accel_z').value

        self._gnss_max_cov  = self.get_parameter('gnss_max_covariance').value
        self._geo_lat_min   = self.get_parameter('gnss_geofence_lat_min').value
        self._geo_lat_max   = self.get_parameter('gnss_geofence_lat_max').value
        self._geo_lon_min   = self.get_parameter('gnss_geofence_lon_min').value
        self._geo_lon_max   = self.get_parameter('gnss_geofence_lon_max').value

        # ── Per-check last-published status (avoids re-publishing unchanged state) ──
        self._imu_accel_active   = False
        self._imu_gyro_active    = False
        self._imu_gravity_active = False
        self._gnss_geofence_active = False

        # ── Publisher / Subscribers ──────────────────────────────────────────
        self._status_pub = self.create_publisher(
            IntegrityStatus, '/poise/integrity_status', INTEGRITY_QOS
        )

        self.create_subscription(Imu,       '/sim/imu',  self._imu_cb,  SENSOR_QOS)
        self.create_subscription(NavSatFix, '/sim/gnss', self._gnss_cb, SENSOR_QOS)

        # 1 Hz heartbeat — publish STATUS_OK when all checks pass
        self.create_timer(1.0, self._heartbeat_cb)

        self.get_logger().info(
            f'CalibrationValidator started | '
            f'imu_max_accel={self._imu_max_accel} m/s² | '
            f'imu_max_gyro={self._imu_max_gyro} rad/s | '
            f'imu_z_range=[{self._imu_min_accel_z}, {self._imu_max_accel_z}] m/s² | '
            f'geofence=[{self._geo_lat_min},{self._geo_lat_max}]°lat '
            f'[{self._geo_lon_min},{self._geo_lon_max}]°lon'
        )

    # ── IMU callback (100 Hz) ─────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        stamp = msg.header.stamp

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # ── Check 1: linear acceleration magnitude ────────────────────────
        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
        if accel_mag > self._imu_max_accel:
            if not self._imu_accel_active:
                self._imu_accel_active = True
                self._publish_status(
                    check_name='imu_accel_envelope',
                    status=IntegrityStatus.STATUS_CRITICAL,
                    recoverable=False,
                    fault_code='IMU_ACCEL_OUT_OF_ENVELOPE',
                    description=(
                        f'Acceleration magnitude {accel_mag:.2f} m/s² exceeds '
                        f'calibrated limit {self._imu_max_accel:.2f} m/s²'
                    ),
                    measured=accel_mag, threshold=self._imu_max_accel,
                    units='m/s²', stamp=stamp,
                )
        elif self._imu_accel_active:
            self._imu_accel_active = False
            self._publish_status(
                check_name='imu_accel_envelope',
                status=IntegrityStatus.STATUS_OK,
                recoverable=False,
                fault_code='',
                description='IMU linear acceleration within calibrated range',
                measured=accel_mag, threshold=self._imu_max_accel,
                units='m/s²', stamp=stamp,
            )

        # ── Check 2: angular velocity magnitude ───────────────────────────
        gyro_mag = math.sqrt(wx**2 + wy**2 + wz**2)
        if gyro_mag > self._imu_max_gyro:
            if not self._imu_gyro_active:
                self._imu_gyro_active = True
                self._publish_status(
                    check_name='imu_gyro_envelope',
                    status=IntegrityStatus.STATUS_CRITICAL,
                    recoverable=False,
                    fault_code='IMU_GYRO_OUT_OF_ENVELOPE',
                    description=(
                        f'Angular velocity magnitude {gyro_mag:.4f} rad/s exceeds '
                        f'calibrated limit {self._imu_max_gyro:.2f} rad/s'
                    ),
                    measured=gyro_mag, threshold=self._imu_max_gyro,
                    units='rad/s', stamp=stamp,
                )
        elif self._imu_gyro_active:
            self._imu_gyro_active = False
            self._publish_status(
                check_name='imu_gyro_envelope',
                status=IntegrityStatus.STATUS_OK,
                recoverable=False,
                fault_code='',
                description='IMU angular velocity within calibrated range',
                measured=gyro_mag, threshold=self._imu_max_gyro,
                units='rad/s', stamp=stamp,
            )

        # ── Check 3: Z-axis gravity range ─────────────────────────────────
        if az < self._imu_min_accel_z or az > self._imu_max_accel_z:
            if not self._imu_gravity_active:
                self._imu_gravity_active = True
                limit = (self._imu_min_accel_z if az < self._imu_min_accel_z
                         else self._imu_max_accel_z)
                self._publish_status(
                    check_name='imu_gravity_envelope',
                    status=IntegrityStatus.STATUS_CRITICAL,
                    recoverable=False,
                    fault_code='IMU_GRAVITY_OUT_OF_ENVELOPE',
                    description=(
                        f'Z-axis acceleration {az:.3f} m/s² outside expected gravity '
                        f'range [{self._imu_min_accel_z:.1f}, '
                        f'{self._imu_max_accel_z:.1f}] m/s²'
                    ),
                    measured=az, threshold=limit,
                    units='m/s²', stamp=stamp,
                )
        elif self._imu_gravity_active:
            self._imu_gravity_active = False
            self._publish_status(
                check_name='imu_gravity_envelope',
                status=IntegrityStatus.STATUS_OK,
                recoverable=False,
                fault_code='',
                description='IMU Z-axis acceleration within expected gravity range',
                measured=az, threshold=self._imu_min_accel_z,
                units='m/s²', stamp=stamp,
            )

    # ── GNSS callback (10 Hz) ─────────────────────────────────────────────────

    def _gnss_cb(self, msg: NavSatFix):
        stamp = msg.header.stamp
        lat   = msg.latitude
        lon   = msg.longitude

        # ── Check 4: geofence ─────────────────────────────────────────────
        outside = (
            lat < self._geo_lat_min or lat > self._geo_lat_max or
            lon < self._geo_lon_min or lon > self._geo_lon_max
        )
        if outside:
            if not self._gnss_geofence_active:
                self._gnss_geofence_active = True
            self._publish_status(
                check_name='gnss_geofence',
                status=IntegrityStatus.STATUS_WARN,
                recoverable=True,
                fault_code='GNSS_OUT_OF_GEOFENCE',
                description=(
                    f'Position ({lat:.4f}°, {lon:.4f}°) outside operating geofence '
                    f'([{self._geo_lat_min},{self._geo_lat_max}]°lat, '
                    f'[{self._geo_lon_min},{self._geo_lon_max}]°lon)'
                ),
                measured=lat, threshold=self._geo_lat_min,
                units='degrees', stamp=stamp,
            )
        elif self._gnss_geofence_active:
            self._gnss_geofence_active = False
            self._publish_status(
                check_name='gnss_geofence',
                status=IntegrityStatus.STATUS_OK,
                recoverable=True,
                fault_code='',
                description='GNSS position within operating geofence',
                measured=lat, threshold=self._geo_lat_min,
                units='degrees', stamp=stamp,
            )

    # ── Heartbeat (1 Hz) ──────────────────────────────────────────────────────

    def _heartbeat_cb(self):
        """Publish STATUS_OK at 1 Hz when all checks pass — confirms node is alive."""
        any_active = (
            self._imu_accel_active   or
            self._imu_gyro_active    or
            self._imu_gravity_active or
            self._gnss_geofence_active
        )
        if not any_active:
            stamp = self.get_clock().now().to_msg()
            self._publish_status(
                check_name='calibration_validator',
                status=IntegrityStatus.STATUS_OK,
                recoverable=True,
                fault_code='',
                description='All calibration envelope checks nominal',
                measured=0.0, threshold=0.0,
                units='', stamp=stamp,
            )

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self, *, check_name, status, recoverable, fault_code,
                        description, measured, threshold, units, stamp):
        msg = IntegrityStatus()
        msg.header.stamp        = stamp
        msg.header.frame_id     = 'calibration_validator'
        msg.check_name          = check_name
        msg.status              = status
        msg.recoverable         = recoverable
        msg.fault_code          = fault_code
        msg.description         = description
        msg.measured_value      = float(measured)
        msg.threshold_exceeded  = float(threshold)
        msg.units               = units
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
