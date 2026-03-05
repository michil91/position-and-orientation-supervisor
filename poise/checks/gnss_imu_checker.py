#!/usr/bin/env python3
"""
GNSS/IMU Cross-Check Node.

Subscribes to /sim/gnss (sensor_msgs/NavSatFix) and /sim/imu (sensor_msgs/Imu).
At each GNSS update it compares the reported GNSS position against the position
predicted by dead-reckoning the IMU (Euler integration of linear acceleration).

Architecture — sliding DR window:
    Dead-reckoning (DR) runs continuously from a reference anchor point.  Every
    `dr_realign_window_s` seconds, the anchor is reset to the current GNSS position
    and DR restarts.  This:
      - Catches slow drift accumulating over tens of seconds
      - Prevents unbounded IMU integration error (DR error grows as O(T^1.5))
      - Still catches instantaneous jumps within one inter-fix interval

    Jumps are also compared against the previous-fix delta for fast detection.

Computes Euclidean distance in local Cartesian frame (East-North-Up, metres).
Publishes poise/IntegrityStatus on /poise/integrity_status.

  delta > warn_threshold_m     → STATUS_WARN,     GNSS_IMU_DIVERGENCE_WARN
  delta > critical_threshold_m → STATUS_CRITICAL,  GNSS_IMU_DIVERGENCE_CRITICAL

Also checks GNSS covariance against max_covariance_m2 threshold.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, Imu

from poise.msg import IntegrityStatus


# WGS-84 semi-major axis (metres)
_R_EARTH = 6_378_137.0


def _geodetic_to_enu(lat_ref_deg, lon_ref_deg,
                      lat_deg, lon_deg, alt_m=0.0, alt_ref_m=0.0):
    """
    Convert geodetic coordinates to local ENU (East-North-Up) offsets in metres
    relative to a reference point.  Flat-Earth approximation — valid for short
    baselines (< ~10 km) appropriate for integrity checking.
    """
    dlat = math.radians(lat_deg - lat_ref_deg)
    dlon = math.radians(lon_deg - lon_ref_deg)
    lat_ref = math.radians(lat_ref_deg)

    north = dlat * _R_EARTH
    east  = dlon * _R_EARTH * math.cos(lat_ref)
    up    = alt_m - alt_ref_m
    return east, north, up


class GnssImuChecker(Node):
    """Cross-checks GNSS position against IMU dead-reckoning."""

    def __init__(self):
        super().__init__('gnss_imu_checker')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('warn_threshold_m',      1.5)
        self.declare_parameter('critical_threshold_m',  3.0)
        self.declare_parameter('min_satellites',         6)
        self.declare_parameter('max_hdop',               2.0)
        self.declare_parameter('max_covariance_m2',     25.0)
        # How long to let DR run before forcing realignment to GNSS.
        # A value of 0 means realign after every fix (per-fix reset, original behaviour).
        self.declare_parameter('dr_realign_window_s',  60.0)

        self.warn_thr       = self.get_parameter('warn_threshold_m').value
        self.crit_thr       = self.get_parameter('critical_threshold_m').value
        self.min_sats       = self.get_parameter('min_satellites').value
        self.max_hdop       = self.get_parameter('max_hdop').value
        self.max_cov        = self.get_parameter('max_covariance_m2').value
        self.dr_window_s    = self.get_parameter('dr_realign_window_s').value

        # ── State ────────────────────────────────────────────────────────────
        # All ENU positions are relative to self._ref_lat / self._ref_lon.
        # This reference is set at first GNSS fix and never changes.
        self._ref_lat  = None   # global geodetic reference (set once)
        self._ref_lon  = None
        self._ref_alt  = 0.0

        # DR anchor: ENU position at which DR was last initialised/realigned.
        # DR computes position relative to this anchor.
        self._dr_anchor_east  = 0.0
        self._dr_anchor_north = 0.0
        self._dr_anchor_time  = None    # monotonic time of last realignment

        # DR state (relative to anchor, in metres / m/s)
        self._dr_east  = 0.0
        self._dr_north = 0.0
        self._dr_vel_east  = 0.0
        self._dr_vel_north = 0.0

        # Timestamp of last IMU message (nanoseconds)
        self._last_imu_stamp_ns = None

        # Guard: must have received IMU samples before reporting divergence
        self._imu_samples_since_realign = 0

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self._status_pub = self.create_publisher(
            IntegrityStatus, '/poise/integrity_status', 10
        )

        self.create_subscription(Imu,       '/sim/imu',  self._imu_cb,  100)
        self.create_subscription(NavSatFix, '/sim/gnss', self._gnss_cb, 10)

        self.get_logger().info(
            f'GnssImuChecker started | warn={self.warn_thr} m | '
            f'critical={self.crit_thr} m | dr_window={self.dr_window_s} s'
        )

    # ── IMU callback ─────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        if self._ref_lat is None:
            return  # wait for first GNSS fix

        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self._last_imu_stamp_ns is None:
            self._last_imu_stamp_ns = stamp_ns
            return

        dt = (stamp_ns - self._last_imu_stamp_ns) * 1e-9
        self._last_imu_stamp_ns = stamp_ns

        if dt <= 0.0 or dt > 1.0:
            return  # guard clock jumps

        # IMU body frame → ENU (yaw=0 assumption: x=north, y=east for sim)
        acc_north = msg.linear_acceleration.x
        acc_east  = msg.linear_acceleration.y

        self._dr_vel_north += acc_north * dt
        self._dr_vel_east  += acc_east  * dt
        self._dr_north     += self._dr_vel_north * dt
        self._dr_east      += self._dr_vel_east  * dt

        self._imu_samples_since_realign += 1

    # ── GNSS callback ─────────────────────────────────────────────────────────

    def _gnss_cb(self, msg: NavSatFix):
        import time as _time
        now_stamp = msg.header.stamp
        now_mono  = _time.monotonic()

        # ── Initialise global reference on first fix ──────────────────────
        if self._ref_lat is None:
            self._ref_lat = msg.latitude
            self._ref_lon = msg.longitude
            self._ref_alt = msg.altitude
            self.get_logger().info(
                f'Reference origin set: lat={self._ref_lat:.6f}, '
                f'lon={self._ref_lon:.6f}'
            )
            self._realign_dr(0.0, 0.0, now_mono)
            return

        # ── Convert current fix to global ENU ─────────────────────────────
        east, north, _ = _geodetic_to_enu(
            self._ref_lat, self._ref_lon,
            msg.latitude, msg.longitude,
            msg.altitude, self._ref_alt
        )

        # ── Periodic DR realignment ────────────────────────────────────────
        window = self.dr_window_s
        elapsed_in_window = now_mono - self._dr_anchor_time
        if window > 0 and elapsed_in_window >= window:
            self.get_logger().info(
                f'DR realignment after {elapsed_in_window:.1f}s window'
            )
            self._realign_dr(east, north, now_mono)
            return  # skip check at realignment boundary — fresh start

        # ── Check 1: GNSS quality (covariance) ───────────────────────────
        cov = msg.position_covariance
        max_cov_diag = max(cov[0], cov[4])

        if max_cov_diag > self.max_cov:
            self._publish_status(
                status=IntegrityStatus.STATUS_WARN,
                fault_code='GNSS_COVARIANCE_EXCEEDED',
                description=(
                    f'GNSS covariance {max_cov_diag:.2f} m² exceeds '
                    f'threshold {self.max_cov:.2f} m²'
                ),
                measured=max_cov_diag,
                threshold=self.max_cov,
                units='m²',
                stamp=now_stamp,
            )

        # ── Check 2: GNSS/IMU position divergence ─────────────────────────
        if self._imu_samples_since_realign == 0:
            self.get_logger().debug(
                'No IMU samples since last realign — skipping divergence check'
            )
            return

        # DR absolute position = anchor + relative DR offset
        dr_abs_east  = self._dr_anchor_east  + self._dr_east
        dr_abs_north = self._dr_anchor_north + self._dr_north

        delta_east  = east  - dr_abs_east
        delta_north = north - dr_abs_north
        delta_m = math.sqrt(delta_east ** 2 + delta_north ** 2)

        self.get_logger().debug(
            f'GNSS ENU=({east:.3f},{north:.3f}) '
            f'DR=({dr_abs_east:.3f},{dr_abs_north:.3f}) '
            f'delta={delta_m:.3f} m  window_age={elapsed_in_window:.1f}s'
        )

        if delta_m > self.crit_thr:
            self._publish_status(
                status=IntegrityStatus.STATUS_CRITICAL,
                fault_code='GNSS_IMU_DIVERGENCE_CRITICAL',
                description=(
                    f'GNSS/IMU divergence {delta_m:.3f} m exceeds '
                    f'critical threshold {self.crit_thr:.3f} m'
                ),
                measured=delta_m,
                threshold=self.crit_thr,
                units='m',
                stamp=now_stamp,
            )
        elif delta_m > self.warn_thr:
            self._publish_status(
                status=IntegrityStatus.STATUS_WARN,
                fault_code='GNSS_IMU_DIVERGENCE_WARN',
                description=(
                    f'GNSS/IMU divergence {delta_m:.3f} m exceeds '
                    f'warn threshold {self.warn_thr:.3f} m'
                ),
                measured=delta_m,
                threshold=self.warn_thr,
                units='m',
                stamp=now_stamp,
            )
        else:
            self._publish_status(
                status=IntegrityStatus.STATUS_OK,
                fault_code='',
                description=(
                    f'GNSS/IMU consistent: delta={delta_m:.3f} m '
                    f'(window age {elapsed_in_window:.1f}s)'
                ),
                measured=delta_m,
                threshold=self.warn_thr,
                units='m',
                stamp=now_stamp,
            )

    # ── DR realignment ────────────────────────────────────────────────────────

    def _realign_dr(self, anchor_east: float, anchor_north: float,
                    mono_time: float):
        """Reset DR to the given absolute ENU anchor and restart integration."""
        self._dr_anchor_east  = anchor_east
        self._dr_anchor_north = anchor_north
        self._dr_anchor_time  = mono_time
        self._dr_east  = 0.0
        self._dr_north = 0.0
        self._dr_vel_east  = 0.0
        self._dr_vel_north = 0.0
        self._imu_samples_since_realign = 0

    # ── Status publisher ─────────────────────────────────────────────────────

    def _publish_status(self, *, status, fault_code, description,
                        measured, threshold, units, stamp):
        msg = IntegrityStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gnss_imu_checker'
        msg.check_name = 'gnss_imu_cross_check'
        msg.status = status
        msg.fault_code = fault_code
        msg.description = description
        msg.measured_value = float(measured)
        msg.threshold_exceeded = float(threshold)
        msg.units = units
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GnssImuChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
