#!/usr/bin/env python3
"""
GNSS/IMU Cross-Check Node — Phase 2.

Subscribes to /sim/gnss (sensor_msgs/NavSatFix) and /sim/imu (sensor_msgs/Imu).

Checks performed
----------------
gnss_dropout
    Timer-based. Publishes GNSS_DROPOUT (recoverable=True, STATUS_WARN) when
    no GNSS fix has been received for longer than gnss_dropout_timeout_s.
    Clears automatically when GNSS resumes.  On re-acquisition after a dropout,
    DR is force-realigned to prevent a spurious GNSS_IMU_DIVERGENCE.

gnss_covariance
    Per-fix. Publishes GNSS_HIGH_COVARIANCE (recoverable=True, STATUS_WARN)
    when the reported horizontal covariance diagonal exceeds max_covariance_m2.

gnss_satellites
    Per-fix. Publishes GNSS_LOW_SATELLITES (recoverable=True, STATUS_WARN) when
    NavSatFix.status indicates STATUS_NO_FIX.  In the sim this never triggers
    (sim always publishes STATUS_FIX); the check is exercised by real GNSS data.

gnss_imu_cross_check
    Per-fix using sliding DR window (dr_realign_window_s, default 60 s).
    Publishes GNSS_IMU_DIVERGENCE_WARN (recoverable=False, STATUS_WARN) or
    GNSS_IMU_DIVERGENCE_CRITICAL (recoverable=False, STATUS_CRITICAL) when the
    Euclidean delta between GNSS and dead-reckoned position exceeds thresholds.

Fault recoverability
--------------------
Recoverable (environmental):  GNSS_DROPOUT, GNSS_HIGH_COVARIANCE, GNSS_LOW_SATELLITES
Non-recoverable (integrity):   GNSS_IMU_DIVERGENCE_WARN, GNSS_IMU_DIVERGENCE_CRITICAL

All fault status messages are published on /poise/integrity_status using
INTEGRITY_QOS (RELIABLE / VOLATILE).  Sensor subscriptions use SENSOR_QOS
(BEST_EFFORT / VOLATILE) for Autoware compatibility.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, Imu, NavSatStatus

from poise.msg import IntegrityStatus
from poise.qos import SENSOR_QOS, INTEGRITY_QOS


# WGS-84 semi-major axis (metres)
_R_EARTH = 6_378_137.0


def _geodetic_to_enu(lat_ref_deg, lon_ref_deg,
                      lat_deg, lon_deg, alt_m=0.0, alt_ref_m=0.0):
    """Flat-Earth ENU conversion — valid for baselines < ~10 km."""
    dlat = math.radians(lat_deg - lat_ref_deg)
    dlon = math.radians(lon_deg - lon_ref_deg)
    lat_ref = math.radians(lat_ref_deg)
    north = dlat * _R_EARTH
    east  = dlon * _R_EARTH * math.cos(lat_ref)
    up    = alt_m - alt_ref_m
    return east, north, up


class GnssImuChecker(Node):
    """Cross-checks GNSS position against IMU dead-reckoning (Phase 2)."""

    def __init__(self):
        super().__init__('gnss_imu_checker')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('warn_threshold_m',       1.5)
        self.declare_parameter('critical_threshold_m',   3.0)
        self.declare_parameter('min_satellites',          6)
        self.declare_parameter('max_hdop',                2.0)
        self.declare_parameter('max_covariance_m2',      25.0)
        self.declare_parameter('dr_realign_window_s',   60.0)
        # Seconds without a GNSS fix before GNSS_DROPOUT is raised.
        # Should be > 2 × GNSS period (at 10 Hz → 0.2 s); default 2.0 s.
        self.declare_parameter('gnss_dropout_timeout_s', 2.0)

        self.warn_thr            = self.get_parameter('warn_threshold_m').value
        self.crit_thr            = self.get_parameter('critical_threshold_m').value
        self.min_sats            = self.get_parameter('min_satellites').value
        self.max_hdop            = self.get_parameter('max_hdop').value
        self.max_cov             = self.get_parameter('max_covariance_m2').value
        self.dr_window_s         = self.get_parameter('dr_realign_window_s').value
        self.dropout_timeout_s   = self.get_parameter('gnss_dropout_timeout_s').value

        # ── DR state ─────────────────────────────────────────────────────────
        self._ref_lat  = None
        self._ref_lon  = None
        self._ref_alt  = 0.0

        self._dr_anchor_east  = 0.0
        self._dr_anchor_north = 0.0
        self._dr_anchor_time  = None

        self._dr_east        = 0.0
        self._dr_north       = 0.0
        self._dr_vel_east    = 0.0
        self._dr_vel_north   = 0.0

        self._last_imu_stamp_ns        = None
        self._imu_samples_since_realign = 0

        # ── Dropout tracking ─────────────────────────────────────────────────
        self._last_gnss_time: float | None = None   # monotonic
        self._dropout_active = False
        self._dropout_was_active = False  # force DR realign on re-acquisition

        # ── Per-check OK/WARN state (for clearing logic) ─────────────────────
        self._covariance_warn_active  = False
        self._satellites_warn_active  = False
        self._cross_check_fault_active = False  # True when WARN or CRITICAL active

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self._status_pub = self.create_publisher(
            IntegrityStatus, '/poise/integrity_status', INTEGRITY_QOS
        )

        self.create_subscription(Imu,       '/sim/imu',  self._imu_cb,  SENSOR_QOS)
        self.create_subscription(NavSatFix, '/sim/gnss', self._gnss_cb, SENSOR_QOS)

        # 1 Hz timer — dropout detection
        self._dropout_timer = self.create_timer(1.0, self._dropout_check_cb)

        self.get_logger().info(
            f'GnssImuChecker started | warn={self.warn_thr} m | '
            f'critical={self.crit_thr} m | dr_window={self.dr_window_s} s | '
            f'dropout_timeout={self.dropout_timeout_s} s'
        )

    # ── IMU callback ──────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        if self._ref_lat is None:
            return

        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self._last_imu_stamp_ns is None:
            self._last_imu_stamp_ns = stamp_ns
            return

        dt = (stamp_ns - self._last_imu_stamp_ns) * 1e-9
        self._last_imu_stamp_ns = stamp_ns

        if dt <= 0.0 or dt > 1.0:
            return

        acc_north = msg.linear_acceleration.x
        acc_east  = msg.linear_acceleration.y

        self._dr_vel_north += acc_north * dt
        self._dr_vel_east  += acc_east  * dt
        self._dr_north     += self._dr_vel_north * dt
        self._dr_east      += self._dr_vel_east  * dt

        self._imu_samples_since_realign += 1

    # ── GNSS callback ─────────────────────────────────────────────────────────

    def _gnss_cb(self, msg: NavSatFix):
        now_mono  = time.monotonic()
        now_stamp = msg.header.stamp

        # Update last-seen time (used by dropout detector)
        was_in_dropout = self._dropout_active
        self._last_gnss_time = now_mono

        # Clear dropout if it was active — publish STATUS_OK for dropout check
        if self._dropout_active:
            self._dropout_active = False
            self._dropout_was_active = True   # signal DR realign needed
            self._publish_status(
                check_name='gnss_dropout',
                status=IntegrityStatus.STATUS_OK,
                recoverable=True,
                fault_code='',
                description='GNSS re-acquired',
                measured=0.0, threshold=self.dropout_timeout_s,
                units='s', stamp=now_stamp,
            )

        # ── First fix: initialise reference ───────────────────────────────
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

        # ── Convert to ENU ─────────────────────────────────────────────────
        east, north, _ = _geodetic_to_enu(
            self._ref_lat, self._ref_lon,
            msg.latitude, msg.longitude,
            msg.altitude, self._ref_alt
        )

        # ── DR realign on re-acquisition after dropout ─────────────────────
        if self._dropout_was_active:
            self._dropout_was_active = False
            self.get_logger().info(
                'DR realigned to GNSS on dropout re-acquisition'
            )
            self._realign_dr(east, north, now_mono)
            return

        # ── Check: satellite quality (NavSatFix status field) ─────────────
        if msg.status.status <= NavSatStatus.STATUS_NO_FIX:
            if not self._satellites_warn_active:
                self._satellites_warn_active = True
            self._publish_status(
                check_name='gnss_satellites',
                status=IntegrityStatus.STATUS_WARN,
                recoverable=True,
                fault_code='GNSS_LOW_SATELLITES',
                description='GNSS status indicates no fix (low/no satellites)',
                measured=float(msg.status.status),
                threshold=float(NavSatStatus.STATUS_FIX),
                units='fix_status', stamp=now_stamp,
            )
        elif self._satellites_warn_active:
            self._satellites_warn_active = False
            self._publish_status(
                check_name='gnss_satellites',
                status=IntegrityStatus.STATUS_OK,
                recoverable=True,
                fault_code='',
                description='GNSS fix quality restored',
                measured=float(msg.status.status),
                threshold=float(NavSatStatus.STATUS_FIX),
                units='fix_status', stamp=now_stamp,
            )

        # ── Check: covariance ─────────────────────────────────────────────
        cov = msg.position_covariance
        max_cov_diag = max(cov[0], cov[4])
        if max_cov_diag > self.max_cov:
            if not self._covariance_warn_active:
                self._covariance_warn_active = True
            self._publish_status(
                check_name='gnss_covariance',
                status=IntegrityStatus.STATUS_WARN,
                recoverable=True,
                fault_code='GNSS_HIGH_COVARIANCE',
                description=(
                    f'GNSS covariance {max_cov_diag:.2f} m² exceeds '
                    f'threshold {self.max_cov:.2f} m²'
                ),
                measured=max_cov_diag,
                threshold=self.max_cov,
                units='m²', stamp=now_stamp,
            )
        elif self._covariance_warn_active:
            self._covariance_warn_active = False
            self._publish_status(
                check_name='gnss_covariance',
                status=IntegrityStatus.STATUS_OK,
                recoverable=True,
                fault_code='',
                description='GNSS covariance within threshold',
                measured=max_cov_diag,
                threshold=self.max_cov,
                units='m²', stamp=now_stamp,
            )

        # ── Periodic DR realignment ────────────────────────────────────────
        if self._dr_anchor_time is None:
            self._realign_dr(east, north, now_mono)
            return

        elapsed_in_window = now_mono - self._dr_anchor_time
        if self.dr_window_s > 0 and elapsed_in_window >= self.dr_window_s:
            self.get_logger().info(
                f'DR realignment after {elapsed_in_window:.1f}s window'
            )
            self._realign_dr(east, north, now_mono)
            return

        # ── Check: GNSS/IMU position divergence ───────────────────────────
        if self._imu_samples_since_realign == 0:
            return

        dr_abs_east  = self._dr_anchor_east  + self._dr_east
        dr_abs_north = self._dr_anchor_north + self._dr_north

        delta_east  = east  - dr_abs_east
        delta_north = north - dr_abs_north
        delta_m     = math.sqrt(delta_east ** 2 + delta_north ** 2)

        self.get_logger().debug(
            f'GNSS ENU=({east:.3f},{north:.3f}) '
            f'DR=({dr_abs_east:.3f},{dr_abs_north:.3f}) '
            f'delta={delta_m:.3f} m  age={elapsed_in_window:.1f}s'
        )

        if delta_m > self.crit_thr:
            self._cross_check_fault_active = True
            self._publish_status(
                check_name='gnss_imu_cross_check',
                status=IntegrityStatus.STATUS_CRITICAL,
                recoverable=False,
                fault_code='GNSS_IMU_DIVERGENCE_CRITICAL',
                description=(
                    f'GNSS/IMU divergence {delta_m:.3f} m exceeds '
                    f'critical threshold {self.crit_thr:.3f} m'
                ),
                measured=delta_m, threshold=self.crit_thr,
                units='m', stamp=now_stamp,
            )
        elif delta_m > self.warn_thr:
            self._cross_check_fault_active = True
            self._publish_status(
                check_name='gnss_imu_cross_check',
                status=IntegrityStatus.STATUS_WARN,
                recoverable=False,
                fault_code='GNSS_IMU_DIVERGENCE_WARN',
                description=(
                    f'GNSS/IMU divergence {delta_m:.3f} m exceeds '
                    f'warn threshold {self.warn_thr:.3f} m'
                ),
                measured=delta_m, threshold=self.warn_thr,
                units='m', stamp=now_stamp,
            )
        elif self._cross_check_fault_active:
            # Hysteresis: only clear once divergence drops well below warn threshold.
            # Prevents oscillation when drift ≈ threshold (which would reset the
            # escalation timer each cycle and prevent DEGRADED → UNTRUSTED).
            clear_thr = self.warn_thr * 0.5
            if delta_m < clear_thr:
                self._cross_check_fault_active = False
                self._publish_status(
                    check_name='gnss_imu_cross_check',
                    status=IntegrityStatus.STATUS_OK,
                    recoverable=False,
                    fault_code='',
                    description=(
                        f'GNSS/IMU consistent: delta={delta_m:.3f} m '
                        f'(window age {elapsed_in_window:.1f}s)'
                    ),
                    measured=delta_m, threshold=self.warn_thr,
                    units='m', stamp=now_stamp,
                )

    # ── Dropout detection timer ───────────────────────────────────────────────

    def _dropout_check_cb(self):
        """1 Hz check: raise GNSS_DROPOUT if no fix received recently."""
        if self._ref_lat is None:
            return  # haven't had a first fix yet

        now = time.monotonic()
        since_last = now - (self._last_gnss_time or 0.0)

        if since_last > self.dropout_timeout_s and not self._dropout_active:
            self._dropout_active = True
            stamp = self.get_clock().now().to_msg()
            self._publish_status(
                check_name='gnss_dropout',
                status=IntegrityStatus.STATUS_WARN,
                recoverable=True,
                fault_code='GNSS_DROPOUT',
                description=(
                    f'No GNSS fix received for {since_last:.1f}s '
                    f'(timeout={self.dropout_timeout_s}s)'
                ),
                measured=since_last,
                threshold=self.dropout_timeout_s,
                units='s',
                stamp=stamp,
            )
            self.get_logger().warn(
                f'GNSS_DROPOUT: {since_last:.1f}s without fix'
            )

    # ── DR realignment ────────────────────────────────────────────────────────

    def _realign_dr(self, anchor_east: float, anchor_north: float,
                    mono_time: float):
        self._dr_anchor_east  = anchor_east
        self._dr_anchor_north = anchor_north
        self._dr_anchor_time  = mono_time
        self._dr_east         = 0.0
        self._dr_north        = 0.0
        self._dr_vel_east     = 0.0
        self._dr_vel_north    = 0.0
        self._imu_samples_since_realign = 0
        # If a cross-check fault was active, clear it — DR anchor is now GNSS-aligned
        if self._cross_check_fault_active:
            self._cross_check_fault_active = False
            stamp = self.get_clock().now().to_msg()
            self._publish_status(
                check_name='gnss_imu_cross_check',
                status=IntegrityStatus.STATUS_OK,
                recoverable=False,
                fault_code='',
                description='Cross-check fault cleared on DR realignment',
                measured=0.0, threshold=self.warn_thr,
                units='m', stamp=stamp,
            )

    # ── Status publisher ─────────────────────────────────────────────────────

    def _publish_status(self, *, check_name, status, recoverable, fault_code,
                        description, measured, threshold, units, stamp):
        msg = IntegrityStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gnss_imu_checker'
        msg.check_name = check_name
        msg.status = status
        msg.recoverable = recoverable
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
