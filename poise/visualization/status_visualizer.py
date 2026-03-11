#!/usr/bin/env python3
"""
POISE Status Visualizer — Phase 4.

Translates POISE output topics to RViz2-compatible marker messages.
This node is a pure visualization translator — it contains no integrity logic.

Subscriptions
-------------
/poise/system_integrity  std_msgs/String (JSON)          SYSTEM_STATUS_QOS
/poise/integrity_status  poise/IntegrityStatus           INTEGRITY_QOS
/sim/gnss                sensor_msgs/NavSatFix           SENSOR_QOS
/poise/dr_position       geometry_msgs/PointStamped      INTEGRITY_QOS

Publications (all markers in frame 'map')
-----------------------------------------
/poise/viz/trust_state       Marker       Text — current integrity status
/poise/viz/fault_indicators  MarkerArray  Traffic-light spheres + labels per checker
/poise/viz/gnss_current      Marker       Sphere at current GNSS position
/poise/viz/gnss_trail        Marker       Line strip — GNSS position history (50 pts, capped at 4.5m)
/poise/viz/dr_current        Marker       Sphere at current DR position
/poise/viz/dr_trail          Marker       Line strip — DR position history (50 pts, capped at 4.5m)
/poise/viz/grid              MarkerArray  10x10m reference grid (faint 0.5m + thick 2.0m)
/poise/viz/thresholds        Marker       Yellow (1.5m) and orange (3.0m) circles
/poise/viz/legend            MarkerArray  Legend panel text markers
"""

import collections
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from poise.msg import IntegrityStatus
from poise.qos import SENSOR_QOS, INTEGRITY_QOS, SYSTEM_STATUS_QOS


# ── Constants ────────────────────────────────────────────────────────────────

_R_EARTH = 6_378_137.0  # WGS-84 semi-major axis (metres)

# Map check_name → checker node name (for worst-case status per traffic light)
_CHECK_TO_NODE: dict[str, str] = {
    # gnss_imu_checker checks (and heartbeat)
    'gnss_imu_checker':      'gnss_imu_checker',
    'gnss_dropout':          'gnss_imu_checker',
    'gnss_covariance':       'gnss_imu_checker',
    'gnss_satellites':       'gnss_imu_checker',
    'gnss_imu_cross_check':  'gnss_imu_checker',
    # calibration_validator checks (and heartbeat)
    'imu_accel_envelope':    'calibration_validator',
    'imu_gyro_envelope':     'calibration_validator',
    'imu_gravity_envelope':  'calibration_validator',
    'gnss_geofence':         'calibration_validator',
    'gnss_fix_type':         'calibration_validator',
    'calibration_validator': 'calibration_validator',
    # extrinsic_validator checks
    'imu_extrinsic':         'extrinsic_validator',
}

_CHECKERS = ['gnss_imu_checker', 'calibration_validator', 'extrinsic_validator']

# Traffic-light panel — right of map boundary, separated from position tracks
_TRAFFIC_LIGHT_POSITIONS = {
    'gnss_imu_checker':      (6.0,  2.0, 0.0),
    'calibration_validator': (6.0,  0.0, 0.0),
    'extrinsic_validator':   (6.0, -2.0, 0.0),
}

# Status → (r, g, b) color
_STATUS_COLOR = {
    IntegrityStatus.STATUS_OK:       (0.0, 1.0, 0.0),   # green
    IntegrityStatus.STATUS_WARN:     (1.0, 1.0, 0.0),   # yellow
    IntegrityStatus.STATUS_CRITICAL: (1.0, 0.0, 0.0),   # red
    None:                            (0.5, 0.5, 0.5),   # grey — no data
}

_TRUST_COLOR = {
    'TRUSTED':   (0.0, 1.0, 0.0),
    'DEGRADED':  (1.0, 1.0, 0.0),
    'UNTRUSTED': (1.0, 0.0, 0.0),
}

# Legend items: (text, r, g, b)
_LEGEND_ITEMS = [
    ('● GNSS Position',              0.2, 0.4, 1.0),
    ('● DR Position',                1.0, 0.5, 0.0),
    ('— DEGRADED threshold (1.5m)', 1.0, 1.0, 0.0),
    ('— UNTRUSTED threshold (3.0m)', 1.0, 0.5, 0.0),
]


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


def _color_msg(r, g, b, a=1.0):
    from std_msgs.msg import ColorRGBA
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def _pt(x, y, z=0.0):
    """Convenience: build a geometry_msgs/Point."""
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def _line_list_points(step, lo, hi):
    """Return a flat list of Point pairs for a square grid LINE_LIST.

    Each grid line contributes two consecutive points (start, end).
    step: interval between gridlines (metres)
    lo, hi: grid extent in both axes
    """
    vals = []
    v = lo
    while v <= hi + 1e-9:
        vals.append(round(v, 6))
        v += step
    pts = []
    for x in vals:                      # vertical lines
        pts += [_pt(x, lo), _pt(x, hi)]
    for y in vals:                      # horizontal lines
        pts += [_pt(lo, y), _pt(hi, y)]
    return pts


def _circle_points(radius, n=72):
    """Return n+1 Points approximating a closed circle in the XY plane."""
    pts = []
    for i in range(n + 1):             # +1 closes the loop
        angle = math.radians(i * 360.0 / n)
        pts.append(_pt(radius * math.cos(angle), radius * math.sin(angle)))
    return pts


class StatusVisualizer(Node):
    """Translates POISE output topics to RViz2 marker messages."""

    def __init__(self):
        super().__init__('status_visualizer')

        # ── State ────────────────────────────────────────────────────────────
        self._trust_state = 'TRUSTED'

        # Per check_name last received status (int, or absent if never received)
        self._check_status: dict[str, int] = {}
        # Which checker nodes have sent at least one message
        self._node_seen: set[str] = set()

        # GNSS dropout tracking (from gnss_dropout check status)
        self._gnss_dropout_active = False

        # GNSS ENU reference (set on first fix)
        self._gnss_ref_lat: float | None = None
        self._gnss_ref_lon: float | None = None
        self._gnss_ref_alt: float = 0.0

        # Position trails (ENU, x=east, y=north)
        self._gnss_trail: collections.deque = collections.deque(maxlen=50)
        self._dr_trail: collections.deque   = collections.deque(maxlen=50)

        # Current positions (may be None before first message)
        self._gnss_current: tuple[float, float] | None = None
        self._dr_current:   tuple[float, float] | None = None

        # ── Pre-computed static geometry ──────────────────────────────────────
        # Grid point lists (pre-computed, reused every publish cycle)
        self._grid_faint_pts = _line_list_points(0.5, -5.0, 5.0)
        self._grid_thick_pts = _line_list_points(2.0, -4.0, 4.0)

        # Threshold circle points
        self._circle_warn_pts  = _circle_points(1.5)   # DEGRADED (yellow)
        self._circle_crit_pts  = _circle_points(3.0)   # UNTRUSTED (orange)

        # ── Publishers ───────────────────────────────────────────────────────
        _pub = lambda topic, msg_type: self.create_publisher(msg_type, topic, 10)
        self._pub_trust       = _pub('/poise/viz/trust_state',      Marker)
        self._pub_faults      = _pub('/poise/viz/fault_indicators',  MarkerArray)
        self._pub_gnss_cur    = _pub('/poise/viz/gnss_current',      Marker)
        self._pub_gnss_trail  = _pub('/poise/viz/gnss_trail',        Marker)
        self._pub_dr_cur      = _pub('/poise/viz/dr_current',        Marker)
        self._pub_dr_trail    = _pub('/poise/viz/dr_trail',          Marker)
        self._pub_grid        = _pub('/poise/viz/grid',              MarkerArray)
        self._pub_thresholds  = _pub('/poise/viz/thresholds',        Marker)
        self._pub_legend      = _pub('/poise/viz/legend',            MarkerArray)

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            String, '/poise/system_integrity',
            self._system_integrity_cb, SYSTEM_STATUS_QOS
        )
        self.create_subscription(
            IntegrityStatus, '/poise/integrity_status',
            self._integrity_status_cb, INTEGRITY_QOS
        )
        self.create_subscription(
            NavSatFix, '/sim/gnss',
            self._gnss_cb, SENSOR_QOS
        )
        self.create_subscription(
            PointStamped, '/poise/dr_position',
            self._dr_position_cb, INTEGRITY_QOS
        )

        # 10 Hz publish timer — keeps markers fresh and handles RViz reconnections
        self._timer = self.create_timer(0.1, self._publish_all)

        self.get_logger().info('StatusVisualizer started')

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _system_integrity_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._trust_state = data.get('state', 'TRUSTED')
        except (json.JSONDecodeError, KeyError):
            pass

    def _integrity_status_cb(self, msg: IntegrityStatus):
        check = msg.check_name
        self._check_status[check] = msg.status

        # Track which checker nodes are alive
        node = _CHECK_TO_NODE.get(check)
        if node:
            self._node_seen.add(node)

        # Track GNSS dropout for current-position marker colour
        if check == 'gnss_dropout':
            self._gnss_dropout_active = (msg.status != IntegrityStatus.STATUS_OK)

    def _gnss_cb(self, msg: NavSatFix):
        # Establish ENU origin on first fix
        if self._gnss_ref_lat is None:
            self._gnss_ref_lat = msg.latitude
            self._gnss_ref_lon = msg.longitude
            self._gnss_ref_alt = msg.altitude

        east, north, _ = _geodetic_to_enu(
            self._gnss_ref_lat, self._gnss_ref_lon,
            msg.latitude, msg.longitude,
            msg.altitude, self._gnss_ref_alt,
        )
        self._gnss_current = (east, north)
        # Visualization limit (not an integrity limit): stop extending the trail once
        # the position leaves the 4.5m map boundary so the trail stays readable.
        # The current-position sphere continues to follow the true position.
        if max(abs(east), abs(north)) <= 4.5:
            self._gnss_trail.append((east, north))

    def _dr_position_cb(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        self._dr_current = (x, y)
        # Visualization limit (not an integrity limit): stop extending the trail once
        # the position leaves the 4.5m map boundary so the trail stays readable.
        # The current-position sphere continues to follow the true position.
        if max(abs(x), abs(y)) <= 4.5:
            self._dr_trail.append((x, y))

    # ── Status helpers ────────────────────────────────────────────────────────

    def _get_checker_status(self, node_name: str):
        """Return worst status across all checks for this checker node.

        Returns None if no message has been received yet (grey traffic light).
        """
        if node_name not in self._node_seen:
            return None
        worst = IntegrityStatus.STATUS_OK
        for check, node in _CHECK_TO_NODE.items():
            if node != node_name:
                continue
            s = self._check_status.get(check)
            if s is not None and s > worst:
                worst = s
        return worst

    # ── Publish all markers ───────────────────────────────────────────────────

    def _publish_all(self):
        now = self.get_clock().now().to_msg()
        self._publish_trust_state(now)
        self._publish_fault_indicators(now)
        self._publish_gnss_markers(now)
        self._publish_dr_markers(now)
        self._publish_grid(now)
        self._publish_thresholds(now)
        self._publish_legend(now)

    # ── Trust state text marker ───────────────────────────────────────────────

    def _publish_trust_state(self, stamp):
        state = self._trust_state
        r, g, b = _TRUST_COLOR.get(state, (1.0, 1.0, 1.0))

        m = Marker()
        m.header.stamp    = stamp
        m.header.frame_id = 'map'
        m.ns              = 'trust_state'
        m.id              = 0
        m.type            = Marker.TEXT_VIEW_FACING
        m.action          = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 7.0
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.z         = 0.5
        m.color           = _color_msg(r, g, b, 1.0)
        m.text            = f'INTEGRITY STATUS: {state}'
        self._pub_trust.publish(m)

    # ── Traffic-light fault indicators ───────────────────────────────────────

    def _publish_fault_indicators(self, stamp):
        markers = []

        # Panel title
        title = Marker()
        title.header.stamp    = stamp
        title.header.frame_id = 'map'
        title.ns              = 'fault_indicators'
        title.id              = 20
        title.type            = Marker.TEXT_VIEW_FACING
        title.action          = Marker.ADD
        title.pose.position.x = 6.0
        title.pose.position.y = 3.0
        title.pose.position.z = 0.0
        title.pose.orientation.w = 1.0
        title.scale.z = 0.3
        title.color   = _color_msg(1.0, 1.0, 1.0, 1.0)
        title.text    = 'CHECK STATUS'
        markers.append(title)

        for idx, checker in enumerate(_CHECKERS):
            status = self._get_checker_status(checker)
            r, g, b = _STATUS_COLOR[status]
            px, py, pz = _TRAFFIC_LIGHT_POSITIONS[checker]

            # Sphere (radius 0.3m → diameter 0.6m → scale 0.6)
            sphere = Marker()
            sphere.header.stamp    = stamp
            sphere.header.frame_id = 'map'
            sphere.ns              = 'fault_indicators'
            sphere.id              = idx
            sphere.type            = Marker.SPHERE
            sphere.action          = Marker.ADD
            sphere.pose.position.x = px
            sphere.pose.position.y = py
            sphere.pose.position.z = pz
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.6
            sphere.scale.y = 0.6
            sphere.scale.z = 0.6
            sphere.color   = _color_msg(r, g, b, 1.0)
            markers.append(sphere)

            # Label — 0.8m below sphere in Y (top-down view), scale 0.35
            label = Marker()
            label.header.stamp    = stamp
            label.header.frame_id = 'map'
            label.ns              = 'fault_indicators'
            label.id              = 10 + idx
            label.type            = Marker.TEXT_VIEW_FACING
            label.action          = Marker.ADD
            label.pose.position.x = px
            label.pose.position.y = py - 0.8
            label.pose.position.z = pz
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color   = _color_msg(1.0, 1.0, 1.0, 0.9)
            label.text    = checker
            markers.append(label)

        ma = MarkerArray()
        ma.markers = markers
        self._pub_faults.publish(ma)

    # ── GNSS position markers ─────────────────────────────────────────────────

    def _publish_gnss_markers(self, stamp):
        # Current position sphere
        cur = Marker()
        cur.header.stamp    = stamp
        cur.header.frame_id = 'map'
        cur.ns              = 'gnss'
        cur.id              = 0
        cur.type            = Marker.SPHERE
        cur.action          = Marker.ADD
        cur.pose.orientation.w = 1.0
        cur.scale.x = 1.0
        cur.scale.y = 1.0
        cur.scale.z = 1.0

        if self._gnss_current is not None:
            cur.pose.position.x = self._gnss_current[0]
            cur.pose.position.y = self._gnss_current[1]
            cur.pose.position.z = 0.0
            if self._gnss_dropout_active:
                cur.color = _color_msg(0.5, 0.5, 0.5, 0.8)
            else:
                cur.color = _color_msg(0.2, 0.4, 1.0, 1.0)   # blue — matches trail and legend
        else:
            cur.pose.position.x = 0.0
            cur.pose.position.y = 0.0
            cur.pose.position.z = 0.0
            cur.color = _color_msg(0.5, 0.5, 0.5, 0.3)
        self._pub_gnss_cur.publish(cur)

        # Trail line strip
        trail = Marker()
        trail.header.stamp    = stamp
        trail.header.frame_id = 'map'
        trail.ns              = 'gnss'
        trail.id              = 1
        trail.type            = Marker.LINE_STRIP
        trail.action          = Marker.ADD
        trail.pose.orientation.w = 1.0
        trail.scale.x = 0.1
        trail.color   = _color_msg(0.2, 0.4, 1.0, 0.8)   # blue
        for (ex, ny) in self._gnss_trail:
            trail.points.append(_pt(ex, ny, 0.0))
        self._pub_gnss_trail.publish(trail)

    # ── DR position markers ───────────────────────────────────────────────────

    def _publish_dr_markers(self, stamp):
        # Current position sphere
        cur = Marker()
        cur.header.stamp    = stamp
        cur.header.frame_id = 'map'
        cur.ns              = 'dr'
        cur.id              = 0
        cur.type            = Marker.SPHERE
        cur.action          = Marker.ADD
        cur.pose.orientation.w = 1.0
        cur.scale.x = 1.0
        cur.scale.y = 1.0
        cur.scale.z = 1.0
        cur.color   = _color_msg(1.0, 0.5, 0.0, 1.0)    # orange

        if self._dr_current is not None:
            cur.pose.position.x = self._dr_current[0]
            cur.pose.position.y = self._dr_current[1]
        cur.pose.position.z = 0.1   # slight elevation above GNSS marker
        self._pub_dr_cur.publish(cur)

        # Trail line strip
        trail = Marker()
        trail.header.stamp    = stamp
        trail.header.frame_id = 'map'
        trail.ns              = 'dr'
        trail.id              = 1
        trail.type            = Marker.LINE_STRIP
        trail.action          = Marker.ADD
        trail.pose.orientation.w = 1.0
        trail.scale.x = 0.1
        trail.color   = _color_msg(1.0, 0.5, 0.0, 0.7)   # orange
        for (ex, ny) in self._dr_trail:
            trail.points.append(_pt(ex, ny, 0.1))
        self._pub_dr_trail.publish(trail)

    # ── Grid ─────────────────────────────────────────────────────────────────

    def _publish_grid(self, stamp):
        """Publish two LINE_LIST markers: faint 0.5m grid and thick 2.0m grid."""
        markers = []

        # Faint grid — 0.5m intervals, colour (0.3, 0.3, 0.3), width 0.02m
        faint = Marker()
        faint.header.stamp    = stamp
        faint.header.frame_id = 'map'
        faint.ns              = 'grid'
        faint.id              = 0
        faint.type            = Marker.LINE_LIST
        faint.action          = Marker.ADD
        faint.pose.orientation.w = 1.0
        faint.scale.x = 0.02
        faint.color   = _color_msg(0.3, 0.3, 0.3, 1.0)
        faint.points  = self._grid_faint_pts
        markers.append(faint)

        # Thick grid — 2.0m intervals, colour (0.5, 0.5, 0.5), width 0.05m
        thick = Marker()
        thick.header.stamp    = stamp
        thick.header.frame_id = 'map'
        thick.ns              = 'grid'
        thick.id              = 1
        thick.type            = Marker.LINE_LIST
        thick.action          = Marker.ADD
        thick.pose.orientation.w = 1.0
        thick.scale.x = 0.05
        thick.color   = _color_msg(0.5, 0.5, 0.5, 1.0)
        thick.points  = self._grid_thick_pts
        markers.append(thick)

        ma = MarkerArray()
        ma.markers = markers
        self._pub_grid.publish(ma)

    # ── Threshold circles ─────────────────────────────────────────────────────

    def _publish_thresholds(self, stamp):
        """Publish DEGRADED (yellow 1.5m) and UNTRUSTED (orange 3.0m) circles.

        Circles are centred on the current GNSS position so they always show
        the divergence envelope around the live fix — i.e. how far the DR
        position must drift before each alarm triggers.  Circle points are
        pre-computed in local marker frame; the marker pose carries the offset.
        """
        if self._gnss_current is None:
            return  # wait for first GNSS fix

        gx, gy = self._gnss_current

        for m_id, (pts, r, g, b, a) in enumerate([
            (self._circle_warn_pts, 1.0, 1.0, 0.0, 0.6),   # yellow DEGRADED
            (self._circle_crit_pts, 1.0, 0.5, 0.0, 0.6),   # orange UNTRUSTED
        ]):
            circle = Marker()
            circle.header.stamp    = stamp
            circle.header.frame_id = 'map'
            circle.ns              = 'thresholds'
            circle.id              = m_id
            circle.type            = Marker.LINE_STRIP
            circle.action          = Marker.ADD
            circle.pose.position.x = gx
            circle.pose.position.y = gy
            circle.pose.position.z = 0.0
            circle.pose.orientation.w = 1.0
            circle.scale.x = 0.08    # line width
            circle.color   = _color_msg(r, g, b, a)
            circle.points  = pts
            self._pub_thresholds.publish(circle)

    # ── Legend panel ─────────────────────────────────────────────────────────

    def _publish_legend(self, stamp):
        """Publish four text markers as a legend at top-left of the map view."""
        markers = []
        for idx, (text, r, g, b) in enumerate(_LEGEND_ITEMS):
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = 'map'
            m.ns              = 'legend'
            m.id              = idx
            m.type            = Marker.TEXT_VIEW_FACING
            m.action          = Marker.ADD
            m.pose.position.x = -4.5
            m.pose.position.y = 4.5 - idx * 0.4
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.z = 0.25
            m.color   = _color_msg(r, g, b, 1.0)
            m.text    = text
            markers.append(m)

        ma = MarkerArray()
        ma.markers = markers
        self._pub_legend.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = StatusVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
