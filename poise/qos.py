"""
POISE QoS Profiles.

All QoS decisions are centralised here.  No node should construct a QoSProfile
inline — always import from this module so profiles can be audited in one place.

Profile rationale
-----------------
SENSOR_QOS
    BEST_EFFORT / VOLATILE / KEEP_LAST 10.
    Matches Autoware's sensor topic convention (NavSatFix, Imu).  A dropped
    sensor packet is less harmful than the buffering delay that RELIABLE
    would impose.  Using BEST_EFFORT here ensures compatibility when topics
    are remapped to live Autoware sensor sources.

INTEGRITY_QOS
    RELIABLE / VOLATILE / KEEP_LAST 10.
    Every fault report on /poise/integrity_status must be delivered.  A
    missed CRITICAL event would leave the aggregator unaware of a safety
    hazard.  VOLATILE is acceptable because the aggregator is always running;
    it does not need to receive old fault reports after (re)starting.

SYSTEM_STATUS_QOS
    RELIABLE / TRANSIENT_LOCAL / KEEP_LAST 1.
    Any supervisory controller or operator HMI that subscribes to
    /poise/system_integrity must receive the current trust state immediately
    on connection — it must never silently assume TRUSTED by default.
    TRANSIENT_LOCAL ensures late joiners get the last published state.
    depth=1: only the most recent state matters.
"""

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

INTEGRITY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

SYSTEM_STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
