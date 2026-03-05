#!/usr/bin/env python3
"""
Integrity Aggregator Node.

Subscribes to /poise/integrity_status (poise/IntegrityStatus).
Maintains a one-way trust state machine:

    TRUSTED → DEGRADED → UNTRUSTED

Transition rules (no automatic recovery — intentional safety design):
  - Any WARN  → DEGRADED
  - Any CRITICAL → UNTRUSTED immediately
  - Two simultaneous WARNs → UNTRUSTED
  - Single WARN sustained beyond warn_escalation_timeout_s → UNTRUSTED
  - State only moves toward UNTRUSTED; explicit /poise/reset required.

Publishes system state on /poise/system_integrity (std_msgs/String, JSON payload).
Writes JSON-lines log to file on every state transition and fault event.
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from poise.msg import IntegrityStatus


# ── State constants ──────────────────────────────────────────────────────────
STATE_TRUSTED   = 'TRUSTED'
STATE_DEGRADED  = 'DEGRADED'
STATE_UNTRUSTED = 'UNTRUSTED'

# State ordinal — only ever increases
_STATE_RANK = {STATE_TRUSTED: 0, STATE_DEGRADED: 1, STATE_UNTRUSTED: 2}


class IntegrityAggregator(Node):
    """Aggregates per-check integrity reports into a system-level trust state."""

    def __init__(self):
        super().__init__('integrity_aggregator')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('warn_escalation_timeout_s', 5.0)
        self.declare_parameter('log_file_path', '/tmp/poise_integrity_log.jsonl')

        self._escalation_timeout = self.get_parameter(
            'warn_escalation_timeout_s'
        ).value
        self._log_path = self.get_parameter('log_file_path').value

        # ── State ────────────────────────────────────────────────────────────
        self._state = STATE_TRUSTED
        self._active_warns: dict[str, float] = {}   # check_name → wall-clock time of first WARN
        self._active_crits: set[str]          = set()

        # ── Log file ─────────────────────────────────────────────────────────
        try:
            self._log_file = open(self._log_path, 'a', buffering=1)  # line-buffered
            self.get_logger().info(f'Integrity log: {self._log_path}')
        except OSError as e:
            self.get_logger().error(f'Cannot open log file {self._log_path}: {e}')
            self._log_file = None

        # ── Publisher / Subscriber ───────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/poise/system_integrity', 10)

        self.create_subscription(
            IntegrityStatus, '/poise/integrity_status',
            self._status_cb, 100
        )

        # Periodic check for warn escalation timeout
        self._timer = self.create_timer(1.0, self._escalation_check_cb)

        # Publish initial state
        self._publish_state(reason='node_startup')

        self.get_logger().info(
            f'IntegrityAggregator started | '
            f'escalation_timeout={self._escalation_timeout} s'
        )

    # ── Incoming integrity status ────────────────────────────────────────────

    def _status_cb(self, msg: IntegrityStatus):
        check = msg.check_name

        if msg.status == IntegrityStatus.STATUS_OK:
            # Clear warn/crit tracking for this check
            was_warned = check in self._active_warns
            was_crited = check in self._active_crits
            self._active_warns.pop(check, None)
            self._active_crits.discard(check)
            if was_warned or was_crited:
                self._log_event('fault_cleared', {
                    'check_name': check,
                    'state': self._state,
                })
            return

        # Log every non-OK event
        self._log_event('fault_detected', {
            'check_name':    check,
            'status':        msg.status,
            'fault_code':    msg.fault_code,
            'description':   msg.description,
            'measured':      msg.measured_value,
            'threshold':     msg.threshold_exceeded,
            'units':         msg.units,
            'system_state':  self._state,
        })

        if msg.status == IntegrityStatus.STATUS_CRITICAL:
            self._active_crits.add(check)
            self._transition_to(STATE_UNTRUSTED,
                                reason=f'CRITICAL from {check}: {msg.fault_code}')

        elif msg.status == IntegrityStatus.STATUS_WARN:
            # Record first-seen time if new
            if check not in self._active_warns:
                self._active_warns[check] = time.monotonic()

            # Two simultaneous warns → UNTRUSTED
            if len(self._active_warns) >= 2:
                self._transition_to(
                    STATE_UNTRUSTED,
                    reason=f'Multiple simultaneous WARNs: {list(self._active_warns.keys())}'
                )
            else:
                self._transition_to(STATE_DEGRADED,
                                    reason=f'WARN from {check}: {msg.fault_code}')

    # ── Escalation timer ─────────────────────────────────────────────────────

    def _escalation_check_cb(self):
        now = time.monotonic()
        for check, first_warn_time in list(self._active_warns.items()):
            age = now - first_warn_time
            if age > self._escalation_timeout:
                self._transition_to(
                    STATE_UNTRUSTED,
                    reason=(
                        f'WARN from {check} sustained for '
                        f'{age:.1f}s (timeout={self._escalation_timeout}s)'
                    )
                )
                break
        # Heartbeat — publish current state every second so the topic remains live
        self._publish_state(reason='heartbeat')

    # ── State machine ────────────────────────────────────────────────────────

    def _transition_to(self, new_state: str, reason: str):
        """Advance state — only moves toward UNTRUSTED, never back."""
        if _STATE_RANK[new_state] <= _STATE_RANK[self._state]:
            # No downgrade and no no-op transitions generate a log entry
            return

        old_state = self._state
        self._state = new_state
        self.get_logger().warn(
            f'State transition: {old_state} → {new_state} | {reason}'
        )
        self._log_event('state_transition', {
            'from_state': old_state,
            'to_state':   new_state,
            'reason':     reason,
        })
        self._publish_state(reason=reason)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_state(self, reason: str = ''):
        payload = {
            'state':        self._state,
            'reason':       reason,
            'active_warns': list(self._active_warns.keys()),
            'active_crits': list(self._active_crits),
            'timestamp':    self.get_clock().now().nanoseconds,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._state_pub.publish(msg)

    # ── Logging ──────────────────────────────────────────────────────────────

    def _log_event(self, event_type: str, details: dict):
        if self._log_file is None:
            return
        record = {
            'event':     event_type,
            'wall_time': time.time(),
            'ros_time':  self.get_clock().now().nanoseconds,
            **details,
        }
        try:
            self._log_file.write(json.dumps(record) + '\n')
        except OSError as e:
            self.get_logger().error(f'Log write error: {e}')

    def destroy_node(self):
        if self._log_file:
            self._log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IntegrityAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
