#!/usr/bin/env python3
"""
Integrity Aggregator Node — Phase 5.

Subscribes to /poise/integrity_status (poise/IntegrityStatus).
Maintains a four-state trust state machine:

    TRUSTED ←──────────────────────────────────────────────────────────────┐
       │                                                                    │
       │ environmental fault(s) only                                        │
       ▼                                                                    │ revalidation
    ENVIRONMENT DEGRADED ──────────────────────────────────────────────────┤ complete
       │                                                                    │
       │ integrity fault active                                             │
       ▼                                                                    │
    SYSTEM DEGRADED ───────────────────────────────────────────────────────┤
       │                                                                    │ /poise/reset
       │ CRITICAL  OR  sustained > warn_escalation_timeout_s (60 s)        │ (after all faults
       ▼                                                                    │  have cleared)
    UNTRUSTED ─────────────────────────────────────────────────────────────┘

State determination (§4.2 — evaluated top to bottom each cycle)
---------------------------------------------------------------
  1. Any CRITICAL severity fault         → UNTRUSTED (immediate)
  2. SYSTEM DEGRADED sustained > 60 s   → UNTRUSTED
  3. Any INTEGRITY-category fault active → SYSTEM DEGRADED
  4. Any ENVIRONMENTAL fault active,
     no INTEGRITY fault active           → ENVIRONMENT DEGRADED
  5. No active faults                    → TRUSTED

Fault categories
----------------
  Environmental: GNSS_DROPOUT, GNSS_HIGH_COVARIANCE, GNSS_LOW_SATELLITES,
                 ODOM_DROPOUT
  Integrity: all other fault codes

Recovery rules (§4.3)
---------------------
  From ENVIRONMENT DEGRADED:
    All env faults clear → revalidation_period_s (10 s) → TRUSTED (auto)

  From SYSTEM DEGRADED / UNTRUSTED:
    Integrity fault clears, env fault remains → ENVIRONMENT DEGRADED (auto)
    All faults clear → /poise/reset required
      → reset accepted → revalidation_period_s (10 s) → TRUSTED

  /poise/reset rejected if ANY fault active (env or integrity).
  New fault during revalidation cancels the revalidation timer.

QoS
---
  /poise/integrity_status subscriber : INTEGRITY_QOS (RELIABLE/VOLATILE)
  /poise/system_integrity publisher  : SYSTEM_STATUS_QOS (RELIABLE/TRANSIENT_LOCAL/depth=1)
  /poise/heartbeat publisher         : SYSTEM_STATUS_QOS
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger
from poise.msg import IntegrityStatus
from poise.qos import INTEGRITY_QOS, SYSTEM_STATUS_QOS


# ── State constants ──────────────────────────────────────────────────────────
STATE_TRUSTED      = 'TRUSTED'
STATE_ENV_DEGRADED = 'ENVIRONMENT DEGRADED'
STATE_SYS_DEGRADED = 'SYSTEM DEGRADED'
STATE_UNTRUSTED    = 'UNTRUSTED'

_STATE_RANK = {
    STATE_TRUSTED:      0,
    STATE_ENV_DEGRADED: 1,
    STATE_SYS_DEGRADED: 2,
    STATE_UNTRUSTED:    3,
}

# Fault codes classified as Environmental; all others are Integrity.
_ENVIRONMENTAL_FAULTS = frozenset({
    'GNSS_DROPOUT',
    'GNSS_HIGH_COVARIANCE',
    'GNSS_LOW_SATELLITES',
    'ODOM_DROPOUT',
})


class IntegrityAggregator(Node):
    """Aggregates per-check integrity reports into a four-state trust signal."""

    def __init__(self):
        super().__init__('integrity_aggregator')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('warn_escalation_timeout_s', 60.0)
        self.declare_parameter('revalidation_period_s',     10.0)
        self.declare_parameter('log_file_path', '/tmp/poise_integrity_log.jsonl')
        self.declare_parameter('heartbeat_rate_hz', 1.0)

        self._escalation_timeout  = self.get_parameter('warn_escalation_timeout_s').value
        self._revalidation_period = self.get_parameter('revalidation_period_s').value
        self._log_path            = self.get_parameter('log_file_path').value
        self._heartbeat_rate      = self.get_parameter('heartbeat_rate_hz').value

        # ── State ────────────────────────────────────────────────────────────
        self._state = STATE_TRUSTED

        # Startup grace: ignore integrity_status messages for the first N seconds.
        # Prevents stale DDS shared-memory messages from a previous session
        # from poisoning state before any live sensor node has started.
        self._startup_time    = time.monotonic()
        self._startup_grace_s = 4.0

        # Per-check fault tracking.
        # Structure: {check_name: {'status': int, 'recoverable': bool,
        #                          'fault_code': str, 'first_seen': float}}
        self._active_faults: dict[str, dict] = {}

        # True when an integrity fault has been active since the last reset.
        # Blocks automatic recovery; cleared by /poise/reset or the transitional
        # SYSTEM DEGRADED → ENVIRONMENT DEGRADED path.
        self._awaiting_reset = False

        # Monotonic timestamp of when SYSTEM DEGRADED was first entered.
        # Used to drive the 60-s escalation to UNTRUSTED.
        # None whenever the system is not in SYSTEM DEGRADED.
        self._sys_degraded_since: float | None = None

        # Revalidation state — transition delay before returning to TRUSTED.
        self._revalidating       = False
        self._revalidation_start = 0.0

        # ── Log file ─────────────────────────────────────────────────────────
        try:
            self._log_file = open(self._log_path, 'a', buffering=1)
            self.get_logger().info(f'Integrity log: {self._log_path}')
        except OSError as e:
            self.get_logger().error(f'Cannot open log file {self._log_path}: {e}')
            self._log_file = None

        # ── Publisher / Subscriber / Service ─────────────────────────────────
        self._state_pub = self.create_publisher(
            String, '/poise/system_integrity', SYSTEM_STATUS_QOS
        )
        self._heartbeat_pub = self.create_publisher(
            Header, '/poise/heartbeat', SYSTEM_STATUS_QOS
        )
        self.create_subscription(
            IntegrityStatus, '/poise/integrity_status',
            self._status_cb, INTEGRITY_QOS
        )
        self._reset_srv = self.create_service(
            Trigger, '/poise/reset', self._reset_cb
        )

        # 1 Hz timer: escalation check + revalidation + heartbeat publish
        self._timer = self.create_timer(1.0, self._periodic_cb)

        # Configurable-rate liveness heartbeat for external watchdog
        heartbeat_period = 1.0 / self._heartbeat_rate
        self._heartbeat_timer = self.create_timer(heartbeat_period, self._heartbeat_cb)

        # Publish initial state (TRANSIENT_LOCAL — late joiners receive this)
        self._publish_state(reason='node_startup')

        self.get_logger().info(
            f'IntegrityAggregator started | '
            f'escalation_timeout={self._escalation_timeout}s | '
            f'revalidation_period={self._revalidation_period}s'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _is_environmental(fault_code: str) -> bool:
        return fault_code in _ENVIRONMENTAL_FAULTS

    def _cancel_revalidation(self):
        if self._revalidating:
            self._revalidating = False
            self.get_logger().info('Revalidation cancelled')

    def _set_state(self, new_state: str, reason: str):
        """Transition to new_state.  Tracks _sys_degraded_since; logs and publishes."""
        if new_state == self._state:
            return
        old_state = self._state

        # Track SYSTEM DEGRADED entry time for the 60-s escalation timer.
        if new_state == STATE_SYS_DEGRADED and old_state != STATE_SYS_DEGRADED:
            self._sys_degraded_since = time.monotonic()
        elif new_state != STATE_SYS_DEGRADED:
            self._sys_degraded_since = None

        self._state = new_state
        self.get_logger().warn(f'State: {old_state} → {new_state} | {reason}')
        self._log_event('state_transition', {
            'from_state': old_state,
            'to_state':   new_state,
            'reason':     reason,
        })
        self._publish_state(reason=reason)

    # ── State recomputation ───────────────────────────────────────────────────

    def _apply_state(self, now: float):
        """Recompute trust state from the current fault set and apply any transitions.

        Called after every fault change (fault added or cleared).
        Priority order follows §4.2 of the POISE safety concept.
        """
        has_critical = any(
            f['status'] == IntegrityStatus.STATUS_CRITICAL
            for f in self._active_faults.values()
        )
        has_integrity = any(
            not self._is_environmental(f['fault_code'])
            for f in self._active_faults.values()
        )
        has_env = any(
            self._is_environmental(f['fault_code'])
            for f in self._active_faults.values()
        )

        # ── Rule 1: Any CRITICAL → UNTRUSTED immediately ──────────────────
        if has_critical:
            self._awaiting_reset = True
            self._cancel_revalidation()
            self._set_state(STATE_UNTRUSTED, 'CRITICAL fault active')
            return

        # ── Rule 2: SYSTEM DEGRADED escalation ────────────────────────────
        # Handled in _periodic_cb so the 60-s timeout fires on schedule
        # regardless of whether a new fault arrives.

        # ── Rule 3: Active integrity fault → SYSTEM DEGRADED ─────────────
        if has_integrity:
            self._awaiting_reset = True
            self._cancel_revalidation()
            self._set_state(STATE_SYS_DEGRADED, 'integrity fault active')
            return

        # Past here: no CRITICAL, no integrity faults currently active.

        # ── Awaiting reset ────────────────────────────────────────────────
        if self._awaiting_reset:
            if has_env:
                # Transitional path (§4.3): integrity fault has cleared but
                # environmental fault remains.  Drop to ENVIRONMENT DEGRADED
                # and clear the reset flag — the env-only recovery path applies.
                self._awaiting_reset = False
                self._cancel_revalidation()
                self._set_state(
                    STATE_ENV_DEGRADED,
                    'integrity fault cleared; environmental fault remains',
                )
            else:
                # All faults have cleared, but an integrity fault was present.
                # Operator must call /poise/reset before returning to TRUSTED.
                self._publish_state(reason='all_faults_cleared_awaiting_reset')
            return

        # ── Rule 4: Environmental faults only ─────────────────────────────
        if has_env:
            # Cancel any revalidation in progress — new fault resets the clock.
            self._cancel_revalidation()
            self._set_state(STATE_ENV_DEGRADED, 'environmental fault active')
            return

        # ── Rule 5: No active faults, no reset required ───────────────────
        if self._state == STATE_TRUSTED:
            return  # already nominal

        # Start or restart revalidation towards TRUSTED.
        if self._revalidating:
            # New fault arrived and cleared during revalidation — restart timer.
            self._revalidation_start = now
            self.get_logger().info('Revalidation timer restarted')
        else:
            self._revalidating       = True
            self._revalidation_start = now
            self.get_logger().info(
                f'Revalidation started ({self._revalidation_period}s)'
            )
            self._log_event('revalidation_started', {
                'period_s': self._revalidation_period,
                'state':    self._state,
            })
            self._publish_state(reason='revalidation_started')

    # ── Heartbeat callback ────────────────────────────────────────────────────

    def _heartbeat_cb(self):
        """Publish a liveness heartbeat for an external watchdog.

        Published unconditionally regardless of trust state.  Silence on this
        topic indicates a POISE process failure detectable by any subscriber
        monitoring the publishing interval.
        """
        msg = Header()
        msg.stamp    = self.get_clock().now().to_msg()
        msg.frame_id = 'poise'
        self._heartbeat_pub.publish(msg)

    # ── Status callback ───────────────────────────────────────────────────────

    def _status_cb(self, msg: IntegrityStatus):
        check = msg.check_name
        now   = time.monotonic()

        # Drop messages during the startup grace period to avoid stale DDS
        # messages from a previous session poisoning the initial state.
        if now - self._startup_time < self._startup_grace_s:
            return

        # ── STATUS_OK: clear this check ──────────────────────────────────
        if msg.status == IntegrityStatus.STATUS_OK:
            if check in self._active_faults:
                self._log_event('fault_cleared', {
                    'check_name': check,
                    'fault_code': self._active_faults[check]['fault_code'],
                    'state':      self._state,
                })
                del self._active_faults[check]
                self._apply_state(now)
            return

        # ── Log non-OK event ─────────────────────────────────────────────
        self._log_event('fault_detected', {
            'check_name':   check,
            'status':       msg.status,
            'fault_code':   msg.fault_code,
            'recoverable':  msg.recoverable,
            'description':  msg.description,
            'measured':     msg.measured_value,
            'threshold':    msg.threshold_exceeded,
            'units':        msg.units,
            'system_state': self._state,
        })

        # ── Record or update fault ────────────────────────────────────────
        if check not in self._active_faults:
            self._active_faults[check] = {
                'status':     msg.status,
                'recoverable': msg.recoverable,
                'fault_code': msg.fault_code,
                'first_seen': now,
            }
        else:
            self._active_faults[check]['status']     = msg.status
            self._active_faults[check]['fault_code'] = msg.fault_code

        self._apply_state(now)

    # ── Periodic callback (1 Hz) ──────────────────────────────────────────────

    def _periodic_cb(self):
        now = time.monotonic()

        # ── Rule 2: SYSTEM DEGRADED escalation ────────────────────────────
        if self._state == STATE_SYS_DEGRADED and self._sys_degraded_since is not None:
            age = now - self._sys_degraded_since
            if age > self._escalation_timeout:
                reason = f'SYSTEM DEGRADED escalation after {age:.0f}s'
                self._cancel_revalidation()
                self._set_state(STATE_UNTRUSTED, reason)

        # ── Revalidation timer ────────────────────────────────────────────
        if self._revalidating:
            elapsed = now - self._revalidation_start
            if elapsed >= self._revalidation_period:
                self._revalidating = False
                old = self._state
                self._state = STATE_TRUSTED
                reason = f'revalidation complete after {elapsed:.1f}s'
                self.get_logger().info(f'State: {old} → TRUSTED | {reason}')
                self._log_event('revalidation_complete', {
                    'elapsed_s': elapsed,
                    'state':     old,
                })
                self._log_event('state_transition', {
                    'from_state': old,
                    'to_state':   STATE_TRUSTED,
                    'reason':     reason,
                })
                self._publish_state(reason='revalidation_complete')
                return

        # ── 1 Hz heartbeat ────────────────────────────────────────────────
        self._publish_state(reason='heartbeat')

    # ── Reset service ─────────────────────────────────────────────────────────

    def _reset_cb(self, request, response):
        """
        /poise/reset — Trigger service.

        Accepted only when no fault conditions are currently active (env or
        integrity).  Starts the revalidation period; system returns to TRUSTED
        if no new faults occur within revalidation_period_s.
        """
        now = time.monotonic()

        # Reject if any fault is active
        if self._active_faults:
            active_codes = [f['fault_code'] for f in self._active_faults.values()]
            msg = (
                f'Reset rejected: {len(self._active_faults)} fault(s) still active: '
                f'{active_codes}. Resolve all faults before resetting.'
            )
            self.get_logger().warn(f'Reset REJECTED: {msg}')
            self._log_event('reset_rejected', {
                'reason':        msg,
                'active_faults': active_codes,
                'state':         self._state,
            })
            response.success = False
            response.message  = msg
            return response

        # Already trusted and nothing to reset
        if self._state == STATE_TRUSTED and not self._awaiting_reset:
            response.success = True
            response.message  = 'System already TRUSTED — no reset required'
            return response

        # Accept reset — start revalidation
        old_state            = self._state
        self._awaiting_reset = False
        self._revalidating       = True
        self._revalidation_start = now
        self._sys_degraded_since = None

        reason = f'operator reset — {old_state} → revalidation ({self._revalidation_period}s)'
        self.get_logger().info(f'Reset accepted: {reason}')
        self._log_event('reset_accepted', {
            'from_state': old_state,
            'reason':     'operator reset via /poise/reset',
        })
        self._log_event('state_transition', {
            'from_state': old_state,
            'to_state':   STATE_TRUSTED,
            'reason':     f'operator reset (revalidation {self._revalidation_period}s pending)',
        })
        self._publish_state(reason='revalidation_started')

        response.success = True
        response.message  = (
            f'Reset accepted — revalidation period {self._revalidation_period}s started. '
            f'System will return to TRUSTED if no new faults occur.'
        )
        return response

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish_state(self, reason: str = ''):
        payload = {
            'state':         self._state,
            'reason':        reason,
            'revalidating':  self._revalidating,
            'active_faults': {
                k: {'fault_code': v['fault_code'], 'recoverable': v['recoverable']}
                for k, v in self._active_faults.items()
            },
            'timestamp':     self.get_clock().now().nanoseconds,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._state_pub.publish(msg)

    # ── Logger ────────────────────────────────────────────────────────────────

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
