#!/usr/bin/env python3
"""
Integrity Aggregator Node — Phase 2.

Subscribes to /poise/integrity_status (poise/IntegrityStatus).
Maintains a recoverable trust state machine:

    TRUSTED ←──────────────────────────────────────────────────────┐
       │                                                            │
       │ any WARN                                                   │
       ▼                                                            │
    DEGRADED ──────────────────────────────────────────────────────┤
       │                                                            │ revalidation
       │ CRITICAL | 2× WARN | non-recoverable WARN > timeout       │ complete
       ▼                                                            │
    UNTRUSTED                                                       │
                                                                    │
    Recoverable path: all faults clear → revalidation period ──────┘
    Non-recoverable path: /poise/reset required (if no active faults)

Escalation rules (unchanged from Phase 1)
------------------------------------------
  Any CRITICAL              → UNTRUSTED immediately
  Any WARN                  → DEGRADED
  Two simultaneous WARNs    → UNTRUSTED
  Non-recoverable WARN sustained > warn_escalation_timeout → UNTRUSTED
    (recoverable WARNs do NOT escalate via timeout — they are environmental)

Recovery rules (Phase 2)
-------------------------
  All active faults are recoverable AND they all clear
      → start re-validation timer (revalidation_period_s, default 10 s)
  No new faults during re-validation period
      → return to TRUSTED
  Any fault occurs during re-validation
      → restart re-validation timer
  Any non-recoverable fault was present
      → no auto-recovery; /poise/reset required

Special case — environmental fault during integrity fault
----------------------------------------------------------
  A recoverable fault arriving while a non-recoverable fault is already active
  → escalate to UNTRUSTED.  Losing cross-check capability simultaneously with
  an integrity violation is suspicious and must not mask the integrity fault.

/poise/reset service (std_srvs/srv/Trigger)
-------------------------------------------
  Accepted only when no fault conditions are currently active.
  Rejected (with explanation) if any check is still in WARN/CRITICAL.
  Every attempt is logged to the JSONL file.

QoS
---
  /poise/integrity_status subscriber : INTEGRITY_QOS (RELIABLE/VOLATILE)
  /poise/system_integrity publisher  : SYSTEM_STATUS_QOS (RELIABLE/TRANSIENT_LOCAL/depth=1)
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from poise.msg import IntegrityStatus
from poise.qos import INTEGRITY_QOS, SYSTEM_STATUS_QOS


# ── State constants ──────────────────────────────────────────────────────────
STATE_TRUSTED   = 'TRUSTED'
STATE_DEGRADED  = 'DEGRADED'
STATE_UNTRUSTED = 'UNTRUSTED'

_STATE_RANK = {STATE_TRUSTED: 0, STATE_DEGRADED: 1, STATE_UNTRUSTED: 2}


class IntegrityAggregator(Node):
    """Aggregates per-check integrity reports into a system-level trust state."""

    def __init__(self):
        super().__init__('integrity_aggregator')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('warn_escalation_timeout_s', 5.0)
        self.declare_parameter('revalidation_period_s',    10.0)
        self.declare_parameter('log_file_path', '/tmp/poise_integrity_log.jsonl')

        self._escalation_timeout   = self.get_parameter('warn_escalation_timeout_s').value
        self._revalidation_period  = self.get_parameter('revalidation_period_s').value
        self._log_path             = self.get_parameter('log_file_path').value

        # ── State ────────────────────────────────────────────────────────────
        self._state = STATE_TRUSTED

        # Startup grace: ignore integrity_status messages for the first 2 s.
        # Prevents stale DDS shared-memory messages from a previous session
        # (which arrive before any live sensor node starts) from poisoning state.
        self._startup_time = time.monotonic()
        self._startup_grace_s = 4.0

        # Per-check fault tracking.
        # Structure: {check_name: {'status': int, 'recoverable': bool,
        #                          'fault_code': str, 'first_warn_time': float}}
        self._active_faults: dict[str, dict] = {}

        # Re-validation
        self._revalidating        = False
        self._revalidation_start  = 0.0

        # Tracks whether any non-recoverable fault has been active since last reset.
        # Prevents auto-recovery when the fault condition clears but the integrity
        # violation was real — operator must explicitly confirm via /poise/reset.
        self._ever_had_nonrecoverable = False

        # Persistent first-seen time per non-recoverable check, reset only by
        # /poise/reset.  Survives fault oscillation so the escalation timer
        # accumulates from the very first occurrence, not the most recent one.
        self._nonrec_first_warn: dict[str, float] = {}

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

        self.create_subscription(
            IntegrityStatus, '/poise/integrity_status',
            self._status_cb, INTEGRITY_QOS
        )

        self._reset_srv = self.create_service(
            Trigger, '/poise/reset', self._reset_cb
        )

        # 1 Hz timer: escalation + revalidation + heartbeat
        self._timer = self.create_timer(1.0, self._periodic_cb)

        # Publish initial state (TRANSIENT_LOCAL means late joiners get this)
        self._publish_state(reason='node_startup')

        self.get_logger().info(
            f'IntegrityAggregator started | '
            f'escalation_timeout={self._escalation_timeout}s | '
            f'revalidation_period={self._revalidation_period}s'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _has_nonrecoverable_fault(self) -> bool:
        return any(not f['recoverable'] for f in self._active_faults.values())

    def _all_faults_recoverable(self) -> bool:
        return bool(self._active_faults) and all(
            f['recoverable'] for f in self._active_faults.values()
        )

    # ── Status callback ───────────────────────────────────────────────────────

    def _status_cb(self, msg: IntegrityStatus):
        check = msg.check_name
        now   = time.monotonic()

        # ── Drop messages during startup grace period ─────────────────────
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
                self._on_faults_changed(now)
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

        # ── Special case: recoverable fault while non-recoverable active ──
        if msg.recoverable and self._has_nonrecoverable_fault():
            # Environmental fault + integrity fault simultaneously → UNTRUSTED
            if check not in self._active_faults:
                self._active_faults[check] = {
                    'status':          msg.status,
                    'recoverable':     msg.recoverable,
                    'fault_code':      msg.fault_code,
                    'first_warn_time': now,
                }
            self._transition_to(
                STATE_UNTRUSTED,
                reason=(
                    f'Recoverable fault {msg.fault_code} on {check} arrived '
                    f'while non-recoverable fault is active — treating as '
                    f'mask attempt; escalating to UNTRUSTED'
                )
            )
            return

        # ── Cancel revalidation on any new fault ──────────────────────────
        if self._revalidating:
            self._revalidating = False
            self.get_logger().info(
                f'Revalidation cancelled by new fault: {msg.fault_code}'
            )

        # ── Record or update fault ────────────────────────────────────────
        is_new = check not in self._active_faults
        if is_new:
            # For non-recoverable checks, preserve the very first occurrence time
            # across oscillation cycles so the escalation timer is not reset.
            if not msg.recoverable and check in self._nonrec_first_warn:
                first_time = self._nonrec_first_warn[check]
            else:
                first_time = now
                if not msg.recoverable:
                    self._nonrec_first_warn[check] = now
            self._active_faults[check] = {
                'status':          msg.status,
                'recoverable':     msg.recoverable,
                'fault_code':      msg.fault_code,
                'first_warn_time': first_time,
            }
        else:
            self._active_faults[check]['status']      = msg.status
            self._active_faults[check]['fault_code']  = msg.fault_code
            self._active_faults[check]['recoverable'] = msg.recoverable

        # ── Apply escalation rules ────────────────────────────────────────
        if msg.status == IntegrityStatus.STATUS_CRITICAL:
            self._transition_to(
                STATE_UNTRUSTED,
                reason=f'CRITICAL from {check}: {msg.fault_code}'
            )

        elif msg.status == IntegrityStatus.STATUS_WARN:
            active_warns = {
                k: v for k, v in self._active_faults.items()
                if v['status'] == IntegrityStatus.STATUS_WARN
            }
            if len(active_warns) >= 2:
                self._transition_to(
                    STATE_UNTRUSTED,
                    reason=(
                        f'Multiple simultaneous WARNs: '
                        f'{list(active_warns.keys())}'
                    )
                )
            else:
                self._transition_to(
                    STATE_DEGRADED,
                    reason=f'WARN from {check}: {msg.fault_code}'
                )

    # ── Periodic callback (1 Hz) ──────────────────────────────────────────────

    def _periodic_cb(self):
        now = time.monotonic()

        # ── Escalation: non-recoverable WARNs only ────────────────────────
        for check, fault in list(self._active_faults.items()):
            if (fault['status'] == IntegrityStatus.STATUS_WARN
                    and not fault['recoverable']):
                age = now - fault['first_warn_time']
                if age > self._escalation_timeout:
                    self._transition_to(
                        STATE_UNTRUSTED,
                        reason=(
                            f'Non-recoverable WARN from {check} '
                            f'sustained {age:.1f}s '
                            f'(timeout={self._escalation_timeout}s)'
                        )
                    )
                    break

        # ── Revalidation timer ────────────────────────────────────────────
        if self._revalidating:
            elapsed = now - self._revalidation_start
            if elapsed >= self._revalidation_period:
                self._revalidating = False
                self._log_event('revalidation_complete', {
                    'elapsed_s': elapsed,
                    'state':     self._state,
                })
                # Return to TRUSTED regardless of which degraded state we're in
                old = self._state
                self._state = STATE_TRUSTED
                self.get_logger().info(
                    f'Revalidation complete after {elapsed:.1f}s — '
                    f'{old} → TRUSTED'
                )
                self._log_event('state_transition', {
                    'from_state': old,
                    'to_state':   STATE_TRUSTED,
                    'reason':     f'revalidation complete after {elapsed:.1f}s',
                })
                self._publish_state(reason='revalidation_complete')
                return

        # Heartbeat
        self._publish_state(reason='heartbeat')

    # ── Fault-clear side-effects ──────────────────────────────────────────────

    def _on_faults_changed(self, now: float):
        """Called when a fault clears.  Start revalidation if eligible."""
        if self._active_faults:
            # Faults still present — no revalidation
            return

        if self._state == STATE_TRUSTED:
            return  # nothing to do

        if self._has_nonrecoverable_fault():
            # Non-recoverable faults must not auto-recover — but we just
            # removed a fault so re-check: _has_nonrecoverable_fault() looks
            # at ACTIVE faults only.  We need to remember if any non-recoverable
            # fault was ever active since last reset.
            pass

        # If we reach here: all faults have cleared.
        # Check if any was non-recoverable (we track this via _ever_had_nonrecoverable)
        if self._ever_had_nonrecoverable:
            self.get_logger().info(
                'All faults cleared but non-recoverable fault was active — '
                'system stays in UNTRUSTED until /poise/reset is called'
            )
            self._publish_state(reason='all_faults_cleared_requires_reset')
            return

        # All-recoverable fault set has cleared → start revalidation
        if not self._revalidating:
            self._revalidating       = True
            self._revalidation_start = now
            self.get_logger().info(
                f'All recoverable faults cleared — '
                f'revalidation timer started ({self._revalidation_period}s)'
            )
            self._log_event('revalidation_started', {
                'period_s': self._revalidation_period,
                'state':    self._state,
            })
            self._publish_state(reason='revalidation_started')

    # ── State machine ─────────────────────────────────────────────────────────

    def _transition_to(self, new_state: str, reason: str):
        """Advance state — cancels revalidation, tracks non-recoverable history."""
        if _STATE_RANK[new_state] <= _STATE_RANK[self._state]:
            return

        # Cancel any revalidation in progress
        if self._revalidating:
            self._revalidating = False

        # Track if any non-recoverable fault caused this transition
        if self._has_nonrecoverable_fault():
            self._ever_had_nonrecoverable = True

        old_state   = self._state
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

    # ── Reset service ─────────────────────────────────────────────────────────

    def _reset_cb(self, request, response):
        """
        /poise/reset — Trigger service.

        Accepted only when no fault conditions are currently active.
        Clears the non-recoverable fault history and returns to TRUSTED.
        """
        now = time.monotonic()

        if self._active_faults:
            active_codes = [f['fault_code'] for f in self._active_faults.values()]
            msg = (
                f'Reset rejected: {len(self._active_faults)} fault(s) still '
                f'active: {active_codes}. Resolve all faults before resetting.'
            )
            self.get_logger().warn(f'Reset REJECTED: {msg}')
            self._log_event('reset_rejected', {
                'reason':       msg,
                'active_faults': active_codes,
                'state':        self._state,
            })
            response.success = False
            response.message = msg
            return response

        # Accept reset
        old_state = self._state
        self._state                   = STATE_TRUSTED
        self._ever_had_nonrecoverable = False
        self._revalidating            = False
        self._nonrec_first_warn.clear()

        msg = f'Reset accepted — {old_state} → TRUSTED (no active faults)'
        self.get_logger().info(msg)
        self._log_event('reset_accepted', {
            'from_state': old_state,
            'to_state':   STATE_TRUSTED,
            'reason':     'operator reset via /poise/reset',
        })
        self._log_event('state_transition', {
            'from_state': old_state,
            'to_state':   STATE_TRUSTED,
            'reason':     'operator reset',
        })
        self._publish_state(reason='operator_reset')

        response.success = True
        response.message = msg
        return response

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish_state(self, reason: str = ''):
        payload = {
            'state':               self._state,
            'reason':              reason,
            'revalidating':        self._revalidating,
            'active_faults':       {
                k: {'fault_code': v['fault_code'], 'recoverable': v['recoverable']}
                for k, v in self._active_faults.items()
            },
            'timestamp':           self.get_clock().now().nanoseconds,
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
