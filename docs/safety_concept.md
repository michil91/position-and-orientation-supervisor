# POISE — Safety Concept Document

**Document ID:** POISE-SC-001
**Phase:** 2
**Status:** Draft
**Revision:** 0.2.0

---

## 1. Purpose and Scope

This document provides the formal safety rationale for the Position and Orientation
Integrity Supervision Engine (POISE), Phase 2.  It covers:

- The problem being monitored (localization integrity)
- The failure modes detected by Phase 2 checks
- Fault classification: recoverable (environmental) vs. non-recoverable (integrity)
- The detection logic and threshold rationale for each check
- The recoverable trust state machine with revalidation-based auto-recovery
- The `/poise/reset` operator service for non-recoverable fault clearance
- QoS profile design and the `poise.qos` module
- Known limitations of the Phase 2 implementation

POISE is a **monitoring layer** — it does not replace the primary localization
system, nor does it issue vehicle control commands.  Its outputs are advisory
and must be consumed by a supervisory controller or safety manager that takes
appropriate action (reduce speed, pull over, alert operator).

---

## 2. Problem Statement

### 2.1 The Localization Integrity Problem

Autonomous vehicle operation requires confident knowledge of the vehicle's
position and orientation.  A localization failure — where the system believes
it is somewhere it is not — may lead to incorrect path planning, obstacle
avoidance failure, or off-road excursions.

Modern AV stacks (e.g., Autoware Universe) typically fuse multiple sensors
(GNSS, IMU, LiDAR NDT, camera odometry) into a single "best estimate."  The
fusion output may, under certain fault conditions, appear smooth and confident
while being significantly in error.  Failures include:

| Failure Mode             | Example Cause                                          |
|--------------------------|--------------------------------------------------------|
| GNSS position jump       | Signal multipath in urban canyons, spoofing            |
| GNSS gradual drift       | Ionospheric error, poor satellite geometry (high DOP)  |
| GNSS dropout             | Tunnel, underpass, heavy foliage                       |
| GNSS covariance mismatch | Receiver reports false confidence                      |
| IMU bias                 | Temperature drift, aging sensor                        |
| IMU spike                | Electrical interference, vibration resonance           |

### 2.2 Monitoring Approach

POISE cross-checks independent sensor modalities against each other.
Disagreement between sensors that *should* agree is evidence of a fault in
at least one of them.  The system cannot determine *which* sensor is faulty
without additional sources — this is a known Phase 1 limitation (see §6).

---

## 2a. Fault Classification (Phase 2)

Phase 2 assigns each fault code an explicit `recoverable` boolean.  This
classification drives the recovery policy in `integrity_aggregator`.

| Fault Code | Category | Recoverable | Rationale |
|---|---|---|---|
| `GNSS_DROPOUT` | Environmental | **Yes** | Signal loss is transient (tunnels, underpasses). GNSS can re-acquire a valid position; the vehicle's true position has not changed. |
| `GNSS_HIGH_COVARIANCE` | Environmental | **Yes** | Degraded signal quality is transient. Once quality improves the covariance self-reports correctly. |
| `GNSS_LOW_SATELLITES` | Environmental | **Yes** | Satellite visibility is geometry-dependent and recoverable by movement or time. |
| `GNSS_IMU_DIVERGENCE_WARN` | Integrity | **No** | The sensors disagree on position. The cause is unknown — it may be GNSS spoofing, IMU bias, or a real displacement. Recovery without root-cause analysis is unsafe. |
| `GNSS_IMU_DIVERGENCE_CRITICAL` | Integrity | **No** | 3 m divergence exceeds lane-level positioning tolerance. Operator confirmation is mandatory. |

---

## 3. Monitored Failure Modes

### 3.1 Check: GNSS/IMU Position Divergence

**Node:** `gnss_imu_checker`
**Topic:** `/poise/integrity_status`

#### 3.1.1 Detection Logic

The IMU is dead-reckoned by Euler integration of linear acceleration:

```
v(t + dt) = v(t) + a(t) · dt
p(t + dt) = p(t) + v(t + dt) · dt
```

At each GNSS arrival, the GNSS-reported position is converted to ENU metres
(flat-Earth approximation, WGS-84 reference ellipsoid) and compared against
the dead-reckoned position:

```
delta = sqrt((E_gnss - E_dr)^2 + (N_gnss - N_dr)^2)
```

**Sliding DR window (dr_realign_window_s, default 60 s):**
DR runs continuously from an anchor point for `dr_realign_window_s` seconds,
then is re-anchored to the current GNSS position.  This design:
- Detects **slow drift** accumulating over the full window duration
- Prevents unbounded IMU integration error (DR position error ≈ O(T^1.5))
- Catches **instantaneous jumps** within a single inter-fix interval

For a drift of 0.05 m/s and a 60 s window, the maximum accumulated divergence
before realignment is ~3 m — near the critical threshold.  A drift rate ≥ 0.05
m/s will raise STATUS_WARN after ~30 s and escalate to UNTRUSTED after a
further `warn_escalation_timeout_s` seconds (default 5 s).

#### 3.1.2 Threshold Rationale

| Threshold       | Default | Rationale                                                  |
|-----------------|---------|------------------------------------------------------------|
| `warn_threshold_m`     | 1.5 m | Exceeds 3σ noise for a 0.5 m stddev GNSS sensor over a 0.1 s dead-reckoning window.  A stationary vehicle should produce < 0.1 m delta. |
| `critical_threshold_m` | 3.0 m | Corresponds to the minimum positioning accuracy required for lane-level operation.  A 3 m error places the vehicle in an adjacent lane or on a kerb. |

These thresholds are configurable.  For production use they should be derived
from a formal HARA (Hazard Analysis and Risk Assessment) per ISO 26262.

### 3.2 Check: GNSS Covariance

**Node:** `gnss_imu_checker`

GNSS receivers report a 3×3 position covariance matrix.  POISE reads the
East and North diagonal entries and compares them against `max_covariance_m2`.

| Threshold            | Default | Rationale                                            |
|----------------------|---------|------------------------------------------------------|
| `max_covariance_m2`  | 25.0 m² | Corresponds to a 5 m 1σ uncertainty — above which positioning is insufficient for safe lane-keeping. |

A covariance check is raised as WARN rather than CRITICAL because a high-
covariance GNSS fix is still a signal (just a less confident one); divergence
from IMU DR is the stronger safety indicator.

---

## 4. Trust State Machine (Phase 2)

### 4.1 States

| State      | Meaning                                                           |
|------------|-------------------------------------------------------------------|
| TRUSTED    | All checks nominal; localization may be used for AV operation.    |
| DEGRADED   | At least one WARN active; caution advised; reduce operational speed. |
| UNTRUSTED  | Localization integrity cannot be assured; AV must stop or hand control to operator. |

### 4.2 Escalation Rules

| Trigger                                                          | Transition              |
|------------------------------------------------------------------|-------------------------|
| Any check reports STATUS_WARN                                    | → DEGRADED              |
| Any check reports STATUS_CRITICAL                                | → UNTRUSTED             |
| Two simultaneous WARN checks                                     | → UNTRUSTED             |
| Non-recoverable WARN sustained > `warn_escalation_timeout_s`    | → UNTRUSTED             |
| Recoverable fault arrives while non-recoverable fault is active  | → UNTRUSTED (masking)   |

Note: recoverable WARNs (environmental faults) do **not** escalate via timeout.
They are expected to be transient; the timeout applies only to integrity faults
that indicate a persistent position disagreement.

### 4.3 Recovery Rules (Phase 2)

```
All recoverable faults clear
    → start revalidation_period_s (10 s) timer
    → no new faults during period
    → return to TRUSTED

Any non-recoverable fault was active (even if now cleared)
    → auto-recovery blocked
    → operator calls /poise/reset (accepted only when no active faults)
    → return to TRUSTED
```

### 4.4 Recovery Rationale

**Recoverable (environmental) faults:**  A GNSS dropout is analogous to
entering a tunnel.  The vehicle's position is not in question — only the
monitoring capability is temporarily reduced.  Once GNSS re-acquires, the
disagreement will be zero (DR is force-realigned on re-acquisition).  A short
revalidation period (10 s) confirms stability before returning to TRUSTED.

**Non-recoverable (integrity) faults:**  A GNSS/IMU divergence means the sensors
disagree on position.  The cause is unknown: it may be GNSS spoofing, gradual
IMU bias, a real displacement event, or sensor failure.  Auto-recovery after
clearing would allow the system to silently resume normal operation without any
confirmation that:
- The root cause was identified
- The position estimate was reconciled
- The fault will not recur

This matches ISO 26262 Part 6 §9.4.3 (fail-safe state persistence) and SOTIF
(ISO 21448) §8.3 (safe state definition).

### 4.5 `/poise/reset` Service

The `/poise/reset` service (`std_srvs/srv/Trigger`) is the operator's explicit
confirmation that the system has been inspected and may return to TRUSTED.

**Acceptance condition:** No fault conditions currently active.
**Effect:** Clears non-recoverable fault history; returns immediately to TRUSTED.
**Rejection:** If any check is still in WARN/CRITICAL, the reset is rejected with
an explanation message listing active fault codes.

This design ensures the operator cannot reset into a state where a fault is still
actively triggering — the root cause must be resolved first.

---

## 5. QoS Profile Design (Phase 2)

Phase 2 defines all QoS decisions in `poise/qos.py` to ensure consistency
across nodes and Autoware compatibility.

| Profile | Reliability | Durability | Depth | Used for |
|---|---|---|---|---|
| `SENSOR_QOS` | BEST_EFFORT | VOLATILE | 10 | `/sim/gnss`, `/sim/imu` |
| `INTEGRITY_QOS` | RELIABLE | VOLATILE | 10 | `/poise/integrity_status` |
| `SYSTEM_STATUS_QOS` | RELIABLE | TRANSIENT_LOCAL | 1 | `/poise/system_integrity` |

**SENSOR_QOS:** Matches Autoware sensor topic QoS.  Losing an occasional IMU or
GNSS message is acceptable; the DR integration will skip samples with bad
timestamps.

**INTEGRITY_QOS:** Fault reports must not be lost — every WARN and CRITICAL must
reach the aggregator.  VOLATILE means no late-joiner delivery (a new subscriber
does not receive historical fault messages from before it subscribed).

**SYSTEM_STATUS_QOS:** TRANSIENT_LOCAL with depth=1 means a late-joining
subscriber (e.g. a newly started safety manager) immediately receives the most
recent system integrity state without needing to wait for the next heartbeat.

---

## 6. Known Limitations of Phase 2

| Limitation                          | Impact                                         | Planned Phase |
|-------------------------------------|------------------------------------------------|---------------|
| Only two sensor modalities checked  | Cannot isolate faulty sensor; a faulty IMU looks identical to a faulty GNSS | Phase 3 (LiDAR/NDT) |
| Flat-Earth ENU approximation        | Error < 1 mm at < 100 m baseline; acceptable for integrity checking at urban speeds | N/A |
| DR window limited to 60 s | DR position error grows as O(T^1.5); beyond ~90 s IMU noise exceeds warn threshold | Phase 3 (wheel odometry to bound DR error) |
| No heading/orientation cross-check  | IMU angular velocity vs. GNSS course-over-ground not compared | Phase 3 |
| No time-synchronisation enforcement | GNSS and IMU messages correlated by arrival time, not sensor timestamp | Phase 3 (hardware timestamps, synchronised clocks) |
| Single-threaded ROS callbacks       | High IMU rate (100 Hz) and check computations compete on the ROS executor | Acceptable for simulation |
| DR anchor accepts drifted GNSS on realign | A deliberately drifted GNSS position becomes the new anchor at t=60 s; a second fault cycle could hide the cumulative error | Mitigated by fault escalation within the first window |

---

## 7. Relationship to ISO 26262 / SOTIF

POISE Phase 1 is a research/portfolio prototype.  It has **not** been developed
under a certified ISO 26262 process.  The design concepts are informed by:

- ISO 26262-6: Detection and mitigation of systematic faults in software
- ISO 21448 (SOTIF): Monitoring of sensor performance envelopes
- MISRA C:2012 principles applied to Python (fail-safe defaults, no implicit conversions)
- Autoware Foundation ADS safety architecture

A production deployment would require:
1. Formal HARA to derive ASIL levels for the localization monitoring function
2. Validated threshold derivation from sensor characterisation data
3. Systematic test coverage (MC/DC) per ASIL-B or higher requirements
4. Review of the no-recovery policy against the specific operational design domain
