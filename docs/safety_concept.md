# POISE — Safety Concept Document

**Document ID:** POISE-SC-001
**Phase:** 1
**Status:** Draft
**Revision:** 0.1.0

---

## 1. Purpose and Scope

This document provides the formal safety rationale for the Position and Orientation
Integrity Supervision Engine (POISE), Phase 1.  It covers:

- The problem being monitored (localization integrity)
- The failure modes detected by Phase 1 checks
- The detection logic and threshold rationale for each check
- The trust state machine design decisions, including no-auto-recovery
- Known limitations of the Phase 1 implementation

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

## 4. Trust State Machine

### 4.1 States

```
TRUSTED ──WARN──→ DEGRADED ──CRITICAL/timeout──→ UNTRUSTED
         ←─────────────────────────────────────── (never)
```

| State      | Meaning                                                           |
|------------|-------------------------------------------------------------------|
| TRUSTED    | All checks nominal; localization may be used for AV operation.    |
| DEGRADED   | At least one WARN active; caution advised; reduce operational speed. |
| UNTRUSTED  | Localization integrity cannot be assured; AV must stop or hand control to operator. |

### 4.2 Transition Rules

| Trigger                                         | Transition            |
|-------------------------------------------------|-----------------------|
| Any check reports STATUS_WARN                   | TRUSTED → DEGRADED    |
| Any check reports STATUS_CRITICAL               | Any → UNTRUSTED       |
| Two simultaneous WARN checks                    | Any → UNTRUSTED       |
| Single WARN sustained > `warn_escalation_timeout_s` | DEGRADED → UNTRUSTED |

### 4.3 No-Auto-Recovery Rationale

**POISE does not automatically recover from DEGRADED or UNTRUSTED.**

This is a deliberate safety design decision, not a simplification.

An AV that automatically restores "TRUSTED" state after a sensor recovers
may re-engage autonomous driving in a situation where:
- The fault was intermittent and will recur
- The position estimate drifted during the fault and has not been corrected
- The root cause has not been identified or remediated

Autonomous recovery from a safety-critical fault without operator confirmation
or a validated re-initialisation procedure introduces unquantified risk.
Conservative design requires that recovery is an explicit act (e.g., operator
reset, successful GNSS re-acquisition with position reconciliation).

This matches the design philosophy of ISO 26262 Part 6 §9.4.3 (fail-safe state
persistence) and SOTIF (ISO 21448) §8.3 (safe state definition).

---

## 5. Known Limitations of Phase 1

| Limitation                          | Impact                                         | Planned Phase |
|-------------------------------------|------------------------------------------------|---------------|
| Only two sensor modalities checked  | Cannot isolate faulty sensor; a faulty IMU looks identical to a faulty GNSS | Phase 2 (LiDAR/NDT) |
| Flat-Earth ENU approximation        | Error < 1 mm at < 100 m baseline; acceptable for integrity checking at urban speeds | N/A |
| DR window limited to 60 s | DR position error grows as O(T^1.5); beyond ~90 s IMU noise exceeds warn threshold | Phase 2 (sensor fusion with wheel odometry to bound DR error) |
| No heading/orientation cross-check  | IMU angular velocity vs. GNSS course-over-ground not compared | Phase 2 |
| No time-synchronisation enforcement | GNSS and IMU messages correlated by arrival time, not sensor timestamp | Phase 2 (hardware timestamps, synchronised clocks) |
| Single-threaded ROS callbacks       | High IMU rate (100 Hz) and check computations compete on the ROS executor | Acceptable for Phase 1 simulation |
| No GNSS sat-count/HDOP in NavSatFix | These fields are not in the standard ROS message; POISE cannot apply sat/HDOP gates without a custom bridge | Phase 2 (Autoware GNSS message bridge) |

---

## 6. Relationship to ISO 26262 / SOTIF

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
