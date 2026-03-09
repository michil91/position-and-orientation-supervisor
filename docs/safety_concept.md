# POISE — Safety Concept Document

**Document ID:** POISE-SC-001
**Phase:** 3
**Status:** Draft
**Revision:** 0.3.0

---

## 1. Purpose and Scope

This document provides the formal safety rationale for the Position and Orientation
Integrity Supervision Engine (POISE), Phase 3.  It covers:

- The problem being monitored (localization integrity)
- The failure modes detected by all three check nodes
- Fault classification: recoverable (environmental) vs. non-recoverable (integrity)
- The detection logic and threshold rationale for each check
- The recoverable trust state machine with revalidation-based auto-recovery
- The `/poise/reset` operator service for non-recoverable fault clearance
- QoS profile design and the `poise.qos` module
- Known limitations of the Phase 3 implementation

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
| IMU calibration violation| Readings outside physical sensor envelope              |
| IMU mount rotation       | Loose bracket, chassis deformation, improper reinstall |

### 2.2 Monitoring Approach

POISE cross-checks independent sensor modalities against each other and against
their physical operating envelopes.  Disagreement between sensors that *should*
agree, or readings that violate physical limits, are evidence of a fault.  The
system cannot always determine *which* sensor is faulty without additional
sources — this is a known limitation (see §6).

---

## 2a. Fault Classification (Phase 3)

Phase 3 adds five new fault codes to the five from Phase 2.  All ten carry an
explicit `recoverable` boolean that drives the recovery policy in
`integrity_aggregator`.

### Phase 2 — GNSS/IMU Cross-Check (`gnss_imu_checker`)

| Fault Code | Category | Recoverable | Rationale |
|---|---|---|---|
| `GNSS_DROPOUT` | Environmental | **Yes** | Signal loss is transient (tunnels, underpasses). |
| `GNSS_HIGH_COVARIANCE` | Environmental | **Yes** | Degraded signal quality is transient. |
| `GNSS_LOW_SATELLITES` | Environmental | **Yes** | Satellite visibility recovers by movement or time. |
| `GNSS_IMU_DIVERGENCE_WARN` | Integrity | **No** | Position disagreement of unknown cause. |
| `GNSS_IMU_DIVERGENCE_CRITICAL` | Integrity | **No** | 3 m divergence exceeds lane-level tolerance. |

### Phase 3 — Calibration Envelope (`calibration_validator`)

| Fault Code | Category | Recoverable | Rationale |
|---|---|---|---|
| `IMU_ACCEL_OUT_OF_ENVELOPE` | Integrity | **No** | Physically impossible reading indicates sensor failure. |
| `IMU_GYRO_OUT_OF_ENVELOPE` | Integrity | **No** | Physically impossible reading indicates sensor failure. |
| `IMU_GRAVITY_OUT_OF_ENVELOPE` | Integrity | **No** | Z-axis gravity outside calibrated range — catastrophic misalignment or sensor failure. |
| `GNSS_OUT_OF_GEOFENCE` | Environmental | **Yes** | Position may re-enter the operating area. |
| `GNSS_FIX_DEGRADED` | Environmental | **Yes** | Fix quality may improve as satellite geometry changes. |

### Phase 3 — Extrinsic Consistency (`extrinsic_validator`)

| Fault Code | Category | Recoverable | Rationale |
|---|---|---|---|
| `IMU_EXTRINSIC_WARN` | Integrity | **No** | Systematic gravity deviation indicates physical mount shift — cannot self-correct. |
| `IMU_EXTRINSIC_CRITICAL` | Integrity | **No** | Significant mount rotation affecting DR accuracy. Operator inspection mandatory. |

---

## 3. Monitored Failure Modes

### 3.1 Check: GNSS/IMU Position Divergence (`gnss_imu_checker`)

**Detection logic:** IMU dead-reckoning via Euler integration, compared against GNSS
position at every fix arrival.  Sliding 60 s DR window prevents unbounded IMU error.

See Phase 2 document for full details — unchanged in Phase 3.

### 3.2 Check: Calibration Envelope (`calibration_validator`)

**Node:** `calibration_validator`
**Topics:** `/sim/imu` (100 Hz), `/sim/gnss` (10 Hz) → `/poise/integrity_status`

#### 3.2.1 IMU Acceleration Envelope (IMU_ACCEL_OUT_OF_ENVELOPE)

**Condition:** `||linear_acceleration|| > imu_max_linear_accel` (default 49.0 m/s²)

The 3-axis acceleration magnitude is compared against the physical sensor limit.
At ~5G (49 m/s²), any MEMS accelerometer would be saturated or physically
destroyed.  A stationary IMU measures only gravity (≈9.81 m/s²); a vehicle in
motion adds centripetal and tangential components, both much smaller than 5G
under normal driving conditions.  Exceeding this threshold is impossible without
sensor failure or an extreme physical event (collision at >50 m/s).

**Severity:** CRITICAL.  Any value above 5G must be treated as evidence of sensor
failure, not a recoverable condition.

#### 3.2.2 IMU Angular Velocity Envelope (IMU_GYRO_OUT_OF_ENVELOPE)

**Condition:** `||angular_velocity|| > imu_max_angular_velocity` (default 10.0 rad/s)

A rapid vehicle U-turn at 30 km/h produces ~1.5 rad/s; a racing vehicle at high
speed might reach ~3 rad/s.  10 rad/s requires the vehicle to complete a full
rotation in 0.6 s — mechanically impossible without catastrophic failure.

**Severity:** CRITICAL.  Indicates sensor failure, electrical interference, or
an extreme physical event incompatible with normal vehicle operation.

#### 3.2.3 IMU Gravity Range (IMU_GRAVITY_OUT_OF_ENVELOPE)

**Condition:** `accel_z < imu_min_accel_z` or `accel_z > imu_max_accel_z`
(defaults: −15.0 to −5.0 m/s²)

A properly mounted and level IMU in Z-down body frame measures gravity as
≈−9.81 m/s² in the Z-axis.  Vehicle pitch and roll shift this measurement:
±30° tilt changes the Z reading by ±4.9 m/s².  The range [−15.0, −5.0] allows
for ±4.8 m/s² of gravitational projection shift (approximately ±30° tilt).

Values outside this range indicate: sensor disconnection (Z reads 0), catastrophic
mounting failure, sensor saturation, or orientation failure in the primary system.

**Severity:** CRITICAL.  The Z-axis gravity reading is used by the extrinsic
validator and by DR integration; corrupted values propagate silently without
this envelope check.

#### 3.2.4 GNSS Geofence (GNSS_OUT_OF_GEOFENCE)

**Condition:** Latitude or longitude outside configured operating area bounding box.

Recoverable because the vehicle may legitimately move outside the geofence
(delivery edge case, route deviation) and then return.  The check is intended
as an early warning for unexpected geographic displacement, not as a hard
safety barrier.

**Severity:** WARN (recoverable).

#### 3.2.5 GNSS Fix Type (GNSS_FIX_DEGRADED)

**Condition:** `NavSatFix.status.status < gnss_min_fix_type`

The NavSatFix `status` field indicates fix quality: STATUS_NO_FIX=−1,
STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2.  A minimum acceptable fix
type is configured at deployment time.  In simulation, STATUS_FIX (0) is
acceptable; a production deployment near infrastructure should require STATUS_SBAS.

**Severity:** WARN (recoverable).

### 3.3 Check: Extrinsic Consistency (`extrinsic_validator`)

**Node:** `extrinsic_validator`
**Topics:** `/sim/imu` (100 Hz), `/sim/vehicle_state` (50 Hz) → `/poise/integrity_status`

#### 3.3.1 Detection Method

The IMU's Z-axis accelerometer reports gravitational acceleration when the
vehicle is stationary and level.  A properly calibrated IMU at rest reads
approximately `expected_gravity_z` = −9.81 m/s² (Z-down body frame convention).

A **rotational shift** in the IMU mounting angle — caused by a loose bracket,
chassis deformation, or improper reinstallation after maintenance — creates a
permanent offset from the expected gravity vector.  This offset is **systematic**
(constant bias) rather than random (noise).

**Why mean over a rolling window, not per-sample:**

Random IMU noise (typical 1σ ≈ 0.02 m/s²) would cause ~3% of per-sample checks
at a 0.06 m/s² threshold to false-alarm.  A systematic mount shift instead
shifts the *mean* of the distribution without increasing its variance.  Averaging
N=100 samples reduces noise by √100 = 10×, achieving:
- Sensitivity to systematic offsets ≥ ~0.05 m/s² (< 0.3° tilt equivalent)
- Expected false-alarm rate: effectively zero (noise-limited to ~0.002 m/s² after averaging)

#### 3.3.2 Standstill Detection

The check accumulates samples only when `|vehicle_velocity| < stationary_velocity_threshold`
(default 0.5 m/s).  Vehicle-induced accelerations in X and Y do not affect Z
(for a level vehicle), but the check gates on velocity as a safety margin to
prevent motion-induced Z disturbances (road bumps, braking) from contaminating
the window.

**Moving → stationary transition:** the sample window is reset so fresh samples
(without motion-induced transients) are used for evaluation.

#### 3.3.3 Threshold Rationale

| Threshold | Value | Equivalent Tilt | Rationale |
|---|---|---|---|
| `gravity_warn_tolerance` | 0.5 m/s² | ~3° | Detectable systematic offset requiring investigation. IMU DR accuracy begins to degrade. |
| `gravity_critical_tolerance` | 1.5 m/s² | ~9° | Significant offset affecting lane-level positioning. DR position error accumulates at >0.5 m/min. |

These tolerances are larger than noise (0.002 m/s² after averaging) but conservatively
smaller than the calibration_validator's gravity Z range (±4.8 m/s²).  A fault
that passes the extrinsic check always also passes the gravity range check.

#### 3.3.4 Non-Recoverability Rationale

A physical mount rotation does not self-correct.  Auto-recovery after the fault
condition clears would allow the system to resume normal operation without any
confirmation that:
- The mount was physically inspected
- The correct mounting orientation was restored
- The extrinsic calibration (sensor-to-vehicle transform) was re-validated

Operator action via `/poise/reset` is mandatory.

**Known limitation:** The extrinsic check assumes level terrain.  A sustained
road slope will produce an apparent gravity deviation.  In production, terrain
gradient from a digital elevation model or barometer should be compensated before
this check runs.  No terrain correction is applied in Phase 3.

---

## 4. Trust State Machine (Phase 3)

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

### 4.3 Recovery Rules (Phase 2/3)

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

**Recoverable (environmental) faults** (GNSS_DROPOUT, GNSS_OUT_OF_GEOFENCE, etc.):
Environmental conditions are transient.  Once the condition clears, a short
revalidation period (10 s) confirms stability before returning to TRUSTED.

**Non-recoverable (integrity) faults** (all IMU faults, GNSS_IMU_DIVERGENCE):
Auto-recovery is unsafe because the root cause is unknown and may persist.
This matches ISO 26262 Part 6 §9.4.3 (fail-safe state persistence) and SOTIF
(ISO 21448) §8.3 (safe state definition).

---

## 5. QoS Profile Design (Phase 3)

All QoS decisions are centralised in `poise/qos.py`.  Phase 3 adds no new profiles.

| Profile | Reliability | Durability | Depth | Used for |
|---|---|---|---|---|
| `SENSOR_QOS` | BEST_EFFORT | VOLATILE | 10 | `/sim/gnss`, `/sim/imu`, `/sim/vehicle_state` |
| `INTEGRITY_QOS` | RELIABLE | VOLATILE | 10 | `/poise/integrity_status` |
| `SYSTEM_STATUS_QOS` | RELIABLE | TRANSIENT_LOCAL | 1 | `/poise/system_integrity` |

`/sim/vehicle_state` uses `SENSOR_QOS` to match Autoware's kinematic_state topic
QoS convention.  A dropped vehicle state packet is acceptable — the extrinsic
validator conservatively holds the last known stationary/moving determination.

---

## 6. Known Limitations of Phase 3

| Limitation | Impact | Planned Phase |
|---|---|---|
| Extrinsic check assumes level terrain | A sustained road slope produces apparent gravity deviation | Phase 4 (terrain compensation) |
| No orientation correction applied | The extrinsic check cannot distinguish mount tilt from vehicle roll/pitch | Phase 4 (IMU orientation state) |
| Extrinsic check is stationary-only | Mount shift during motion is not detectable | Phase 4 (dynamic gravity estimation) |
| Only two localization modalities | Cannot isolate which sensor is faulty | Phase 4 (LiDAR/NDT cross-check) |
| Flat-Earth ENU approximation | Error < 1 mm at < 100 m baseline; acceptable for integrity checking | N/A |
| DR window limited to 60 s | DR position error grows as O(T^1.5); bounded by periodic GNSS realignment | Phase 4 (wheel odometry integration) |
| Single-threaded ROS callbacks | High IMU rate (100 Hz) and check computations compete on the ROS executor | Acceptable for simulation |
| DR anchor accepts drifted GNSS on realign | A deliberately drifted GNSS position becomes the new anchor at t=60 s | Mitigated by fault escalation within the first window |
| Calibration envelope has no temperature model | IMU sensitivity drifts with temperature; static thresholds may be too tight in extreme environments | Phase 4 (temperature-compensated thresholds) |

---

## 7. Relationship to ISO 26262 / SOTIF

POISE Phase 3 is a research/portfolio prototype.  It has **not** been developed
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
5. Extrinsic check validation against a terrain gradient database for the deployment region
