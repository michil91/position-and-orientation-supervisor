# POISE — Safety Concept Document

**Document ID:** POISE-SC-001
**Status:** Draft
**Revision:** 0.4.0

---

## 1. Purpose and Scope

This document provides the formal safety rationale for the Position and Orientation
Integrity Supervision Engine (POISE). 

Contents:
- Problem Statement
- Failure Modes & Fault Classification
- Trust State Machine
- QoS Profile
- Known Limitations

POISE is a **monitoring layer** — it does not replace the primary localization
system, nor does it issue vehicle control commands.  Its outputs are advisory
and intended to be consumed by a supervisory controller or safety manager that takes
appropriate action (reduce speed, pull over, alert operator).

---

## 2. Problem Statement

### 2.1 The Localization Integrity Problem

Autonomous vehicle operation requires confident knowledge of the vehicle's
position and orientation.  A localization failure — where the system believes
it is somewhere it is not — may lead to incorrect path planning, obstacle
avoidance failure, or other unsafe maneuvers.

Modern AV stacks (e.g., Autoware Universe) fuse multiple sensors (GNSS, IMU,
LiDAR NDT, wheel odometry) into a single pose estimate. Sensor fusion provides
substantial protection against many individual sensor failures — sudden jumps,
dropouts, and spikes are typically detected through innovation monitoring and
the EKF falls back to remaining sources automatically.

However, a specific class of failures can corrupt the fused output while
evading the EKF's internal consistency checks. These failures share a common
characteristic: they are **systematic and gradual**, meaning each individual
measurement update appears plausible given the previous state estimate, so no
single update triggers rejection. Over time the EKF's internal reference drifts
alongside the corruption, and its own covariance estimate no longer reflects the
true position error.

POISE targets this specific failure class by validating raw sensor inputs
before they enter the fusion pipeline. By cross-checking independent sources
directly against each other, POISE can detect disagreements that the EKF misses
because it never uses the EKF's internal state as a reference.

| Failure Mode | EKF Protection | POISE Target | Rationale |
|---|---|---|---|
| GNSS position jump | Strong — large innovation rejected | Marginal | POISE adds explicit fault code only |
| GNSS gradual drift | Weak — small innovations accepted, filter diverges | **Yes** | Primary target of odometry cross-check |
| GNSS dropout | Strong — missing input detected explicitly | Marginal | POISE adds explicit fault code only |
| GNSS covariance mismatch | Partial — EKF over-weights GNSS | **Yes** | POISE checks raw covariance before fusion |
| IMU bias (severe) | Partial — EKF compensates within modelled range | **Yes** | POISE detects via GNSS/IMU divergence |
| IMU spike | Strong — large innovation rejected | No | Handled well by EKF |
| IMU calibration violation | Partial | Marginal | POISE adds pre-fusion envelope check |
| IMU mount rotation | None — systematic error, not random | **Yes** | Primary target of extrinsic validator |
| Wheel odometry slip | Weak — mild slip within noise model | **Yes** | Primary target of odometry checker |
| Wheel odometry dropout | Strong — missing input detected | Marginal | POISE adds explicit fault code only |

The failures marked as primary targets share a common property: they are either
systematic (producing no large innovation) or gradual (individually plausible
updates that corrupt the EKF state over time). These are the conditions under
which sensor fusion provides the least protection and an independent monitoring
layer provides the most value.

### 2.2 Monitoring Approach

POISE cross-checks independent sensor modalities against each other and against
their physical operating envelopes.  Disagreement between sensors that should
agree, or readings that violate physical limits, are evidence of a fault.  The
system cannot always determine which sensor is faulty without additional
sources — this is a known limitation (see detailed section).

---

## 3. Failure Modes & Fault Classification

### 3.1 Fault Codes

Each fault code carries a **category** and a **recoverable** flag. The category
determines which trust state is entered when the fault is active. The recoverable
flag determines whether the system can return to TRUSTED automatically or whether
operator action is required.

**Category definitions:**

- **Environmental** — the fault condition is caused by an external, transient
  factor (signal occlusion, poor satellite geometry, topic silence). The sensor
  hardware is functioning correctly. The condition is expected to clear without
  operator intervention.
- **Integrity** — the fault condition indicates a potential sensor hardware
  failure, systematic calibration error, or localization source disagreement of
  unknown cause. The condition cannot be assumed to self-correct. Operator
  inspection is required before normal operation resumes.

#### GNSS/IMU Cross-Check (`gnss_imu_checker`)

| Fault Code | Category | Recoverable | Rationale |
|---|---|---|---|
| `GNSS_DROPOUT` | Environmental | **Yes** | Signal loss is transient (tunnels, underpasses). |
| `GNSS_HIGH_COVARIANCE` | Environmental | **Yes** | Degraded signal quality is transient. |
| `GNSS_LOW_SATELLITES` | Environmental | **Yes** | Satellite visibility recovers by movement or time. |
| `GNSS_IMU_DIVERGENCE_WARN` | Integrity | **No** | Position disagreement of unknown cause. |
| `GNSS_IMU_DIVERGENCE_CRITICAL` | Integrity | **No** | 3 m divergence exceeds lane-level tolerance. |

#### Calibration Envelope (`calibration_validator`)

| Fault Code | Category | Recoverable | Rationale |
|---|---|---|---|
| `IMU_ACCEL_OUT_OF_ENVELOPE` | Integrity | **No** | Physically impossible reading indicates sensor failure. |
| `IMU_GYRO_OUT_OF_ENVELOPE` | Integrity | **No** | Physically impossible reading indicates sensor failure. |
| `IMU_GRAVITY_OUT_OF_ENVELOPE` | Integrity | **No** | Z-axis gravity outside calibrated range — catastrophic misalignment or sensor failure. |

#### Extrinsic Consistency (`extrinsic_validator`)

| Fault Code | Category | Recoverable | Rationale |
|---|---|---|---|
| `IMU_EXTRINSIC_WARN` | Integrity | **No** | Systematic gravity deviation indicates physical mount shift — cannot self-correct. |
| `IMU_EXTRINSIC_CRITICAL` | Integrity | **No** | Significant mount rotation affecting DR accuracy. Operator inspection mandatory. |

#### Wheel Odometry Cross-Check (`odometry_checker`)

| Fault Code | Category | Recoverable | Rationale |
|---|---|---|---|
| `ODOM_GNSS_DIVERGENCE_WARN` | Integrity | **No** | Path-length disagreement between wheel odometry and GNSS of unknown cause. |
| `ODOM_GNSS_DIVERGENCE_CRITICAL` | Integrity | **No** | Divergence exceeds 3 m — exceeds lane-level positioning tolerance. |
| `ODOM_DROPOUT` | Environmental | **Yes** | Odometry topic silence is transient; clears automatically on resumption. |
| `ODOM_SLIP_SUSPECTED` | Integrity | **No** | Systematic understatement of GNSS path length indicates persistent wheel slip; cannot self-correct without operator inspection. |

---

### 3.2 Monitored Failure Modes

#### 3.2.1 Check: GNSS/IMU Position Divergence (`gnss_imu_checker`)

**Node:** `gnss_imu_checker`
**Topics:** `/sim/gnss` (10 Hz), `/sim/imu` (100 Hz) → `/poise/integrity_status`

##### 3.2.1.1 Detection Method

At each GNSS fix the node compares two displacement magnitudes, both measured
from the same sliding-window anchor point:

- **GNSS displacement** — Euclidean ENU distance between the anchor geodetic
  position and the current GNSS position.
- **DR displacement** — IMU dead-reckoning position, obtained by double-integrating
  linear accelerations (Euler method) from the anchor time.

The comparison is `|gnss_displacement − dr_displacement|`.  Agreement within
threshold indicates both sensors are consistent; divergence indicates at least
one sensor is faulty.

##### 3.2.1.2 Sliding DR Window

IMU dead-reckoning position error grows as O(T^1.5) due to velocity error
accumulating under acceleration integration.  An unbounded DR window would
produce false divergence alarms purely from IMU noise.

The DR window is fixed at 60 s (configurable via `dr_realign_window_s`).  At
each 60 s boundary the anchor is realigned to the current GNSS position and DR
state is reset.  This bounds the worst-case DR error to approximately
`σ_acc × (60)^1.5 ≈ 9 m` at nominal IMU noise, well above the 1.5 m warn
threshold for genuine faults, while keeping the window long enough to detect
gradual drift.

The IMU guard prevents a false alarm on the first GNSS fix after realignment:
the check is skipped unless at least one IMU sample has been integrated since
the anchor was set.

##### 3.2.1.3 GNSS Quality Gates

Before running the cross-check, each GNSS fix is validated against three
independent quality criteria:

| Check | Condition | Fault Code |
|---|---|---|
| Dropout | No fix received for > `gnss_dropout_timeout_s` (2 s) | `GNSS_DROPOUT` |
| Covariance | Any diagonal entry > `max_covariance_m2` (25 m²) | `GNSS_HIGH_COVARIANCE` |
| Satellites | `num_satellites < min_satellites` (6) | `GNSS_LOW_SATELLITES` |

These are classified **recoverable** because the conditions are environmental
and transient.  A GNSS dropout followed by re-acquisition does not by itself
indicate an integrity violation.

##### 3.2.1.4 Divergence Thresholds

| Fault Code | Threshold | Severity |
|---|---|---|
| `GNSS_IMU_DIVERGENCE_WARN` | delta > 1.5 m | WARN (non-recoverable) |
| `GNSS_IMU_DIVERGENCE_CRITICAL` | delta > 3.0 m | CRITICAL (non-recoverable) |

3 m corresponds to approximately one lane width.  Any divergence exceeding
this value is incompatible with safe lane-level positioning and must require
operator action before normal operation resumes.

### 3.2.2 Check: Calibration Envelope (`calibration_validator`)

**Node:** `calibration_validator`
**Topics:** `/sim/imu` (100 Hz), `/sim/gnss` (10 Hz) → `/poise/integrity_status`

#### 3.2.2.1 IMU Acceleration Envelope (IMU_ACCEL_OUT_OF_ENVELOPE)

**Condition:** `||linear_acceleration|| > imu_max_linear_accel` (default 49.0 m/s²)

The 3-axis acceleration magnitude is compared against the physical sensor limit.
At ~5G (49 m/s²), any MEMS accelerometer would be saturated or physically
destroyed.  A stationary IMU measures only gravity (≈9.81 m/s²); a vehicle in
motion adds centripetal and tangential components, both much smaller than 5G
under normal driving conditions.  Exceeding this threshold is impossible without
sensor failure or an extreme physical event (collision at >50 m/s).

**Severity:** CRITICAL.  Any value above 5G must be treated as evidence of sensor
failure, not a recoverable condition.

#### 3.2.2.2 IMU Angular Velocity Envelope (IMU_GYRO_OUT_OF_ENVELOPE)

**Condition:** `||angular_velocity|| > imu_max_angular_velocity` (default 10.0 rad/s)

A rapid vehicle U-turn at 30 km/h produces ~1.5 rad/s; a racing vehicle at high
speed might reach ~3 rad/s.  10 rad/s requires the vehicle to complete a full
rotation in 0.6 s — mechanically impossible without catastrophic failure.

**Severity:** CRITICAL.  Indicates sensor failure, electrical interference, or
an extreme physical event incompatible with normal vehicle operation.

#### 3.2.2.3 IMU Gravity Range (IMU_GRAVITY_OUT_OF_ENVELOPE)

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
this check runs.  No terrain correction is applied.

### 3.4 Check: Wheel Odometry Cross-Check (`odometry_checker`)

**Node:** `odometry_checker`
**Topics:** `/sim/odometry` (50 Hz), `/sim/gnss` (10 Hz) → `/poise/integrity_status`

#### 3.4.1 Detection Method

At each GNSS fix the node compares two scalar path lengths, both accumulated
from the same sliding-window anchor point:

- **GNSS path length** — Euclidean distance between the anchor geodetic position
  and the current GNSS position, computed directly from geodetic coordinates
  (`_geodetic_distance_m`).  No shared world-frame reference origin is involved.
- **Odometry path length** — Total distance the wheel odometer reports since the
  anchor, computed by integrating scalar speed (`√(vx² + vy²)`) over time.

Comparing scalar path lengths rather than signed displacements makes the check
**frame-independent**: the result does not depend on vehicle heading, body-frame
mounting conventions, or whether the vehicle is turning.  The delta
`|gnss_path_length − odom_path_length|` measures the pure path-length discrepancy.

#### 3.4.2 Sliding Integration Window

The integration window is 180 s (configurable via `integration_window_s`).
This is longer than the IMU DR window (60 s) because odometry position error
grows linearly O(T) rather than O(T^1.5) for IMU, making odometry a more
reliable reference over longer horizons.  With a 180 s window and a 1.5 m
warn threshold the minimum detectable drift rate is approximately 8 mm/s,
covering realistic urban GNSS multipath scenarios.

At each 180 s boundary the anchor is realigned to the current GNSS position
and odometry accumulation is reset.

#### 3.4.3 Divergence Thresholds

| Fault Code | Threshold | Severity |
|---|---|---|
| `ODOM_GNSS_DIVERGENCE_WARN` | delta > 1.5 m | WARN (non-recoverable) |
| `ODOM_GNSS_DIVERGENCE_CRITICAL` | delta > 3.0 m | CRITICAL (non-recoverable) |

These thresholds match the GNSS/IMU cross-check to maintain a consistent
definition of lane-level integrity across all cross-checks.

#### 3.4.4 Slip Detection (ODOM_SLIP_SUSPECTED)

In addition to absolute divergence, the node evaluates the **slip ratio**:

```
slip_ratio = odom_path_length / gnss_path_length
```

If `slip_ratio < 0.90` (odometry understates GNSS path length by more than 10%)
and GNSS displacement since anchor exceeds 5 m (to avoid divide-by-near-zero
false alarms at standstill), `ODOM_SLIP_SUSPECTED` is raised.

The 5 m minimum displacement guard also avoids false detection during the first
seconds of a new window when both measurements are near zero.

**Non-recoverability rationale:** Systematic wheel slip does not self-correct.
A vehicle with persistently slipping wheels has compromised odometry that will
continue to understate true displacement.  The condition requires operator
inspection (tyre condition, surface assessment) before the localization stack
can be trusted.

#### 3.4.5 Odometry Dropout (ODOM_DROPOUT)

If no odometry message is received for longer than `dropout_timeout_s` (2 s),
`ODOM_DROPOUT` is raised.  The fault clears automatically when the next odometry
message arrives, at which point the dt baseline is reset to prevent a spuriously
large integration step across the silent gap.

**Recoverability rationale:** Odometry silence may be transient (brief CAN bus
glitch, node restart).  Unlike a divergence fault, silence does not by itself
indicate a position error — it indicates a loss of sensing capability that may
self-restore.

---

## 4. Trust State Machine

### 4.1 States

| State | Meaning | Recommended Planner Response |
|---|---|---|
| `TRUSTED` | All checks nominal. Localization may be used for normal AV operation. | Normal operation. |
| `ENVIRONMENT DEGRADED` | One or more environmental faults active; no integrity fault active. Sensor hardware is functioning but operating conditions are temporarily degraded. | Reduce speed, increase following distance, avoid complex manoeuvres. Continue operating. Recovery expected. |
| `SYSTEM DEGRADED` | One or more integrity faults active. Localization source disagreement or sensor fault suspected. Cause is unknown and may persist. | Prepare for safe stop. Do not initiate new complex manoeuvres. Alert operator. Do not assume recovery. |
| `UNTRUSTED` | Localization integrity cannot be assured. A CRITICAL fault has been detected, or a SYSTEM DEGRADED condition has persisted beyond the escalation timeout. | Immediate safe stop. Refuse intersection entry and lane changes. Hand control to operator. |

The distinction between `ENVIRONMENT DEGRADED` and `SYSTEM DEGRADED` is
operationally significant: a planner receiving `ENVIRONMENT DEGRADED` can
continue with reduced capability in the expectation of recovery, while a planner
receiving `SYSTEM DEGRADED` should treat the condition as potentially permanent
and prepare for a safe stop.

### 4.2 State Determination

The aggregator evaluates all active fault codes at each cycle and determines
the trust state using the following priority-ordered logic:

```
1. If any active fault code has severity CRITICAL
       → UNTRUSTED (immediate, regardless of category)

2. If SYSTEM DEGRADED has been active continuously for
   longer than warn_escalation_timeout_s (60 s)
       → UNTRUSTED

3. If any active fault code has category INTEGRITY
       → SYSTEM DEGRADED

4. If any active fault code has category ENVIRONMENTAL
   and no integrity fault is active
       → ENVIRONMENT DEGRADED

5. If no active fault codes
       → TRUSTED
```

This logic is evaluated top-to-bottom. The first matching condition determines
the state. Rules 1 and 2 take precedence over all others.

**Escalation timeout rationale:** The 60-second timeout for `SYSTEM DEGRADED →
UNTRUSTED` is deliberately longer than the previous 5-second design. A sustained
integrity WARN may represent a genuine but slowly developing fault, or it may
represent a borderline detection near the threshold. A longer timeout allows
the planner time to respond — reducing speed, completing a current manoeuvre
safely, or pulling over in a controlled manner — before the more restrictive
UNTRUSTED state is enforced. A CRITICAL fault bypasses this timeout entirely
and transitions to UNTRUSTED immediately.

### 4.3 Recovery Rules

```
From ENVIRONMENT DEGRADED:
    All environmental faults clear
        → start revalidation_period_s (10 s) timer
        → no new faults during period
        → return to TRUSTED (auto-recovery)

From SYSTEM DEGRADED:
    Integrity fault has cleared (no longer active)
    AND operator calls /poise/reset
        → /poise/reset accepted
        → start revalidation_period_s (10 s) timer
        → no new faults during period
        → return to TRUSTED

    Integrity fault clears but environmental fault remains active
        → system transitions to ENVIRONMENT DEGRADED
        → environmental fault must then clear before auto-recovery to TRUSTED

    /poise/reset called while integrity fault still active
        → reset rejected; fault must clear before reset is accepted

From UNTRUSTED:
    Same rules as SYSTEM DEGRADED — integrity fault must clear,
    then operator calls /poise/reset
        → /poise/reset accepted
        → start revalidation_period_s (10 s) timer
        → no new faults during period
        → return to TRUSTED
```

New faults arriving during the revalidation period restart the timer.

### 4.4 Recovery Rationale

**Environmental faults (ENVIRONMENT DEGRADED):**
The fault condition is caused by external factors outside the sensor's control.
Once the condition clears — the vehicle exits the tunnel, satellite visibility
improves, odometry topic resumes — a short revalidation period confirms stability
before returning to TRUSTED. Operator action is not required because the sensor
hardware is functioning correctly and no integrity violation has occurred.

**Integrity faults (SYSTEM DEGRADED / UNTRUSTED):**
The cause of the fault is unknown. It may be transient (a momentary electrical
spike) or persistent (a loose IMU bracket, systematic GNSS multipath). Allowing
auto-recovery when the fault condition clears would permit the system to return
to TRUSTED without any confirmation that the underlying cause has been resolved.
For a fault that was transient, this is conservative but safe. For a fault that
is persistent but intermittent, auto-recovery would create a false sense of
safety — the system resumes normal operation during the quiet period and the
fault reappears later, potentially at a worse moment.

Requiring the fault to have cleared before reset is accepted ensures the operator
is not acknowledging an active fault. Requiring `/poise/reset` ensures a human
has made a deliberate decision to return the system to TRUSTED. This is consistent
with ISO 26262 Part 6 §9.4.3 (fail-safe state persistence) and SOTIF (ISO 21448)
§8.3 (safe state definition).

---

## 5. QoS Profile Design

All QoS decisions are centralised in `poise/qos.py`.

| Profile | Reliability | Durability | Depth | Used for |
|---|---|---|---|---|
| `SENSOR_QOS` | BEST_EFFORT | VOLATILE | 10 | `/sim/gnss`, `/sim/imu`, `/sim/vehicle_state`, `/sim/odometry` |
| `INTEGRITY_QOS` | RELIABLE | VOLATILE | 10 | `/poise/integrity_status` |
| `SYSTEM_STATUS_QOS` | RELIABLE | TRANSIENT_LOCAL | 1 | `/poise/system_integrity`, `/poise/heartbeat` |

`/sim/vehicle_state` and `/sim/odometry` use `SENSOR_QOS` to match Autoware's
kinematic_state and odometry topic QoS conventions.  A dropped packet is
acceptable — the extrinsic validator conservatively holds the last known
stationary/moving determination, and the odometry checker's dropout detection
will raise `ODOM_DROPOUT` if silence persists beyond the configured timeout.

`/poise/heartbeat` uses `SYSTEM_STATUS_QOS` (RELIABLE / TRANSIENT_LOCAL) so
that a watchdog joining late immediately receives the most recent heartbeat
timestamp and can establish a baseline before monitoring the publishing interval.

---

## 6. Known Limitations

| Limitation | Impact | Planned implementation |
|---|---|---|
| Extrinsic check assumes level terrain | A sustained road slope produces apparent gravity deviation | TBD |
| No orientation correction applied | The extrinsic check cannot distinguish mount tilt from vehicle roll/pitch | TBD |
| Extrinsic check is stationary-only | Mount shift during motion is not detectable | TBD |
| Only two localization modalities | Cannot isolate which sensor is faulty | TBD |
| Flat-Earth ENU approximation | Error < 1 mm at < 100 m baseline; acceptable for integrity checking | N/A |
| Single-threaded ROS callbacks | High IMU rate (100 Hz) and check computations compete on the ROS executor | Acceptable for simulation |
| DR anchor accepts drifted GNSS on realign | A deliberately drifted GNSS position becomes the new anchor at t=60 s | Mitigated by fault escalation within the first window |
| Calibration envelope has no temperature model | IMU sensitivity drifts with temperature; static thresholds may be too tight in extreme environments | TBD |
| GNSS/IMU cross-check assumes near-stationary vehicle | IMU dead-reckoning integrates acceleration; constant-velocity motion produces near-zero DR displacement causing false divergence with a moving GNSS | TBD — requires velocity-seeded DR or GNSS velocity integration |

---

## 7. Monitor Integrity

### 7.1 The "Who Watches the Watchman" Problem

POISE is a safety monitor.  If the monitor itself fails silently, the system it protects
loses its integrity guarantee without any indication that the protection has been
removed.

A POISE process crash, an unhandled exception in the ROS2 executor, a node
respawn that loses accumulated fault state, or a silent memory corruption can
all cause POISE to stop performing its monitoring function.  The system under
supervision would continue operating under a false assumption of monitoring
coverage.

### 7.2 /poise/heartbeat as the External Interface Point

POISE publishes a `std_msgs/Header` message on `/poise/heartbeat` at a
configurable rate (default 1 Hz, configured via `heartbeat_rate_hz` in
`sim_config.yaml`).  Each message contains only the current timestamp — there
is no payload.

The heartbeat is published **unconditionally** regardless of the current trust
state (`TRUSTED`, `ENVIRONMENT DEGRADED`, `SYSTEM DEGRADED`, or `UNTRUSTED`).
It is architecturally separate from `/poise/system_integrity`, which communicates
integrity *state*.  The heartbeat communicates *liveness*: the POISE process is
running and its ROS2 executor is alive.

A POISE process failure causes the heartbeat to go **silent**.  Any external
subscriber monitoring the publishing interval will detect this silence and can
take appropriate action (inhibit autonomous operation, alert the operator,
engage a fallback safety mode).

The intended consumer is a **vehicle safety arbiter** or **system watchdog**
external to POISE — running in a separate process, on a separate compute unit,
or implemented in hardware.  Its design is outside the scope of this document
and of the POISE package.

---

## 8. Relationship to ISO 26262 / SOTIF

POISE is a research/portfolio prototype.  It has **not** been developed
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