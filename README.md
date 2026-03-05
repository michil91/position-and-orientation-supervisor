# POISE — Position and Orientation Integrity Supervision Engine

> **Phase 1** · ROS2 Humble · Python · Autoware-compatible

POISE monitors the trustworthiness of an autonomous vehicle's localization
solution by cross-checking independent sensor sources against each other and
publishing a system-level integrity state that a supervisory controller can
act on.

---

## Motivation — The Localization Integrity Problem

Autonomous vehicles rely on a fused localization estimate (GNSS + IMU + LiDAR)
to navigate safely.  Sensor fusion algorithms produce a single "best estimate"
that appears smooth and confident — even when one or more inputs are faulty.
A jump, drift, or outage in GNSS may not be visible in the fused output until
the vehicle has deviated significantly from its intended path.

POISE adds a **supervision layer** that watches the raw sensor streams
independently, detects disagreements, and raises integrity alarms before the
fault propagates into the vehicle's control loop.

---

## Architecture

```
 ┌─────────────────────────────────────────────────────────────────────┐
 │                        POISE Phase 1                                │
 │                                                                     │
 │  ┌──────────────────┐    /sim/gnss     ┌──────────────────────┐    │
 │  │  gnss_publisher  │ ─────────────→   │                      │    │
 │  │  (sim node)      │                  │  gnss_imu_checker    │    │
 │  └──────────────────┘                  │  (cross-check)       │    │
 │                                        │                      │    │
 │  ┌──────────────────┐    /sim/imu      │  • DR integration    │    │
 │  │  imu_publisher   │ ─────────────→   │  • Cov check         │    │
 │  │  (sim node)      │                  │  • Threshold compare │    │
 │  └──────────────────┘                  └──────────┬───────────┘    │
 │                                                   │                │
 │                                    /poise/integrity_status         │
 │                                                   │                │
 │                                        ┌──────────▼───────────┐    │
 │                                        │ integrity_aggregator │    │
 │                                        │                      │    │
 │                                        │  TRUSTED             │    │
 │                                        │    ↓ WARN            │    │
 │                                        │  DEGRADED            │    │
 │                                        │    ↓ CRITICAL/timeout│    │
 │                                        │  UNTRUSTED           │    │
 │                                        └──────────┬───────────┘    │
 │                                                   │                │
 │                               /poise/system_integrity (JSON)       │
 │                               + JSONL log file                     │
 └─────────────────────────────────────────────────────────────────────┘
```

---

## Check Definitions

### Check 1 — GNSS/IMU Position Divergence

At every GNSS fix (10 Hz), the GNSS-reported position is compared against the
position predicted by dead-reckoning the IMU (Euler integration of linear
acceleration between fixes).

| Condition | Threshold (default) | Fault Code | Status |
|---|---|---|---|
| delta ≤ 1.5 m | — | — | OK |
| delta > 1.5 m | warn_threshold_m | GNSS_IMU_DIVERGENCE_WARN | WARN |
| delta > 3.0 m | critical_threshold_m | GNSS_IMU_DIVERGENCE_CRITICAL | CRITICAL |

**Threshold rationale:**
- 1.5 m exceeds 3σ noise for a 0.5 m stddev sensor over a 0.1 s DR window.
  A healthy stationary vehicle produces < 0.1 m delta.
- 3.0 m corresponds to minimum lane-level positioning accuracy; an error of
  this magnitude places the vehicle in an adjacent lane or on a kerb.

Dead-reckoning state is **reset to GNSS position** after each check.  This is
intentional — the check tests short-term consistency (step jumps, sudden
divergence), not long-term navigation accuracy.

### Check 2 — GNSS Covariance

The GNSS receiver's reported horizontal position covariance is compared against
`max_covariance_m2` (default 25 m² = 5 m 1σ).  Exceeding this threshold raises
STATUS_WARN with fault code `GNSS_COVARIANCE_EXCEEDED`.

---

## Trust State Machine

```
         ┌───────────┐
   start →│  TRUSTED  │
         └─────┬─────┘
               │ any WARN
               ▼
         ┌───────────┐
         │ DEGRADED  │
         └─────┬─────┘
               │ CRITICAL  │  two simultaneous WARNs
               │ or WARN sustained > escalation_timeout
               ▼
         ┌───────────┐
         │ UNTRUSTED │  ← terminal state (no auto-recovery)
         └───────────┘
```

### No-Auto-Recovery Rationale

POISE **never automatically recovers** to a better state.  Recovery requires an
explicit operator action (system restart or future `/poise/reset` service).

**Why:** An autonomous recovery after a sensor fault may re-engage AV operation
when:
- The fault is intermittent and will recur
- The position estimate drifted during the fault and was not corrected
- The root cause is unknown

Conservative safety design (aligned with ISO 26262 §9.4.3 fail-safe state
persistence) requires that recovery is a deliberate act with human or validated
software confirmation.

---

## Fault Injection Usage

All fault modes are controlled entirely through `config/sim_config.yaml`.

### GNSS Fault Modes

```yaml
gnss_publisher:
  ros__parameters:
    fault_mode: drift        # none | drift | jump | dropout | covariance_inflation

    # drift parameters
    drift_rate_m_per_s: 0.05
    drift_direction_deg: 0.0

    # jump parameters
    jump_time_s: 20.0
    jump_north_m: 5.0
    jump_east_m: 0.0

    # dropout parameters
    dropout_start_s: 15.0
    dropout_duration_s: 5.0

    # covariance_inflation parameters
    inflated_covariance_m2: 50.0
    inflation_start_s: 10.0
    inflation_duration_s: 10.0
```

### IMU Fault Modes

```yaml
imu_publisher:
  ros__parameters:
    fault_mode: bias         # none | bias | spike | dropout

    bias_axis: x
    bias_magnitude_mps2: 0.5

    spike_time_s: 10.0
    spike_duration_s: 0.1
    spike_magnitude_mps2: 10.0

    dropout_start_s: 15.0
    dropout_duration_s: 3.0
```

---

## Build and Run Instructions

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble (`sudo apt install ros-humble-desktop`)
- `colcon` build tool

### 1 — Build

```bash
cd ~/projects/poise/poise_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select poise
```

### 2 — Verify entry points

```bash
source install/setup.bash
ros2 pkg executables poise
# Expected:
#   poise gnss_publisher
#   poise imu_publisher
#   poise gnss_imu_checker
#   poise integrity_aggregator
```

### 3 — Launch (nominal scenario)

```bash
ros2 launch poise poise_phase1.launch.py
```

### 4 — Verify topics

In a second terminal:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic list
# Should include:
#   /sim/gnss
#   /sim/imu
#   /poise/integrity_status
#   /poise/system_integrity

ros2 topic echo /poise/system_integrity
ros2 topic echo /poise/integrity_status
```

### 5 — Fault injection test (GNSS drift)

Edit `src/poise/config/sim_config.yaml`:

```yaml
gnss_publisher:
  ros__parameters:
    fault_mode: drift
    drift_rate_m_per_s: 0.05
```

Rebuild and relaunch:

```bash
colcon build --packages-select poise && \
source install/setup.bash && \
ros2 launch poise poise_phase1.launch.py
```

Monitor integrity status:

```bash
ros2 topic echo /poise/integrity_status
# After ~30 s (1.5 m / 0.05 m/s) you should see STATUS_WARN
# After ~60 s (3.0 m / 0.05 m/s) you should see STATUS_CRITICAL

ros2 topic echo /poise/system_integrity
# State transitions: TRUSTED → DEGRADED → UNTRUSTED
```

Check the JSON log:

```bash
cat /tmp/poise_integrity_log.jsonl | python3 -m json.tool --no-ensure-ascii
```

---

## Relationship to Autoware Universe

POISE is designed to monitor Autoware localization outputs.  The sim topics
(`/sim/gnss`, `/sim/imu`) can be remapped to Autoware's live topics using
launch arguments or a topic remapping YAML:

| POISE sim topic | Autoware topic |
|---|---|
| `/sim/gnss` | `/sensing/gnss/ublox/nav_sat_fix` |
| `/sim/imu`  | `/sensing/imu/tamagawa/imu_raw`   |

To connect to a live Autoware stack, replace the sim publisher nodes with
remapping-only launch entries and point the checker at the real sensor topics:

```python
# In poise_phase1.launch.py — replace sim publishers with remappings
Node(
    package='poise',
    executable='gnss_imu_checker',
    remappings=[
        ('/sim/gnss', '/sensing/gnss/ublox/nav_sat_fix'),
        ('/sim/imu',  '/sensing/imu/tamagawa/imu_raw'),
    ],
)
```

The `IntegrityStatus` and `system_integrity` outputs can then be consumed by
Autoware's `system_monitor` or a custom safety manager node.

---

## See Also

- [`docs/safety_concept.md`](docs/safety_concept.md) — Formal safety rationale,
  failure mode analysis, and known limitations
