# POISE — Position and Orientation Integrity Supervision Engine

> **Phase 2** · ROS2 Humble · Python · Autoware-compatible

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
 │                        POISE Phase 2                                │
 │                                                                     │
 │  ┌──────────────────┐    /sim/gnss     ┌──────────────────────┐    │
 │  │  gnss_publisher  │ ─────────────→   │                      │    │
 │  │  (sim node)      │  SENSOR_QOS      │  gnss_imu_checker    │    │
 │  └──────────────────┘                  │  (cross-check)       │    │
 │                                        │                      │    │
 │  ┌──────────────────┐    /sim/imu      │  • DR integration    │    │
 │  │  imu_publisher   │ ─────────────→   │  • Cov / sat check   │    │
 │  │  (sim node)      │  SENSOR_QOS      │  • Dropout detection │    │
 │  └──────────────────┘                  │  • recoverable flag  │    │
 │                                        └──────────┬───────────┘    │
 │                                                   │ INTEGRITY_QOS  │
 │                                    /poise/integrity_status         │
 │                                                   │                │
 │                                        ┌──────────▼───────────┐    │
 │                                        │ integrity_aggregator │    │
 │                                        │                      │    │
 │                                        │  TRUSTED ←──────┐   │    │
 │                                        │    ↓ any WARN    │   │    │
 │                                        │  DEGRADED        │   │    │
 │                                        │    ↓             │ revalidation
 │                                        │  UNTRUSTED ──────┘   │    │
 │                                        │  (reset via service) │    │
 │                                        └──────────┬───────────┘    │
 │                                                   │ SYSTEM_STATUS  │
 │                               /poise/system_integrity (JSON)       │
 │                               /poise/reset (Trigger service)       │
 │                               + JSONL log file                     │
 └─────────────────────────────────────────────────────────────────────┘
```

---

## Check Definitions

Phase 2 introduces five classified fault codes with explicit recoverability.

| Fault Code | Severity | Recoverable | Trigger |
|---|---|---|---|
| `GNSS_DROPOUT` | WARN | **Yes** | No GNSS fix for > `gnss_dropout_timeout_s` (2 s) |
| `GNSS_HIGH_COVARIANCE` | WARN | **Yes** | Reported covariance > `max_covariance_m2` (25 m²) |
| `GNSS_LOW_SATELLITES` | WARN | **Yes** | NavSatFix status indicates no fix |
| `GNSS_IMU_DIVERGENCE_WARN` | WARN | **No** | GNSS/IMU divergence > `warn_threshold_m` (1.5 m) |
| `GNSS_IMU_DIVERGENCE_CRITICAL` | CRITICAL | **No** | GNSS/IMU divergence > `critical_threshold_m` (3.0 m) |

### Check 1 — GNSS/IMU Position Divergence (non-recoverable)

At every GNSS fix (10 Hz), the GNSS-reported position is compared against the
position predicted by dead-reckoning the IMU (Euler integration of linear
acceleration between fixes).

| Condition | Threshold (default) | Fault Code | Status |
|---|---|---|---|
| delta ≤ 1.5 m | — | — | OK |
| delta > 1.5 m | warn_threshold_m | GNSS_IMU_DIVERGENCE_WARN | WARN |
| delta > 3.0 m | critical_threshold_m | GNSS_IMU_DIVERGENCE_CRITICAL | CRITICAL |

Dead-reckoning uses a **sliding 60 s window** (`dr_realign_window_s`):
the DR anchor is re-set to the current GNSS position every 60 s.  This detects
slow drift accumulating over the window while bounding IMU integration error.

### Check 2 — GNSS Covariance (recoverable)

The GNSS receiver's reported horizontal position covariance is compared against
`max_covariance_m2` (default 25 m² = 5 m 1σ).  Exceeding this threshold raises
STATUS_WARN with fault code `GNSS_HIGH_COVARIANCE` (`recoverable=True`).

### Check 3 — GNSS Dropout (recoverable)

A 1 Hz timer monitors the elapsed time since the last GNSS fix.  If no fix
arrives within `gnss_dropout_timeout_s` (2 s), `GNSS_DROPOUT` is raised
(`recoverable=True`).  On re-acquisition the DR is force-realigned to prevent
a spurious divergence alarm.

---

## Trust State Machine (Phase 2)

```
         ┌───────────┐
   start →│  TRUSTED  │◄──────────────────────────────┐
         └─────┬─────┘                                │
               │ any WARN                              │ revalidation
               ▼                                      │ complete
         ┌───────────┐                                │
         │ DEGRADED  │────────────────────────────────┤
         └─────┬─────┘                                │
               │ CRITICAL | 2× WARNs                  │
               │ | non-recoverable WARN > timeout      │
               ▼                                      │
         ┌───────────┐                                │
         │ UNTRUSTED │────── /poise/reset ────────────┘
         └───────────┘  (accepted when no active faults)
```

### Recovery Rules

| Scenario | Behaviour |
|---|---|
| All active faults are **recoverable** and they clear | Start `revalidation_period_s` (10 s) timer; if no new faults → TRUSTED |
| Any **non-recoverable** fault was active | Auto-recovery blocked; operator must call `/poise/reset` |
| New fault arrives during revalidation | Revalidation timer restarted |
| Recoverable fault arrives while non-recoverable is active | Escalate to UNTRUSTED (potential masking attempt) |

### `/poise/reset` Service

```bash
ros2 service call /poise/reset std_srvs/srv/Trigger
```

- **Accepted** when no fault conditions are currently active.
- **Rejected** if any check is still in WARN or CRITICAL.
- Clears non-recoverable fault history; returns to TRUSTED.
- Every attempt (accepted or rejected) is logged to the JSONL file.

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
cd ~/projects   # parent of the poise/ repo root
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
ros2 launch poise poise_phase2.launch.py
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

### 5 — Fault injection test (GNSS drift — non-recoverable)

Edit `config/sim_config.yaml` under `gnss_publisher`:

```yaml
    fault_mode: drift
    drift_rate_m_per_s: 0.05
```

Relaunch (no rebuild needed — config is loaded at runtime):

```bash
ros2 launch poise poise_phase2.launch.py
```

Monitor integrity status:

```bash
ros2 topic echo /poise/integrity_status
# After ~30 s (1.5 m / 0.05 m/s) you should see GNSS_IMU_DIVERGENCE_WARN
# After ~60 s (3.0 m / 0.05 m/s) you should see GNSS_IMU_DIVERGENCE_CRITICAL

ros2 topic echo /poise/system_integrity
# TRUSTED → DEGRADED → UNTRUSTED (does not auto-recover)
```

Reset after faults clear:

```bash
ros2 service call /poise/reset std_srvs/srv/Trigger
```

### 6 — Fault injection test (GNSS dropout — recoverable, auto-recovery)

```yaml
    fault_mode: dropout
    dropout_start_s: 15.0
    dropout_duration_s: 30.0
```

Expected sequence:
- t=17 s: GNSS_DROPOUT detected → DEGRADED
- t=45 s: GNSS resumes → revalidation timer starts (10 s)
- t=55 s: revalidation complete → auto-return to TRUSTED

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
