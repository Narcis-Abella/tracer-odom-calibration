# Experimental Methodology

## 1. Problem Statement

The AgileX Tracer 2.0 is a skid-steer mobile platform used as the base for an autonomous hospital AGV. Its wheel odometry — published on `/odom` — is the primary dead-reckoning input for SLAM and navigation. Field observations revealed significant and inconsistent yaw error: the robot frequently over- or under-rotated relative to the commanded angle, with the discrepancy varying noticeably across different velocity settings.

The goal of this work is to **characterize the rotational odometry error as a function of angular velocity and acceleration**, producing a quantitative model that can be used to:
1. Improve the wheel odometry noise model in the Ignition Gazebo simulation ([hospital-agv-sim](https://github.com/Narcis-Abella/hospital-agv-sim))
2. Optionally implement a velocity-dependent correction node on the real robot

---

## 2. Software Architecture Discovery

Before designing the experiment, the full odometry computation stack was traced:

```
/cmd_vel  →  tracer_base_node  →  CAN bus  →  Tracer firmware  →  encoders
                                                      │
                                         ω_reported = (v_R - v_L) / track_width_fw
                                                      │ (via CAN)
                                         tracer_messenger.cpp
                                                      │
                                         θ += ω_reported × dt  →  /odom
```

**Key finding:** the Tracer firmware computes angular velocity from encoder counts internally and reports the result over CAN. The ROS driver (`tracer_messenger.cpp`) integrates this reported value — it does not have access to raw encoder data or the `track_width` used in the firmware computation.

Consequences for calibration:
- There is no `track_width` parameter to tune at the ROS level
- Any correction must be applied as a post-processing factor on the `/odom` output
- The error model must account for firmware behavior (e.g., how the firmware handles low-velocity encoder noise, or high-velocity slip)

---

## 3. Error Hypotheses

Two physical mechanisms are expected to dominate the error:

### 3.1 Stiction / backlash at low velocity
Near the motor activation threshold (observed around ω ≈ 0.3 rad/s), the drivetrain exhibits stiction. The firmware may report angular velocity via CAN before the wheels begin moving cleanly, causing `/odom` to accumulate angle while the robot is not yet rotating — or vice versa. Expected effect: **positive error** (odom overestimates angle) or inconsistent behavior.

Empirically, the robot was observed to rotate ~10–20° more than `/odom` reported at low velocities, suggesting `/odom` underestimates actual rotation in this regime.

### 3.2 Wheel slip at high velocity
On polished porcelain tile (low friction coefficient), wheel slip increases with velocity. When wheels slip, encoder counts underestimate true wheel displacement → firmware underestimates angular velocity → `/odom` underestimates actual rotation → robot overshoots the commanded angle.

Expected effect: **positive error** (robot rotates more than odom reports), increasing with velocity.

### 3.3 Predicted error shape
The two mechanisms together predict a **U-shaped error curve** as a function of ω:
- High error at low ω (stiction regime)
- Minimum error at mid ω (both mechanisms minimal)
- Increasing error at high ω (slip regime)

This is consistent with the near-zero visual error observed at ω=2.0, α=2.0 in preliminary tests (though that specific combination may also involve coincidental cancellation).

---

## 4. Experimental Design

### 4.1 Control script
A ROS node (`calibracion_bloque*.py`) commands the robot to rotate by a target angle using a **trapezoidal velocity profile** and stops when `/odom` accumulates the target angle — not based on time. This makes each measurement independent of the velocity/acceleration parameters (the script always stops at the same `/odom` value) while allowing comparison of the resulting real-world rotation.

The trapezoidal profile:
- **Ramp-up:** ω increases at rate α until reaching ω_max
- **Cruise:** constant ω_max
- **Ramp-down:** triggered when remaining angle ≤ ω_max² / (2α). Velocity follows:

```
ω_target = min(sqrt(2 · α · θ_remaining), ω_max)
```

Stop condition: the script terminates when `ω` drops below 0.04 rad/s after having reached at least 50% of `ω_max` (to distinguish end-of-motion from a failure to start).

### 4.2 Measurement matrix

| Factor | Levels | Rationale |
|--------|--------|-----------|
| Target angle | 90°, 180°, 360° | Tests whether error scales linearly with angle (multiplicative) or is a fixed offset |
| ω_max | 0.3, 0.35, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5, 2.0 rad/s | Higher density in 0.3–0.6 range where the steepest gradient was observed |
| α | 0.5, 1.0, 2.0 rad/s² | Tests whether acceleration (ramp aggressiveness) affects error independently of peak velocity |
| Direction | CCW (+1), CW (−1) | Tests directional asymmetry due to motor/drivetrain asymmetry |
| Repetitions | n = 5 | Sufficient for mean ± std characterization; enables outlier detection |

**Total:** 3 × 9 × 3 × 2 × 5 = **810 measurements**

### 4.3 Session structure
Measurements are split into 4 independent blocks, each producing its own CSV:

| Block | File | Content | Est. Time |
|-------|------|---------|-----------|
| 1 | `resultados_bloque1_90.csv` | 90° — all velocities, accelerations, directions | ~38 min |
| 2 | `resultados_bloque2_180.csv` | 180° — all velocities, accelerations, directions | ~51 min |
| 3 | `resultados_bloque3_360_bajas.csv` | 360° — ω ∈ [0.3, 0.5] | ~44 min |
| 4 | `resultados_bloque4_360_altas.csv` | 360° — ω ∈ [0.6, 2.0] | ~32 min |

---

## 5. Ground Truth Measurement

### 5.1 Camera setup
- **Device:** iPhone XR (rear camera, 1080×1920 @ 30fps)
- **Mount:** fixed tripod, overhead/angled view of robot
- **Lens intrinsics:** calibrated from a checkerboard pattern (`camera_calibration.py`), RMS reprojection error = 0.05 px

### 5.2 ArUco marker
- **Dictionary:** DICT_4X4_50
- **Marker ID:** 0
- **Physical size:** 9.58 cm (0.0958 m)
- **Placement:** fixed to robot chassis, visible from camera position

For each video frame, `extract_aruco.py` computes the full 6-DOF pose of the marker via `solvePnP`. The yaw component (rotation around the vertical axis) gives the instantaneous robot heading. Integrating heading changes across a run gives `Δθ_real`.

### 5.3 Temporal synchronization
The key challenge is aligning the video timeline with the ROS timestamps in the CSV. This is solved by a **sync move**: at the start and end of each recording session, the robot executes an abrupt 30° CW + 30° CCW rotation at 2.0 rad/s with no ramp. This produces a sharp, easily detectable signature in both streams:

- In the video: a sudden large ArUco displacement
- In the ROS CSV: a `SYNC_START` / `SYNC_END` row with a precise Unix timestamp

`merge_calibration.py` detects the sync events in both streams, computes the time offset, and applies it to align all intermediate measurements. Residual sync drift < 0.5s is the acceptance criterion.

### 5.4 Run segmentation
`extract_aruco.py` segments the video into individual runs by detecting pauses: periods where the cumulative ArUco yaw change is below `PAUSA_UMBRAL_DEG` for at least `PAUSA_MIN_SEG` seconds. The number of detected runs must match the ROS CSV exactly before the merge proceeds.

---

## 6. Planned Statistical Analysis

Once the full dataset is available, `robot_rotation_analyzer.py` will compute:

1. **Per-combination statistics:** mean(Δθ_real − Δθ_odom), std, and 95% CI for each (ω, α, angle, direction) combination
2. **Error vs velocity curves:** for each target angle and acceleration level, plot mean error vs ω_max with error bars
3. **Directional asymmetry test:** paired t-test comparing CW vs CCW error at each (ω, α, angle) combination
4. **Angle scaling test:** check whether error at 360° ≈ 4× error at 90° (multiplicative/systematic) vs a fixed offset
5. **Acceleration effect:** ANOVA across α levels within each ω — specifically to test the hypothesis that high-α, low-ω combinations are statistically indistinguishable (the "triangular profile" degeneracy noted in the design)
6. **Model fitting:** fit a parametric model `error(ω, α) = f(ω, α)` — candidate forms include polynomial regression, piecewise linear, or a physics-motivated model based on the stiction/slip decomposition

---

## 7. Surface Dependency Note

All measurements were taken on polished porcelain tile (~50×50 cm) at IQS Barcelona. This is a low-friction surface that is expected to amplify slip-induced error at higher velocities. The error model derived from this dataset is **surface-specific**. Deployment on carpet, concrete, or outdoor surfaces will require recalibration or a surface-aware correction term.

---

## 8. Known Limitations

- **Single surface:** results do not generalize across floor types without recalibration
- **Temperature / tire wear:** not controlled — assumed stable within sessions
- **CAN bus stability:** the Tracer driver occasionally drops the CAN connection mid-session; affected runs are detectable by anomalous duration in the CSV
- **ArUco detection failures:** occasional frame dropout in the video (blur, occlusion) introduces noise in Δθ_real estimates; `extract_aruco.py` interpolates across short gaps
- **Directional hysteresis:** the robot may not return to exactly its starting heading after each run. Accumulated heading error across a block is not corrected — each run's Δθ is computed relative to its own starting frame
