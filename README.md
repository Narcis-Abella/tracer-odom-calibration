# Tracer Agilex 2.0 — Rotational Odometry Calibration

**Characterizing velocity-dependent yaw error in a skid-steer AGV using ArUco-based visual ground truth**

> Part of an autonomous hospital AGV project for the [Social Tech Challenge](https://github.com/Narcis-Abella/SocialTech_C_Setup) at IQS School of Engineering, Universitat Ramon Llull, Barcelona.

---

## Demo

<video src="media/out.mp4" controls title="Project Demo" style="max-width: 100%;">
  Your browser does not support the video tag.
</video>

> The video shows the physical setup: Tracer Agilex 2.0 on polished tile, iPhone XR on tripod, ArUco marker on the robot chassis, and SSH connection to the Jetson Nano running the ROS measurement script.

---

## Why This Exists

The [hospital-agv-sim](https://github.com/Narcis-Abella/hospital-agv-sim) project uses a physics-grounded noise model for every sensor on the Tracer — IMU, LiDAR, RGB-D — but the wheel odometry model was a placeholder: a fixed Gaussian random walk on yaw, with no connection to the real hardware's actual error characteristics.

That is what this repo fixes.

Before deploying any SLAM stack (see [livox3d-slam-benchmark](https://github.com/Narcis-Abella/livox3d-slam-benchmark)) on a real hospital floor, you need to know how much the robot's `/odom` drifts and under what conditions. If the odometry is wrong by a velocity-dependent factor and you don't know it, your map will be wrong, your localization will drift, and your navigation will fail.

The goal of this project is to produce a **velocity-dependent yaw error model** — a function `f(ω, α)` that maps angular velocity and acceleration to expected odometry error — which will replace the placeholder noise model in the simulator.

---

## Key Finding (Preliminary)

> *Full statistical analysis pending the complete dataset. Preliminary results from an earlier session (n=3, single acceleration level) suggest the following.*

The odometry error is **not constant** and does not behave like simple white noise. It follows a velocity-dependent curve with two distinct regimes:

- **Low velocity (ω ≈ 0.3 rad/s):** large positive error — `/odom` underestimates actual rotation by ~10–20°/rev. Likely cause: drivetrain stiction and backlash near the motor activation threshold. The firmware reports angular velocity via CAN even when the wheels are not moving cleanly.

- **High velocity (ω ≥ 1.5 rad/s):** increasing positive error. Likely cause: wheel slip on polished porcelain tile (low friction surface). Encoder-derived velocity underestimates true rotation when wheels slip.

- **Mid-range (ω ≈ 0.6–1.0 rad/s):** minimum error regime. Near-zero visual error was observed at ω=2.0, α=2.0 in preliminary tests, though this may reflect coincidental cancellation between stiction and slip effects.

This U-shaped error profile rules out any simple fixed correction factor and motivates the full factorial experiment design described below.

---

## Experimental Design

A full factorial design with **810 measurements** across four sessions:

| Factor | Levels |
|--------|--------|
| Target angle | 90°, 180°, 360° |
| Angular velocity `ω` | 0.3, 0.35, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5, 2.0 rad/s |
| Angular acceleration `α` | 0.5, 1.0, 2.0 rad/s² |
| Direction | CCW (+1), CW (−1) |
| Repetitions | n = 5 per combination |

**162 unique combinations × 5 repetitions = 810 measurements**

The velocity range 0.3–0.6 rad/s was sampled at higher density (0.35, 0.4 added) because preliminary data showed the steepest error gradient in this region. The 1.0 rad/s practical ceiling was also observed during testing — the robot struggles to maintain stable rotation above this on polished tile at α=0.5.

Sessions are split into four independent blocks to enable parallel processing and fault isolation:

| Block | Script | Runs | Est. Time |
|-------|--------|------|-----------|
| 1 | `calibracion_bloque1_90.py` | 270 | ~38 min |
| 2 | `calibracion_bloque2_180.py` | 270 | ~51 min |
| 3 | `calibracion_bloque3_360_bajas.py` | 120 | ~44 min |
| 4 | `calibracion_bloque4_360_altas.py` | 150 | ~32 min |

---

## Ground Truth: ArUco-Based Visual Measurement

Standard wheel odometry calibration typically uses mechanical references (tape on the floor, protractors) or motion capture systems. This project uses a **monocular computer vision pipeline** running on a calibrated iPhone XR.

```
iPhone XR (1080×1920 @ 30fps, fixed tripod)
        │
        ▼
ArUco marker (ID=0, DICT_4X4_50, 9.58 cm) mounted on robot chassis
        │
        ▼  pose estimation via solvePnP
extract_aruco.py  →  Δθ_real per run  (aruco_iters.csv)
                  →  sync timestamps   (aruco_iters_sync.csv)
        │
        ▼
merge_calibration.py  ←  resultados_YYYYMMDD.csv (from Jetson /odom)
        │
        ▼
merged_YYYYMMDD.csv:  n, vel_max, acel, angulo_objetivo, direccion,
                      angulo_odom, angulo_real
```

**Camera calibration:** intrinsics computed from a checkerboard pattern using `camera_calibration.py`. RMS reprojection error: 0.05 px. Parameters stored in `scripts/calibracion_camara.json`.

**Temporal synchronization:** a *sync move* (abrupt 30° CW + 30° CCW at 2.0 rad/s, no ramp) is executed at the start and end of each recording. The sharp angular signature is easily detected in both the video (large ArUco displacement) and the ROS CSV (timestamp), providing sub-second synchronization between the two data streams.

See [docs/pipeline.md](docs/pipeline.md) for the full step-by-step data pipeline.

---

## Software Architecture Finding

A key discovery during setup: **the Tracer firmware computes angular velocity internally and reports it over CAN — it is not accessible or modifiable from the ROS layer.**

```
/cmd_vel → tracer_base_node → CAN → Tracer firmware → encoders
                                          ↓
                              ω = (v_right - v_left) / track_width_firmware
                                          ↓
                              reports angular_speed_ via CAN
                                          ↓
                         tracer_messenger.cpp → integrates → /odom
```

The ROS driver (`tracer_messenger.cpp`) integrates `angular_speed_` reported by the firmware — it does not compute odometry from wheel encoders directly. The `track_width` used for this computation is hardcoded in the firmware and inaccessible at the ROS level.

**Consequence:** any odometry correction must be applied post-hoc at the ROS level, either as a velocity-dependent scaling factor on `/odom` or as a corrective node republishing `/odom_calibrated`. This is the model that will be implemented in [hospital-agv-sim](https://github.com/Narcis-Abella/hospital-agv-sim).

---

## Repository Structure

```
tracer-odom-calibration/
├── scripts/
│   ├── calibracion_bloque1_90.py       # ROS node — block 1 (90°)
│   ├── calibracion_bloque2_180.py      # ROS node — block 2 (180°)
│   ├── calibracion_bloque3_360_bajas.py # ROS node — block 3 (360°, low vel)
│   ├── calibracion_bloque4_360_altas.py # ROS node — block 4 (360°, high vel)
│   ├── camera_calibration.py           # iPhone intrinsics from checkerboard
│   ├── extract_aruco.py                # ArUco pose estimation → Δθ_real
│   ├── merge_calibration.py            # Merge /odom CSV + ArUco CSV
│   ├── robot_rotation_analyzer.py      # Statistical analysis & plots
│   └── calibracion_camara.json         # Camera intrinsics (RMS=0.05px)
├── data/                               # CSVs generated per session (git-ignored if large)
├── docs/
│   ├── methodology.md                  # Full experimental methodology
│   └── pipeline.md                     # Step-by-step data pipeline
└── media/
    └── pattern.png                     # Checkerboard calibration pattern
```

---

## Setup & Reproducibility

### Hardware
- Robot: AgileX Tracer 2.0 (skid-steer)
- Compute: NVIDIA Jetson Nano (ROS1 Melodic, Ubuntu 20)
- Camera: iPhone XR, fixed tripod
- Surface: polished porcelain tile (~50×50 cm), IQS Barcelona

### ROS dependencies
```bash
# On Jetson
source ~/catkin_ws/devel/setup.bash
roslaunch tracer_bringup tracer_robot_base.launch --screen

# Run a measurement block
python3 calibracion_bloque1_90.py
```

### Python dependencies (post-processing, on host)
```bash
pip install opencv-python numpy pandas scipy matplotlib
```

### Camera calibration
```bash
python3 scripts/camera_calibration.py --video media/iphone_calibration.mov \
    --output scripts/calibracion_camara.json
```

### ArUco extraction + merge
```bash
python3 scripts/extract_aruco.py \
    --video media/SESSION.mov \
    --calib scripts/calibracion_camara.json \
    --output data/aruco_iters.csv

python3 scripts/merge_calibration.py \
    --aruco data/aruco_iters.csv \
    --sync  data/aruco_iters_sync.csv \
    --odom  data/resultados_YYYYMMDD.csv \
    --out   data/merged_YYYYMMDD.csv
```

---

## Status

| Phase | Status |
|-------|--------|
| Preliminary session (n=3, acel=0.5 only) |  Complete |
| Full factorial session (n=5, 810 runs) |  Data collection in progress |
| Statistical analysis & model fitting |  Pending full dataset |
| C++ implementation in hospital-agv-sim |  Pending model |

---

## Related Repositories

| Repository | Relation |
|---|---|
| [hospital-agv-sim](https://github.com/Narcis-Abella/hospital-agv-sim) | The calibrated model will replace the placeholder odometry noise node in the Ignition Gazebo simulation |
| [livox3d-slam-benchmark](https://github.com/Narcis-Abella/livox3d-slam-benchmark) | SLAM benchmarks depend on solid odometry as dead-reckoning baseline |
| [slam-sensor-metrological-validation](https://github.com/Narcis-Abella/slam-sensor-metrological-validation) | Parallel metrological track — sensor validation using ABB YuMi as ground truth |

---

## Environment

| Parameter | Value |
|---|---|
| Platform | AgileX Tracer 2.0 |
| ROS | ROS1 Melodic |
| OS (Jetson) | Ubuntu 20.04 |
| Surface | Polished porcelain tile, IQS Barcelona |
| ArUco dict | DICT_4X4_50, ID=0, 9.58 cm |
| Camera | iPhone XR, 1080×1920 @ 30fps |
| Camera RMS | 0.05 px |

---

## License

MIT — see [LICENSE](LICENSE)

---

*Part of the hospital AGV autonomy stack developed for the Social Tech Challenge, IQS School of Engineering, Universitat Ramon Llull.*
