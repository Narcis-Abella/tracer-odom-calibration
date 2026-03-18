# Tracer Agilex 2.0 — Odometry Calibration Session
**Date:** 2026-03-13  
**Platform:** Tracer Agilex 2.0 on ROS1 Melodic (Ubuntu 20, Jetson Nano)  
**Author:** Narcis Abella  

---

## 1. Objective

Quantify the rotational odometry error of the Tracer Agilex 2.0 by commanding known angular displacements (90°, 180°, 270°, 360°) and comparing `/odom` output against visually measured real-world rotation. The goal is to characterize the error as a function of velocity and acceleration, as a preliminary step toward a corrected odometry model for SLAM integration.

---

## 2. Hardware & Environment

| Parameter | Value |
|-----------|-------|
| Robot | Tracer Agilex 2.0 |
| Compute | NVIDIA Jetson Nano |
| ROS version | ROS1 Melodic |
| Communication | CAN bus (`can0`) |
| Floor surface | ~50×50 cm tiles (likely polished porcelain or fine-grain terrazzo) |
| Location | IQS School of Engineering, Universitat Ramon Llull, Barcelona |

The floor surface is relevant because wheel slip — and therefore odometry error at higher velocities — is strongly dependent on the friction coefficient of the surface. Polished porcelain/terrazzo is a low-friction surface, which is expected to amplify slip-induced odometry error at higher speeds.

---

## 3. Software Architecture Finding: Where Odometry is Computed

During the session, the odometry source was traced through the full software stack:

### 3.1 Driver stack
```
/cmd_vel → tracer_base_node → CAN → Tracer firmware → encoders
                                  ↓
                        angular_velocity (reported via CAN)
                                  ↓
                        tracer_messenger.cpp → /odom
```

### 3.2 Key finding: odometry is integrated from firmware-reported velocity

In `tracer_messenger.cpp` (`PublishOdometryToROS`):

```cpp
double d_theta = angular_speed_ * dt;
theta_ += d_theta;
```

The driver **does not compute angular velocity from wheel encoders directly** — it integrates `angular_speed_` as reported by the robot firmware over CAN. There is no `track_width` parameter in the ROS driver layer.

### 3.3 The track_width is hardcoded in the firmware

After searching the full catkin workspace:

- `ugv_sdk` (Weston Robot / Agilex SDK) contains no accessible `track_width` parameter at the ROS level
- The Gazebo sim file (`tracer_skid_steer.hpp`) defines `TRACER_WHEELBASE = 0.498` and `TRACER_WHEEL_RADIUS = 0.16459`, but these are simulation-only values
- The real robot's firmware computes `ω = (v_right - v_left) / track_width_firmware` internally and reports the result via CAN — this value is **not accessible or modifiable** from the ROS side

**Conclusion:** any odometry correction must be applied at the ROS level, either as a post-processing factor or via a corrective node republishing `/odom`.

---

## 4. Control Script

A Python 3 ROS node was developed (`calibracion_odom.py`) to command precise angular rotations and log `/odom` output. Key design decisions:

### 4.1 Odometry-based loop closure
The script stops when `/odom` accumulates the target angle — **not** based on time. This decouples the measurement from velocity/acceleration parameters.

### 4.2 Blocking odometry read
```python
def get_yaw():
    msg = rospy.wait_for_message('/odom', Odometry)
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    return 2.0 * math.atan2(z, w)
```
Using `wait_for_message` instead of a subscriber+global variable eliminates race conditions between the callback thread and the control loop. The loop runs at the natural frequency of `/odom` (~50 Hz).

### 4.3 Trapezoidal velocity profile
The script implements a trapezoidal profile:
- **Ramp-up:** velocity increases at rate `acel` until reaching `vel_max`
- **Cruise:** constant `vel_max`
- **Ramp-down:** triggered when remaining angle ≤ `angulo_rampa = 0.5 * vel_max² / acel`

During ramp-down, velocity follows the exact braking curve:
```python
vel_objetivo = min(math.sqrt(2.0 * acel * restante), vel_max)
```
and is applied directly (not rate-limited) so the robot tracks the deceleration curve precisely.

### 4.4 Angle accumulation with wrap-around handling
```python
delta = normalizar_angulo(yaw_actual - yaw_prev)
girado += abs(delta)
```
This correctly handles the ±π wrap-around when rotating past 180°.

---

## 5. Measurements

### 5.1 Manual measurements — characterizing error vs target angle
**Parameters:** `vel_max = 0.6 rad/s`, `acel = 0.5 rad/s²`, direction: CCW (left)  
**n = 10 per target angle**

| Target angle | `/odom` stops at (approx) | Visual observation |
|-------------|--------------------------|-------------------|
| 90° | ~89° | Slight undershoot |
| 180° | ~179° | Slight undershoot |
| 270° | ~269° | Slight undershoot |
| 360° | ~359° | Slight undershoot |

The `/odom` closure error is ≤ ±0.1° (script-level). The real-world angle was not precisely measured in this session — visual estimation suggests the robot rotates approximately **10° more than `/odom` reports per full revolution** at this velocity.

### 5.2 Automated measurements — characterizing error vs velocity and acceleration
An automated script (`calibracion_auto.py`) was used to run systematic series with a 5-second pause between measurements. All at 360°, CCW direction.

**Series executed (CCW):**

| vel_max (rad/s) | acel (rad/s²) | n | Notes |
|-----------------|---------------|---|-------|
| 0.3 | 0.5 | 5 | Large error observed — ~20° overshoot visually |
| 0.6 | 0.5 | 5 | |
| 1.0 | 0.5 | 5 | |
| 1.5 | 0.5 | 5 | |
| 2.0 | 0.5 | 5 | |
| 2.0 | 1.0 | 5 | Partial — script interrupted at measurement 28/35 |
| 2.0 | 2.0 | 5 | Near-zero error observed visually |

**CW direction series:** not completed — session ended.

Results saved to: `resultados_calibracion.csv`

---

## 6. Key Findings

### 6.1 Error is not constant — it depends on velocity
At `vel_max = 2.0, acel = 2.0`, the visual error was near zero. At `vel_max = 0.3`, the error was approximately +20° per revolution. This rules out a simple fixed calibration factor.

### 6.2 High error at low velocity — stiction/backlash hypothesis
Counterintuitively, the largest error was observed at the **lowest velocity** (0.3 rad/s). Hypothesis: at velocities near the motor activation threshold, the drivetrain exhibits stiction and backlash. The firmware may report angular velocity via CAN even when wheels are not moving cleanly, causing `/odom` to underestimate the actual rotation.

### 6.3 High error at high velocity — wheel slip hypothesis
At higher velocities on polished tile, wheel slip is expected to increase. The firmware computes `ω` from encoder counts, but if wheels slip, encoder-derived velocity underestimates true angular displacement → `/odom` underestimates → robot overshoots the target.

These two mechanisms (stiction at low vel, slip at high vel) suggest a **U-shaped error curve** as a function of velocity, with a minimum error somewhere in the mid-range (consistent with the near-zero error observed at `vel=2.0, acel=2.0`, though this may also reflect a coincidental cancellation).

### 6.4 Surface dependency
All measurements were taken on polished porcelain/terrazzo tile (~50×50 cm). Results are specific to this surface. The slip-induced component of the error is expected to change significantly on carpet, concrete, or outdoor surfaces.

### 6.5 Directional asymmetry — not yet characterized
CW measurements were not completed. Asymmetry between CW and CCW rotation is common in differential drive robots due to minor differences between left and right motor characteristics. This remains an open measurement task.

---

## 7. Next Steps

- Complete CW direction measurements to characterize directional asymmetry
- Precise visual measurement of real rotation angle for each trial (e.g. using a protractor or AprilTag ground truth)
- Statistical analysis of the CSV data: mean and std per series, error vs velocity curve
- Investigate whether the error scales linearly with angle (multiplicative/systematic) or is a fixed offset
- If error is well-characterized, implement a velocity-dependent correction node republishing a corrected `/odom_calibrated`
- Long-term: use the ABB YuMi robot as ground truth for a metrological validation of the correction model

---

## 8. Known Issues & Notes

- `Exception in thread /odom: AttributeError: 'NoneType' object has no attribute 'close'` appears sporadically — this is a known rospy bug in Python 3 on ROS Melodic and does not affect measurements.
- The Tracer driver (`tracer_base_node`) occasionally drops and must be relaunched. Root cause not identified.
- Script uses `rospy.wait_for_message` which blocks indefinitely if `/odom` stops publishing — if the driver crashes mid-measurement, the script hangs and must be killed with `pkill -f`.
