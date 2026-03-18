# Data Pipeline

Step-by-step guide for running a measurement session and producing the merged dataset.

---

## Pre-session Checklist

- [ ] iPhone charged, enough storage (> 500 MB per block)
- [ ] Tripod fixed, robot centered in frame, ArUco clearly visible
- [ ] Good lighting — no motion blur on the ArUco at maximum velocity
- [ ] Jetson powered on, CAN interface up (`ip link show can0` → UP)
- [ ] ROS driver running (`roslaunch tracer_bringup tracer_robot_base.launch`)
- [ ] Launch ROS script **before** pressing record on iPhone
- [ ] Verify robot responds to `/cmd_vel` before starting

---

## Step 1 — Run the ROS Script (Jetson)

```bash
# Terminal 1 — CAN + driver
source ~/catkin_ws/devel/setup.bash
rosrun tracer_bringup setup_can2usb.bash
roslaunch tracer_bringup tracer_robot_base.launch --screen

# Terminal 2 — measurement script (one block at a time)
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/narcis_ws
python3 calibracion_bloque1_90.py
```

The script will:
1. Wait 2s for `/odom` messages
2. Execute the **sync move** (30° CW + 30° CCW, abrupt, no ramp) — *start iPhone recording now if not already*
3. Run all measurement combinations in sequence, with 5s pauses between runs
4. Execute a final **sync move** at the end
5. Write results to `resultados_bloque1_90.csv`

**Important:** rename the output CSV immediately with the date:
```bash
cp resultados_bloque1_90.csv resultados_bloque1_90_YYYYMMDD.csv
```

---

## Step 2 — Transfer Files to Host

```powershell
# Transfer CSV from Jetson
scp jetson@<JETSON_IP>:/home/jetson/catkin_ws/src/narcis_ws/resultados_bloque1_90.csv `
    ".\data\resultados_bloque1_90_YYYYMMDD.csv"

# Transfer video from iPhone (via USB)
# Explorer → This PC → Apple iPhone → Internal Storage → DCIM
# Copy .MOV to media/
```

---

## Step 3 — Extract ArUco Ground Truth

```bash
python3 scripts/extract_aruco.py \
    --video  media/SESSION.mov \
    --calib  scripts/calibracion_camara.json \
    --output data/aruco_iters_YYYYMMDD.csv
```

**Outputs:**
- `aruco_iters_YYYYMMDD.csv` — one row per run: `t_inicio, t_fin, delta_theta_real`
- `aruco_iters_sync_YYYYMMDD.csv` — sync move timestamps (used for alignment)

**Verify:**
- Number of detected runs == number of `RUN` rows in the CSV from Jetson
- `Δθ_real` values are close to `angulo_objetivo` (tolerance: ±5° for high velocities, ±2° for low)
- Two sync events detected in `aruco_iters_sync_YYYYMMDD.csv`

**Tune if detection fails:**
```python
# In extract_aruco.py
PAUSA_UMBRAL_DEG = 5.0   # increase if robot vibrates during pauses
PAUSA_MIN_SEG    = 2.0   # decrease if pauses are shorter than expected
```

---

## Step 4 — Merge Odometry + ArUco

```bash
python3 scripts/merge_calibration.py \
    --aruco data/aruco_iters_YYYYMMDD.csv \
    --sync  data/aruco_iters_sync_YYYYMMDD.csv \
    --odom  data/resultados_bloque1_90_YYYYMMDD.csv \
    --out   data/merged_bloque1_90_YYYYMMDD.csv
```

**Verify:**
- `drift` between initial and final sync < 0.5s (printed to stdout)
- `Matches: N/N` — all runs matched between streams

**Output schema:**
```
n, vel_max, acel, angulo_objetivo, direccion, angulo_odom, angulo_real
1, 0.3, 0.5, 90, CCW, 89.958, 91.3
2, 0.3, 0.5, 90, CCW, 89.973, 91.1
...
```

The key columns are:
- `angulo_odom` — what `/odom` reported (degrees)
- `angulo_real` — what the ArUco measured as actual rotation (degrees)
- Error = `angulo_real − angulo_odom`

---

## Step 5 — Accumulate Sessions

After all 4 blocks are complete, combine into a single dataset:

```powershell
# PowerShell — merge all blocks into one dataset
$header = Get-Content data\merged_bloque1_90_YYYYMMDD.csv | Select-Object -First 1
$header | Out-File data\dataset_completo.csv -Encoding utf8

Get-ChildItem data\merged_bloque*.csv | ForEach-Object {
    Get-Content $_ | Select-Object -Skip 1
} | Add-Content data\dataset_completo.csv
```

Or on Linux/Mac:
```bash
head -1 data/merged_bloque1_90_YYYYMMDD.csv > data/dataset_completo.csv
tail -n +2 -q data/merged_bloque*.csv >> data/dataset_completo.csv
```

---

## Step 6 — Statistical Analysis

```bash
python3 scripts/robot_rotation_analyzer.py \
    --input data/dataset_completo.csv \
    --output results/
```

This generates:
- Mean error ± std per (ω, α, angle, direction) combination
- Error vs velocity curves (one per target angle)
- Directional asymmetry plots (CW vs CCW)
- Model fit parameters

---

## Reference Parameters

| Parameter | Value |
|---|---|
| ArUco dictionary | DICT_4X4_50 |
| Marker ID | 0 |
| Marker size | 9.58 cm (0.0958 m) |
| Video resolution | 1080×1920 @ 30fps |
| Camera calibration RMS | 0.05 px |
| Pause between runs | 5.0 s |
| Pre-session pause | 2.0 s |
| Sync move velocity | 2.0 rad/s |
| Sync move angle | 30° CW + 30° CCW |

---

## Troubleshooting

**Script hangs mid-session:**
`rospy.wait_for_message` blocks indefinitely if `/odom` stops. Kill with `pkill -f calibracion_bloque` and check CAN status.

**CAN dropped:**
```bash
watch -n 1 "ip -s link show can0"   # monitor dropped packets
# If dropped count rises fast → restart driver
killall -9 roslaunch && roslaunch tracer_bringup tracer_robot_base.launch
```

**ArUco not detected:**
Check lighting and marker orientation. The marker must be approximately face-on to the camera (< 60° tilt). Add a physical guide mark on the floor to ensure consistent robot starting position.

**Sync drift > 0.5s:**
Usually means the video was not recording during the sync move. Discard the session and re-run. Confirm the ROS script is launched before pressing record.
