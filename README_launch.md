# Launch Tracer Calibración — Flujo 3 Terminales

## Pre-requisito
Jetson reiniciada y robot encendido.

## Terminal 1 — CAN + Base node
```bash
source ~/catkin_ws/devel/setup.bash
rosrun tracer_bringup setup_can2usb.bash
ip link show can0  # debe aparecer UP
roslaunch tracer_bringup tracer_robot_base.launch --screen
```

## Terminal 2 — Verificación + Monitor CAN
Antes de lanzar el script, verifica que el robot responde:
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -r 10
```
Si gira físicamente → Ctrl+C y lanza el monitor:
```bash
watch -n 1 "ip -s link show can0"
```

## Terminal 3 — Script de calibración
```bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/narcis_ws
python3 calibracion_odom_auto.py
```

## Debug
En `calibracion_odom_auto.py` cambiar `DEBUG = False` a `DEBUG = True`.

## Si falla
1. Comprobar `dropped` en Terminal 2 — si sube rápido, problema de CAN
2. Comprobar que `can0` sigue `UP`
3. Matar todo y reiniciar desde Terminal 1:
```bash
killall -9 roslaunch
rosnode kill /tracer_base_node
```