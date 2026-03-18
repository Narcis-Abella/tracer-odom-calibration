#!/usr/bin/env python3
import rospy
import math
import csv
import os
import time
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# --- Configuracion -----------------------------------------------------------
DEBUG    = True
RATE_HZ  = 50
PAUSA    = 5.0

# Sync move: giro brusco CW 30 ida y vuelta
SYNC_VEL    = 2.0
SYNC_ANGULO = math.radians(30)

# --- BLOQUE 2 -- 180 grados  (270 runs, ~51 min) ---
CSV_FILE = "resultados_bloque2_180.csv"

# PLAN: (vel_max, acel, angulo_deg, repeticiones, direccion)
#   direccion: +1 = CCW, -1 = CW
PLAN = []
# -- 180 grados --
PLAN.append((0.3, 0.5, 180, 5, +1))
PLAN.append((0.3, 0.5, 180, 5, -1))
PLAN.append((0.3, 1.0, 180, 5, +1))
PLAN.append((0.3, 1.0, 180, 5, -1))
PLAN.append((0.3, 2.0, 180, 5, +1))
PLAN.append((0.3, 2.0, 180, 5, -1))
PLAN.append((0.35, 0.5, 180, 5, +1))
PLAN.append((0.35, 0.5, 180, 5, -1))
PLAN.append((0.35, 1.0, 180, 5, +1))
PLAN.append((0.35, 1.0, 180, 5, -1))
PLAN.append((0.35, 2.0, 180, 5, +1))
PLAN.append((0.35, 2.0, 180, 5, -1))
PLAN.append((0.4, 0.5, 180, 5, +1))
PLAN.append((0.4, 0.5, 180, 5, -1))
PLAN.append((0.4, 1.0, 180, 5, +1))
PLAN.append((0.4, 1.0, 180, 5, -1))
PLAN.append((0.4, 2.0, 180, 5, +1))
PLAN.append((0.4, 2.0, 180, 5, -1))
PLAN.append((0.5, 0.5, 180, 5, +1))
PLAN.append((0.5, 0.5, 180, 5, -1))
PLAN.append((0.5, 1.0, 180, 5, +1))
PLAN.append((0.5, 1.0, 180, 5, -1))
PLAN.append((0.5, 2.0, 180, 5, +1))
PLAN.append((0.5, 2.0, 180, 5, -1))
PLAN.append((0.6, 0.5, 180, 5, +1))
PLAN.append((0.6, 0.5, 180, 5, -1))
PLAN.append((0.6, 1.0, 180, 5, +1))
PLAN.append((0.6, 1.0, 180, 5, -1))
PLAN.append((0.6, 2.0, 180, 5, +1))
PLAN.append((0.6, 2.0, 180, 5, -1))
PLAN.append((0.8, 0.5, 180, 5, +1))
PLAN.append((0.8, 0.5, 180, 5, -1))
PLAN.append((0.8, 1.0, 180, 5, +1))
PLAN.append((0.8, 1.0, 180, 5, -1))
PLAN.append((0.8, 2.0, 180, 5, +1))
PLAN.append((0.8, 2.0, 180, 5, -1))
PLAN.append((1.0, 0.5, 180, 5, +1))
PLAN.append((1.0, 0.5, 180, 5, -1))
PLAN.append((1.0, 1.0, 180, 5, +1))
PLAN.append((1.0, 1.0, 180, 5, -1))
PLAN.append((1.0, 2.0, 180, 5, +1))
PLAN.append((1.0, 2.0, 180, 5, -1))
PLAN.append((1.5, 0.5, 180, 5, +1))
PLAN.append((1.5, 0.5, 180, 5, -1))
PLAN.append((1.5, 1.0, 180, 5, +1))
PLAN.append((1.5, 1.0, 180, 5, -1))
PLAN.append((1.5, 2.0, 180, 5, +1))
PLAN.append((1.5, 2.0, 180, 5, -1))
PLAN.append((2.0, 0.5, 180, 5, +1))
PLAN.append((2.0, 0.5, 180, 5, -1))
PLAN.append((2.0, 1.0, 180, 5, +1))
PLAN.append((2.0, 1.0, 180, 5, -1))
PLAN.append((2.0, 2.0, 180, 5, +1))
PLAN.append((2.0, 2.0, 180, 5, -1))

# ─── Subscriber persistente ───────────────────────────────────────────────────
_last_odom  = None
_odom_event = None
_odom_count = 0

def _odom_cb(msg):
    global _last_odom, _odom_count
    _last_odom = msg
    _odom_count += 1
    if _odom_event is not None:
        _odom_event.set()

def get_yaw():
    global _odom_event
    _odom_event = threading.Event()
    triggered = _odom_event.wait(timeout=5.0)
    _odom_event = None
    if not triggered or _last_odom is None:
        raise RuntimeError("Timeout esperando mensaje nuevo de /odom")
    z = _last_odom.pose.pose.orientation.z
    w = _last_odom.pose.pose.orientation.w
    return 2.0 * math.atan2(z, w)

# ─── Utilidades ───────────────────────────────────────────────────────────────
def normalizar_angulo(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def dbg(msg):
    if DEBUG:
        print("  [DEBUG] " + msg)

# ─── Funcion de giro ──────────────────────────────────────────────────────────
def girar(pub, angulo_objetivo_rad, vel_max, acel, direccion):
    cmd          = Twist()
    angulo_rampa = 0.5 * vel_max**2 / acel
    dbg("Iniciando: obj={:.1f} vel_max={} acel={} rampa={:.1f} dir={}".format(
        math.degrees(angulo_objetivo_rad), vel_max, acel,
        math.degrees(angulo_rampa), "CCW" if direccion > 0 else "CW"))
    yaw_prev      = get_yaw()
    vel           = 0.0
    girado        = 0.0
    dt            = 1.0 / RATE_HZ
    iteracion     = 0
    ultimo_girado = 0.0
    odom_prev     = _odom_count
    ha_acelerado  = False
    dbg("yaw inicial: {:.3f} rad ({:.1f})".format(yaw_prev, math.degrees(yaw_prev)))
    while not rospy.is_shutdown():
        yaw_actual = get_yaw()
        delta      = normalizar_angulo(yaw_actual - yaw_prev)
        girado    += abs(delta)
        yaw_prev   = yaw_actual
        restante   = angulo_objetivo_rad - girado
        iteracion += 1
        if DEBUG and iteracion % 50 == 0:
            avance = girado - ultimo_girado
            print("  [DEBUG] iter={} girado={:.2f} restante={:.2f} vel={:.4f} cmd_z={:.4f} avance_1s={:.3f} odom_msgs={}".format(
                iteracion,
                math.degrees(girado),
                math.degrees(max(restante, 0)),
                vel,
                direccion * vel,
                math.degrees(avance),
                _odom_count - odom_prev
            ))
            if vel > 0.04 and math.degrees(avance) < 0.5:
                print("  [DEBUG] Robot no avanza aunque vel={:.3f} -- CAN caido?".format(vel))
            ultimo_girado = girado
            odom_prev     = _odom_count
        if restante <= 0:
            vel_objetivo = 0.0
        elif restante <= angulo_rampa:
            vel_objetivo = min(math.sqrt(2.0 * acel * restante), vel_max)
        else:
            vel_objetivo = vel_max
        if vel < vel_objetivo:
            vel = min(vel + acel * dt, vel_objetivo)
        else:
            vel = vel_objetivo
        vel = max(vel, 0.0)
        cmd.angular.z = direccion * vel
        cmd.linear.x  = 0.0
        pub.publish(cmd)
        if vel >= vel_max * 0.5:
            ha_acelerado = True
        if ha_acelerado and vel < 0.04:
            dbg("Velocidad minima alcanzada con girado={:.2f} -- dando por completado".format(math.degrees(girado)))
            break
        if girado >= angulo_objetivo_rad:
            dbg("Completado: girado={:.2f} en {} iters".format(math.degrees(girado), iteracion))
            break
    cmd.angular.z = 0.0
    pub.publish(cmd)
    rospy.sleep(0.3)
    return math.degrees(girado)

# ─── Sync move: giro brusco CW 30 ida y vuelta ───────────────────────────────
def sync_move(pub):
    cmd = Twist()
    print("  [SYNC] Ejecutando sync move...")
    t_sync = time.time()
    # Ida: CW 30
    girado   = 0.0
    yaw_prev = get_yaw()
    while girado < SYNC_ANGULO and not rospy.is_shutdown():
        yaw_actual = get_yaw()
        delta      = abs(normalizar_angulo(yaw_actual - yaw_prev))
        girado    += delta
        yaw_prev   = yaw_actual
        cmd.angular.z = -SYNC_VEL
        pub.publish(cmd)
    # Vuelta: CCW 30
    girado   = 0.0
    yaw_prev = get_yaw()
    while girado < SYNC_ANGULO and not rospy.is_shutdown():
        yaw_actual = get_yaw()
        delta      = abs(normalizar_angulo(yaw_actual - yaw_prev))
        girado    += delta
        yaw_prev   = yaw_actual
        cmd.angular.z = +SYNC_VEL
        pub.publish(cmd)
    cmd.angular.z = 0.0
    pub.publish(cmd)
    rospy.sleep(0.3)
    print("  [SYNC] t_sync = {:.6f}".format(t_sync))
    return t_sync

# ─── CSV ──────────────────────────────────────────────────────────────────────
def guardar_csv(fila):
    existe = os.path.exists(CSV_FILE)
    with open(CSV_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        if not existe:
            writer.writerow(['tipo', 't_inicio', 't_fin', 'vel_max', 'acel', 'angulo_objetivo', 'direccion', 'angulo_odom'])
        writer.writerow(fila)

# ─── Main ─────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    rospy.init_node('calibracion_automatica_tracer')
    rospy.Subscriber('/odom', Odometry, _odom_cb)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    dbg("Esperando primeros mensajes de /odom...")
    rospy.sleep(2.0)
    dbg("Mensajes /odom recibidos: {}".format(_odom_count))
    if DEBUG and _odom_count == 0:
        print("  [DEBUG] No se ha recibido ningun mensaje de /odom")
    total  = sum(n for _, _, _, n, _ in PLAN)
    cuenta = 0
    print("\n" + "=" * 50)
    print("  CALIBRACION AUTOMATICA -- {} mediciones".format(total))
    print("=" * 50 + "\n")
    # ── Sync move inicial ────────────────────────────────────────────────────
    print("-- Sync move INICIAL --")
    t_sync_inicio = sync_move(pub)
    guardar_csv(['SYNC_START', t_sync_inicio, t_sync_inicio, '', '', '', '', ''])
    print("  Pausa 2s antes de empezar...\n")
    rospy.sleep(2.0)
    # ── Plan de mediciones ───────────────────────────────────────────────────
    for vel_max, acel, angulo_deg, repeticiones, direccion in PLAN:
        angulo_rad  = math.radians(angulo_deg)
        dir_str     = "CCW" if direccion > 0 else "CW"
        print("-- Serie: vel={} acel={} angulo={} {} x{} --".format(
            vel_max, acel, angulo_deg, dir_str, repeticiones))
        for i in range(repeticiones):
            cuenta += 1
            print("  [{}/{}] Midiendo...".format(cuenta, total))
            t_inicio    = time.time()
            girado_odom = girar(pub, angulo_rad, vel_max, acel, direccion)
            t_fin       = time.time()
            guardar_csv(['RUN', t_inicio, t_fin, vel_max, acel, angulo_deg, dir_str, round(girado_odom, 3)])
            print("  /odom: {:.2f}  ->  guardado  [t={:.3f}-{:.3f}]".format(
                girado_odom, t_inicio, t_fin))
            if cuenta < total:
                print("  Pausa {} s...\n".format(PAUSA))
                rospy.sleep(PAUSA)
    # ── Sync move final ──────────────────────────────────────────────────────
    print("\n-- Sync move FINAL --")
    rospy.sleep(2.0)
    t_sync_fin = sync_move(pub)
    guardar_csv(['SYNC_END', t_sync_fin, t_sync_fin, '', '', '', '', ''])
    print("\nTodas las mediciones completadas.")
    print("  Resultados en: {}/{}".format(os.getcwd(), CSV_FILE))
