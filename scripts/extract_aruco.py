#!/usr/bin/env python3
"""
extract_aruco.py
Extrae el ángulo de rotación de un marcador ArUco frame a frame,
acumula el ángulo con unwrapping, segmenta iteraciones por pausas,
y exporta un CSV con delta_theta_real por iteración.

Uso:
    python3 extract_aruco.py --video ruta/video.mov --calib calibracion_camara.json
"""

import cv2
import numpy as np
import json
import argparse
import os
import csv
from collections import deque

# ─── Configuración ArUco ──────────────────────────────────────────────────────
ARUCO_DICT_ID  = cv2.aruco.DICT_4X4_50
MARKER_ID      = 0
MARKER_SIZE_M  = 0.0958  # metros — medido físicamente

# ─── Configuración segmentación ───────────────────────────────────────────────
PAUSA_MIN_SEG      = 2.0    # segundos quieto para detectar pausa
PAUSA_UMBRAL_DEG   = 4.0    # grados/s — por debajo de esto = quieto
PAUSA_MIN_SEG_POST = 1.0    # segundos mínimos de movimiento para cerrar iteración
SAMPLE_EVERY_N_FRAMES = 5   # procesa 1 de cada n frames

# ─── Configuración detección ──────────────────────────────────────────────────
MAX_FRAMES_SIN_DETECCION = 10  # frames consecutivos sin ArUco antes de advertir

# ─── Utilidades ───────────────────────────────────────────────────────────────
def cargar_calibracion(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    K    = np.array(data['K'])
    dist = np.array(data['dist_coeffs'])
    return K, dist

def rotation_matrix_to_angle_z(rvec):
    """Convierte rvec (Rodrigues) a ángulo de rotación en Z (yaw) en grados."""
    R, _ = cv2.Rodrigues(rvec)
    # Ángulo yaw: rotación en el plano XY de la cámara
    angle = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return angle

def unwrap_delta(delta):
    """Corrige saltos de ±360° en el delta frame a frame."""
    while delta >  180: delta -= 360
    while delta < -180: delta += 360
    return delta

# ─── Main ─────────────────────────────────────────────────────────────────────
def procesar(video_path, calib_path, output_csv):
    print("\n═══════════════════════════════════════════")
    print("  EXTRACCIÓN ÁNGULO ArUco")
    print("═══════════════════════════════════════════\n")

    K, dist = cargar_calibracion(calib_path)
    print(f"Calibración cargada: {calib_path}")

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"❌ No se puede abrir: {video_path}")
        return

    fps           = cap.get(cv2.CAP_PROP_FPS)
    total_frames  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duracion_s    = total_frames / fps
    width         = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height        = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Vídeo: {os.path.basename(video_path)}")
    print(f"Resolución: {width}x{height}  |  FPS: {fps:.1f}  |  Duración: {duracion_s:.1f}s\n")

    # Inicializar detector ArUco
    aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # ── Serie temporal de ángulo acumulado ────────────────────────────────────
    # Lista de (t_seg, theta_acumulado)
    serie = []

    theta_acum    = 0.0
    theta_prev    = None
    frame_idx     = 0
    sin_deteccion = 0

    print("Procesando frames...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_idx += 1
        
        if frame_idx % SAMPLE_EVERY_N_FRAMES != 0:
            continue
        
        t = frame_idx / fps  # tiempo en segundos desde inicio del vídeo

        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        detectado = False
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                if mid == MARKER_ID:
                    detectado = True
                    sin_deteccion = 0

                    # Estimar pose
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i:i+1], MARKER_SIZE_M, K, dist
                    )
                    theta = rotation_matrix_to_angle_z(rvec[0])

                    if theta_prev is not None:
                        delta = unwrap_delta(theta - theta_prev)
                        theta_acum += delta

                    theta_prev = theta
                    serie.append((t, theta_acum))
                    break

        if not detectado:
            sin_deteccion += 1
            if sin_deteccion == MAX_FRAMES_SIN_DETECCION:
                t_s = int(t)
                print(f"  ⚠️  {MAX_FRAMES_SIN_DETECCION} frames sin detectar ArUco (t={t:.1f}s)")

        if frame_idx % 300 == 0:
            pct = (frame_idx / total_frames) * 100
            print(f"  Frame {frame_idx:5d}/{total_frames} ({pct:4.1f}%)  θ_acum={theta_acum:.2f}°")

    cap.release()
    print(f"\nFrames procesados: {frame_idx}")
    print(f"Detecciones ArUco: {len(serie)}")

    if len(serie) < 10:
        print("❌ Muy pocas detecciones — revisa que el ArUco es DICT_4X4_50 ID=0")
        return

    # ── Segmentación por pausas ───────────────────────────────────────────────
    print(f"\nSegmentando iteraciones (pausa > {PAUSA_MIN_SEG}s a vel < {PAUSA_UMBRAL_DEG}°/s)...")

    ventana_frames = int(PAUSA_MIN_SEG * fps / 1)  # frames en la ventana de pausa
    # Convertimos serie a arrays para facilitar el procesado
    tiempos = np.array([s[0] for s in serie])
    thetas  = np.array([s[1] for s in serie])

    # Velocidad angular frame a frame (°/s)
    vel_angular = np.zeros(len(serie))
    for i in range(1, len(serie)):
        dt = tiempos[i] - tiempos[i-1]
        if dt > 0:
            vel_angular[i] = abs(thetas[i] - thetas[i-1]) / dt
            
    # DEBUG temporal — borrar después
    print(f"vel_angular stats: max={vel_angular.max():.2f} mean={vel_angular.mean():.2f} "
        f"percentil5={np.percentile(vel_angular,5):.2f} percentil95={np.percentile(vel_angular,95):.2f}")
    print(f"Frames con vel<4°/s: {(vel_angular < 4.0).sum()} de {len(vel_angular)}")
    print(f"Frames con vel<20°/s: {(vel_angular < 20.0).sum()} de {len(vel_angular)}")

    # Detectar zonas de pausa: ventana deslizante
    en_pausa      = np.zeros(len(serie), dtype=bool)
    n_ventana     = max(1, int(PAUSA_MIN_SEG * fps / SAMPLE_EVERY_N_FRAMES))

    for i in range(len(serie)):
        # Ventana hacia adelante
        i_fin = min(i + n_ventana, len(serie))
        ventana_vel = vel_angular[i:i_fin]
        dt_ventana  = tiempos[min(i_fin-1, len(tiempos)-1)] - tiempos[i]
        if dt_ventana >= PAUSA_MIN_SEG * 0.8 and np.mean(ventana_vel) < PAUSA_UMBRAL_DEG:
            en_pausa[i] = True

    # Encontrar transiciones pausa→movimiento como inicio de iteración
    # y movimiento→pausa como fin de iteración
    iteraciones = []
    en_iter      = False
    i_inicio     = 0
    theta_inicio = 0.0

    for i in range(1, len(serie)):
        if not en_pausa[i] and en_pausa[i-1]:
            # Inicio de movimiento
            en_iter      = True
            i_inicio     = i
            theta_inicio = thetas[i]

        elif en_pausa[i] and not en_pausa[i-1] and en_iter:
            # Fin de movimiento
            theta_fin   = thetas[i-1]
            delta_theta = theta_fin - theta_inicio
            t_inicio    = tiempos[i_inicio]
            t_fin       = tiempos[i-1]
            duracion    = t_fin - t_inicio

            if duracion > PAUSA_MIN_SEG_POST and abs(delta_theta) > 5.0:
                signo = "CCW" if delta_theta > 0 else "CW"
                iteraciones.append({
                    'iter_id':          len(iteraciones) + 1,
                    't_inicio_video':   round(t_inicio, 3),
                    't_fin_video':      round(t_fin, 3),
                    'delta_theta_real': round(delta_theta, 3),
                    'signo':            signo,
                    'duracion_s':       round(duracion, 2)
                })
                en_iter = False

    print(f"Iteraciones detectadas: {len(iteraciones)}\n")

    if not iteraciones:
        print("⚠️  No se detectaron iteraciones.")
        print("   Ajusta PAUSA_UMBRAL_DEG o PAUSA_MIN_SEG si el robot no paraba completamente.")
        return

    # ── Buscar sync moves ─────────────────────────────────────────────────────
    # Los sync moves son movimientos cortos (<5s) con delta pequeño (~30°)
    # Los separamos de las iteraciones reales (>90°)
    runs_reales = [it for it in iteraciones if abs(it['delta_theta_real']) > 90]
    sync_moves  = [it for it in iteraciones if abs(it['delta_theta_real']) <= 90]

    print(f"  Runs reales (>90°): {len(runs_reales)}")
    print(f"  Sync moves (<90°): {len(sync_moves)}")

    # Mostrar resumen
    print(f"\n{'ID':>4}  {'t_ini':>8}  {'t_fin':>8}  {'Δθ_real':>10}  {'signo':>5}  {'dur':>6}")
    print("─" * 55)
    for it in runs_reales:
        print(f"  {it['iter_id']:>2}  {it['t_inicio_video']:>8.1f}s  {it['t_fin_video']:>8.1f}s  "
              f"{it['delta_theta_real']:>9.2f}°  {it['signo']:>5}  {it['duracion_s']:>5.1f}s")

    # ── Guardar CSV ───────────────────────────────────────────────────────────
    with open(output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'iter_id', 't_inicio_video', 't_fin_video',
            'delta_theta_real', 'signo', 'duracion_s'
        ])
        writer.writeheader()
        writer.writerows(runs_reales)

    # Guardar también sync moves por separado (útil para sincronización)
    sync_csv = output_csv.replace('.csv', '_sync.csv')
    with open(sync_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'iter_id', 't_inicio_video', 't_fin_video',
            'delta_theta_real', 'signo', 'duracion_s'
        ])
        writer.writeheader()
        writer.writerows(sync_moves)

    print(f"\n✓ Runs guardados en:       {output_csv}")
    print(f"✓ Sync moves guardados en: {sync_csv}")
    print("═══════════════════════════════════════════\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extracción ángulo ArUco + segmentación')
    parser.add_argument('--video',  required=True,                      help='Ruta al vídeo .mov/.mp4')
    parser.add_argument('--calib',  default='calibracion_camara.json',  help='JSON de calibración de cámara')
    parser.add_argument('--output', default='aruco_iters.csv',          help='CSV de salida')
    args = parser.parse_args()

    procesar(args.video, args.calib, args.output)