#!/usr/bin/env python3
"""
calibrar_camara.py
Calibración intrínseca de cámara a partir de un vídeo con tablero de ajedrez.
Uso: python3 calibrar_camara.py --video ruta/al/video.mov
"""

import cv2
import numpy as np
import json
import argparse
import os

# ─── Configuración del tablero ────────────────────────────────────────────────
# Tablero oficial OpenCV: 10 cols x 7 filas de cuadrados = 9x6 esquinas interiores
COLS_ESQUINAS = 9
FILAS_ESQUINAS = 6

# Cada cuántos frames intentar detectar (no hace falta procesar todos)
SAMPLE_EVERY_N_FRAMES = 15

# Mínimo de detecciones buenas para considerar la calibración válida
MIN_DETECCIONES = 25

# ─── Main ─────────────────────────────────────────────────────────────────────
def calibrar(video_path):
    print("\n═══════════════════════════════════════════")
    print("  CALIBRACIÓN DE CÁMARA")
    print("═══════════════════════════════════════════\n")

    if not os.path.exists(video_path):
        print(f"❌ No se encuentra el vídeo: {video_path}")
        return

    # Puntos 3D del tablero en el sistema de coordenadas del tablero
    # (z=0 porque el tablero es plano)
    objp = np.zeros((FILAS_ESQUINAS * COLS_ESQUINAS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:COLS_ESQUINAS, 0:FILAS_ESQUINAS].T.reshape(-1, 2)

    obj_points = []  # puntos 3D reales
    img_points = []  # puntos 2D en imagen

    cap = cv2.VideoCapture(video_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Vídeo: {os.path.basename(video_path)}")
    print(f"Resolución: {width}x{height}  |  FPS: {fps:.1f}  |  Frames totales: {total_frames}")
    print(f"Tablero buscado: {COLS_ESQUINAS}x{FILAS_ESQUINAS} esquinas interiores")
    print(f"Muestreando cada {SAMPLE_EVERY_N_FRAMES} frames...\n")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    frame_idx    = 0
    detecciones  = 0
    procesados   = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_idx += 1

        if frame_idx % SAMPLE_EVERY_N_FRAMES != 0:
            continue

        procesados += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(
            gray,
            (COLS_ESQUINAS, FILAS_ESQUINAS),
            None
        )

        if found:
            detecciones += 1
            # Refinar esquinas a subpíxel
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            obj_points.append(objp)
            img_points.append(corners_refined)
            status = "✓"
        else:
            status = "✗"

        if procesados % 10 == 0 or found:
            pct = (frame_idx / total_frames) * 100
            print(f"  Frame {frame_idx:5d}/{total_frames} ({pct:4.1f}%)  {status}  detecciones: {detecciones}")

    cap.release()

    print(f"\nFrames procesados: {procesados}")
    print(f"Detecciones válidas: {detecciones}")

    if detecciones < MIN_DETECCIONES:
        print(f"\n⚠️  Solo {detecciones} detecciones — mínimo recomendado: {MIN_DETECCIONES}")
        print("   Graba otro vídeo con más variedad de ángulos y distancias.")
        return

    print("\nCalculando calibración...")
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (width, height), None, None
    )

    # Error de reproyección (cuanto más bajo mejor, <1.0 es bueno)
    error_total = 0
    for i in range(len(obj_points)):
        img_points_reproj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        error = cv2.norm(img_points[i], img_points_reproj, cv2.NORM_L2) / len(img_points_reproj)
        error_total += error
    rms = error_total / len(obj_points)

    print(f"\n─── Resultados ───────────────────────────────")
    print(f"RMS error de reproyección: {rms:.4f} px  ", end="")
    if rms < 0.5:
        print("(excelente ✓)")
    elif rms < 1.0:
        print("(bueno ✓)")
    else:
        print("(alto — considera repetir la calibración)")

    print(f"\nMatriz intrínseca K:")
    print(f"  fx = {K[0,0]:.2f} px")
    print(f"  fy = {K[1,1]:.2f} px")
    print(f"  cx = {K[0,2]:.2f} px")
    print(f"  cy = {K[1,2]:.2f} px")

    print(f"\nCoeficientes de distorsión:")
    print(f"  {dist.ravel().tolist()}")

    # Guardar en JSON
    output = {
        "resolución": f"{width}x{height}",
        "fps": fps,
        "detecciones_usadas": detecciones,
        "rms_reproyeccion_px": round(rms, 4),
        "K": K.tolist(),
        "dist_coeffs": dist.tolist()
    }

    json_path = "calibracion_camara.json"
    with open(json_path, 'w') as f:
        json.dump(output, f, indent=2)

    print(f"\n✓ Calibración guardada en: {json_path}")
    print("═══════════════════════════════════════════\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibración intrínseca de cámara')
    parser.add_argument('--video', required=True, help='Ruta al vídeo .mov/.mp4')
    args = parser.parse_args()
    calibrar(args.video)