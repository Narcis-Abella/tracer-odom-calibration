#!/usr/bin/env python3
"""
merge_calibracion.py
Cruza aruco_iters.csv (ángulo real del ArUco) con resultados_calibracion.csv
(ángulo de odometría del Jetson) usando los sync moves para sincronizar
los timestamps del vídeo con los timestamps Unix del Jetson.

Uso:
    python3 merge_calibracion.py --aruco aruco_iters.csv
                                 --sync  aruco_iters_sync.csv
                                 --odom  resultados_calibracion.csv
                                 --out   calibracion_merged.csv
"""

import csv
import argparse
import os

# ─── Carga de CSVs ────────────────────────────────────────────────────────────

def cargar_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))

# ─── Main ─────────────────────────────────────────────────────────────────────

def merge(aruco_path, sync_path, odom_path, out_path):
    print("\n═══════════════════════════════════════════")
    print("  MERGE ARUCO + ODOM")
    print("═══════════════════════════════════════════\n")

    aruco_runs = cargar_csv(aruco_path)
    aruco_sync = cargar_csv(sync_path)
    odom_rows  = cargar_csv(odom_path)

    # ── Extraer sync points ───────────────────────────────────────────────────
    odom_sync_start = next((r for r in odom_rows if r['tipo'] == 'SYNC_START'), None)
    odom_sync_end   = next((r for r in odom_rows if r['tipo'] == 'SYNC_END'),   None)
    odom_runs       = [r for r in odom_rows if r['tipo'] == 'RUN']

    if not odom_sync_start:
        print("❌ No se encontró SYNC_START en el CSV de odom.")
        return

    if len(aruco_sync) < 1:
        print("❌ No se encontró ningún sync move en aruco_iters_sync.csv.")
        return

    t_jetson_sync_start = float(odom_sync_start['t_inicio'])
    t_video_sync_start  = float(aruco_sync[0]['t_inicio_video'])

    # Offset: cuántos segundos hay que sumar al tiempo de vídeo para obtener Unix time
    offset = t_jetson_sync_start - t_video_sync_start

    print(f"Sync move inicial:")
    print(f"  t_jetson = {t_jetson_sync_start:.3f}")
    print(f"  t_video  = {t_video_sync_start:.3f}s")
    print(f"  offset   = {offset:.3f}s\n")

    # Verificar con sync end si existe
    if odom_sync_end and len(aruco_sync) >= 2:
        t_jetson_sync_end = float(odom_sync_end['t_inicio'])
        t_video_sync_end  = float(aruco_sync[-1]['t_inicio_video'])
        offset_end        = t_jetson_sync_end - t_video_sync_end
        drift             = offset_end - offset
        print(f"Sync move final:")
        print(f"  t_jetson = {t_jetson_sync_end:.3f}")
        print(f"  t_video  = {t_video_sync_end:.3f}s")
        print(f"  offset   = {offset_end:.3f}s")
        print(f"  drift    = {drift:.3f}s  ", end="")
        if abs(drift) < 0.5:
            print("(despreciable ✓)")
        elif abs(drift) < 2.0:
            print("(pequeño — usando offset promedio)")
            offset = (offset + offset_end) / 2
        else:
            print("⚠️  drift alto — revisa la sincronización")
        print()

    # ── Convertir tiempos de vídeo a Unix time ────────────────────────────────
    for run in aruco_runs:
        run['t_inicio_unix'] = float(run['t_inicio_video']) + offset
        run['t_fin_unix']    = float(run['t_fin_video'])    + offset

    # ── Cruce: para cada run de odom, buscar el run de ArUco correspondiente ──
    print(f"{'#':>3}  {'vel':>5}  {'acel':>5}  {'obj':>6}  {'dir':>5}  {'odom°':>8}  {'real°':>8}  {'match':>6}")
    print("─" * 55)

    resultados = []
    sin_match  = 0

    for odom in odom_runs:
        t_odom = float(odom['t_inicio'])

        # Buscar el run de ArUco cuyo intervalo [t_inicio_unix, t_fin_unix]
        # contiene t_odom, con tolerancia de ±2s
        match = None
        mejor_distancia = float('inf')

        for aruco in aruco_runs:
            t_ini = aruco['t_inicio_unix']
            t_fin = aruco['t_fin_unix']
            # El t_odom del Jetson es el momento en que TERMINA el run
            # así que buscamos que caiga dentro o justo después del intervalo
            if t_ini - 2.0 <= t_odom <= t_fin + 3.0:
                distancia = abs(t_odom - t_fin)
                if distancia < mejor_distancia:
                    mejor_distancia = distancia
                    match = aruco

        if match:
            angulo_odom = float(odom['angulo_odom'])
            angulo_real = abs(float(match['delta_theta_real']))
            direccion   = match['signo']

            resultados.append({
                'n':               len(resultados) + 1,
                'vel_max':         odom['vel_max'],
                'acel':            odom['acel'],
                'angulo_objetivo': odom['angulo_objetivo'],
                'direccion':       direccion,
                'angulo_odom':     round(angulo_odom, 3),
                'angulo_real':     round(angulo_real, 3),
            })

            print(f"  {len(resultados):>2}  {odom['vel_max']:>5}  {odom['acel']:>5}  "
                  f"{odom['angulo_objetivo']:>6}  {direccion:>5}  {angulo_odom:>8.3f}  {angulo_real:>8.3f}  ✓")
        else:
            sin_match += 1
            print(f"  ?  {odom['vel_max']:>5}  {odom['acel']:>5}  "
                  f"{odom['angulo_objetivo']:>6}  {float(odom['angulo_odom']):>8.3f}"
                  f"{'':>30}  ✗ sin match (t={t_odom:.1f})")

    print(f"\nMatches: {len(resultados)}/{len(odom_runs)}  |  Sin match: {sin_match}")

    if not resultados:
        print("\n❌ Ningún match encontrado.")
        print("   Revisa que los archivos corresponden al mismo experimento.")
        print("   Prueba aumentar la tolerancia de búsqueda en el script (±2s/±3s).")
        return

    # ── Guardar CSV final ─────────────────────────────────────────────────────
    campos = ['n', 'vel_max', 'acel', 'angulo_objetivo', 'direccion', 'angulo_odom', 'angulo_real']

    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=campos)
        writer.writeheader()
        writer.writerows(resultados)

    print(f"\n✓ Resultado guardado en: {out_path}")
    print("═══════════════════════════════════════════\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Merge ArUco + odom CSV')
    parser.add_argument('--aruco', default='aruco_iters.csv',       help='CSV runs ArUco')
    parser.add_argument('--sync',  default='aruco_iters_sync.csv',  help='CSV sync moves ArUco')
    parser.add_argument('--odom',  default='resultados_calibracion.csv', help='CSV odom Jetson')
    parser.add_argument('--out',   default='calibracion_merged.csv', help='CSV de salida')
    args = parser.parse_args()

    merge(args.aruco, args.sync, args.odom, args.out)