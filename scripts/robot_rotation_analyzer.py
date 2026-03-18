#!/usr/bin/env python3
"""
robot_rotation_analyzer.py
───────────────────────────
Análisis de rotación del robot Tracer Agilex 2.0 a partir de clips de vídeo
grabados con iPhone XR (HEVC/H.265).

Para cada clip en una carpeta:
  1. Extrae el primer y último frame (ffmpeg como motor principal).
  2. Muestra ambos frames lado a lado.
  3. El usuario hace clic en los 2 extremos del rail de aluminio blanco en
     cada frame (4 clics en total, guiados con instrucciones en pantalla).
  4. Calcula el ángulo de orientación del vector rail en cada frame.
  5. Calcula la rotación Δ entre frame inicial y final.
  6. Guarda los resultados en un CSV.

Requisitos:
    pip install matplotlib numpy opencv-python-headless   # cv2 es opcional
    ffmpeg disponible en PATH  →  https://ffmpeg.org/download.html
                              →  Windows: winget install ffmpeg
                              →  macOS:   brew install ffmpeg
                              →  Linux:   sudo apt install ffmpeg

Uso:
    python robot_rotation_analyzer.py [ruta_carpeta_videos]
    # Si no se pasa carpeta, pregunta de forma interactiva.
"""

import os
import sys
import csv
import math
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Optional

# ── Backend de matplotlib (antes de importar pyplot) ─────────────────────────
import matplotlib
for _backend in ('TkAgg', 'Qt5Agg', 'QtAgg', 'WXAgg', 'Agg'):
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        continue

import matplotlib.pyplot as plt
import numpy as np

# ── cv2 opcional ──────────────────────────────────────────────────────────────
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# ── Constantes ────────────────────────────────────────────────────────────────
VIDEO_EXTENSIONS = {'.mp4', '.mov', '.avi', '.mkv', '.hevc', '.m4v'}
CSV_FILENAME = 'rotation_results.csv'

# Paleta de colores (tema oscuro)
_C = {
    'bg':      '#1e1e2e',
    'initial': '#89b4fa',   # azul
    'final':   '#f38ba8',   # rojo
    'A':       '#a6e3a1',   # verde
    'B':       '#f9e2af',   # amarillo
    'ok':      '#a6e3a1',
    'warn':    '#fab387',
    'text':    'white',
}

# ── Comprobación ffmpeg ───────────────────────────────────────────────────────

def check_ffmpeg() -> None:
    if shutil.which('ffmpeg') is None or shutil.which('ffprobe') is None:
        print(
            "\nERROR: ffmpeg / ffprobe no encontrado en PATH.\n"
            "  Windows : winget install ffmpeg\n"
            "            o descarga desde https://ffmpeg.org/download.html\n"
            "  macOS   : brew install ffmpeg\n"
            "  Linux   : sudo apt install ffmpeg\n"
        )
        sys.exit(1)


# ── Extracción de frames ──────────────────────────────────────────────────────

def _get_video_duration(video_path: str) -> Optional[float]:
    """Obtiene la duración en segundos con ffprobe."""
    cmd = [
        'ffprobe', '-v', 'error',
        '-select_streams', 'v:0',
        '-show_entries', 'format=duration',
        '-of', 'default=noprint_wrappers=1:nokey=1',
        video_path,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        val = r.stdout.strip()
        return float(val) if val and val != 'N/A' else None
    except Exception:
        return None


def extract_frame_ffmpeg(video_path: str, position: str, out_path: str) -> bool:
    """
    Extrae un frame del vídeo con ffmpeg.
    position: 'first' | 'last'
    Devuelve True si el archivo de salida existe y tiene contenido.
    """
    if position == 'first':
        cmd = [
            'ffmpeg', '-y',
            '-i', video_path,
            '-vf', r'select=eq(n\,0)',
            '-vframes', '1',
            '-q:v', '2',
            out_path,
        ]
    else:  # last
        # Seek desde el final; más rápido que decodificar todo el vídeo
        duration = _get_video_duration(video_path)
        if duration and duration > 0.2:
            seek_time = duration - 0.15
            cmd = [
                'ffmpeg', '-y',
                '-ss', f'{seek_time:.4f}',
                '-i', video_path,
                '-vframes', '1',
                '-q:v', '2',
                out_path,
            ]
        else:
            # Fallback: sseof (menos preciso pero funciona sin duración)
            cmd = [
                'ffmpeg', '-y',
                '-sseof', '-0.5',
                '-i', video_path,
                '-vframes', '1',
                '-q:v', '2',
                out_path,
            ]

    try:
        result = subprocess.run(cmd, capture_output=True, timeout=90)
        ok = os.path.exists(out_path) and os.path.getsize(out_path) > 100
        if not ok:
            print(f"    ffmpeg stderr: {result.stderr.decode(errors='replace')[-400:]}")
        return ok
    except subprocess.TimeoutExpired:
        print(f"    TIMEOUT extrayendo frame '{position}' de {video_path}")
        return False
    except Exception as e:
        print(f"    ERROR ffmpeg: {e}")
        return False


def load_frame(video_path: str, position: str, tmp_dir: str) -> Optional[np.ndarray]:
    """
    Intenta cargar el frame con cv2 primero; si falla usa ffmpeg.
    Devuelve imagen RGB uint8 o None.
    """
    # ── Intento cv2 (puede fallar con HEVC en algunos sistemas) ──────────────
    if HAS_CV2:
        cap = cv2.VideoCapture(video_path)
        if cap.isOpened():
            if position == 'last':
                total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                if total > 1:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, total - 2))
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # ── Fallback ffmpeg ───────────────────────────────────────────────────────
    out_path = os.path.join(tmp_dir, f'frame_{position}_{os.getpid()}.jpg')
    if not extract_frame_ffmpeg(video_path, position, out_path):
        return None

    if HAS_CV2:
        img = cv2.imread(out_path)
        if img is not None:
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Último recurso: matplotlib imread
    try:
        img = plt.imread(out_path)
        if img.dtype != np.uint8:
            img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
        return img
    except Exception as e:
        print(f"    No se pudo leer la imagen extraída: {e}")
        return None


# ── Interfaz de anotación ─────────────────────────────────────────────────────

class FrameAnnotator:
    """
    Muestra frame inicial y final lado a lado.
    Recoge 4 clics en este orden obligatorio:
      1. Extremo A  → Frame INICIAL
      2. Extremo B  → Frame INICIAL
      3. Extremo A  → Frame FINAL
      4. Extremo B  → Frame FINAL
    """

    # (frame_label, punto_label, color_marcador, índice_eje)
    STEPS = [
        ('INICIAL', 'Extremo A del rail', _C['A'], 0),
        ('INICIAL', 'Extremo B del rail', _C['B'], 0),
        ('FINAL',   'Extremo A del rail', _C['A'], 1),
        ('FINAL',   'Extremo B del rail', _C['B'], 1),
    ]

    def __init__(self, img_first: np.ndarray, img_last: np.ndarray, video_name: str):
        self.img_first  = img_first
        self.img_last   = img_last
        self.video_name = video_name
        self.points: list[tuple[float, float]] = []

    def run(self) -> list[tuple[float, float]]:
        """
        Bloquea hasta obtener 4 clics válidos o que el usuario cierre la ventana.
        Devuelve lista de 4 puntos: [A_ini, B_ini, A_fin, B_fin].
        Lanza RuntimeError si se cierra antes de completar.
        """
        fig, axes = plt.subplots(1, 2, figsize=(17, 8))
        fig.patch.set_facecolor(_C['bg'])

        self._fig  = fig
        self._axes = axes

        for ax in axes:
            ax.set_facecolor(_C['bg'])
            for spine in ax.spines.values():
                spine.set_color('#45475a')
            ax.tick_params(colors='#6c7086')

        axes[0].imshow(self.img_first)
        axes[1].imshow(self.img_last)
        axes[0].set_title('FRAME INICIAL', color=_C['initial'], fontsize=14,
                           fontweight='bold', pad=10)
        axes[1].set_title('FRAME FINAL',   color=_C['final'],   fontsize=14,
                           fontweight='bold', pad=10)

        fig.suptitle(
            f'Vídeo: {self.video_name}',
            color=_C['text'], fontsize=12, y=0.99,
        )

        self._instr = fig.text(
            0.5, 0.01,
            self._instruction_text(),
            ha='center', fontsize=11, color=_C['ok'],
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#313244', alpha=0.9),
        )

        plt.tight_layout(rect=[0, 0.07, 1, 0.97])
        self._cid = fig.canvas.mpl_connect('button_press_event', self._on_click)
        plt.show()

        if len(self.points) < 4:
            raise RuntimeError(
                f"Ventana cerrada antes de completar los 4 clics para '{self.video_name}'."
            )
        return self.points

    # ── helpers internos ──────────────────────────────────────────────────────

    def _instruction_text(self) -> str:
        n = len(self.points)
        if n >= 4:
            return '✓ Completado — cierra la ventana para continuar al siguiente vídeo.'
        frame_lbl, pt_lbl, _, ax_idx = self.STEPS[n]
        side = 'izquierda' if ax_idx == 0 else 'derecha'
        return (
            f'Clic {n+1}/4  →  Frame {frame_lbl} (panel {side})  |  {pt_lbl}\n'
            f'  Verde = Extremo A   |   Amarillo = Extremo B'
        )

    def _on_click(self, event) -> None:
        if len(self.points) >= 4:
            return
        if event.inaxes is None:
            return

        n = len(self.points)
        frame_lbl, pt_lbl, color, ax_idx = self.STEPS[n]
        expected_ax = self._axes[ax_idx]

        if event.inaxes != expected_ax:
            side = 'izquierda' if ax_idx == 0 else 'derecha'
            self._instr.set_text(
                f'⚠  Haz clic en el frame {frame_lbl} (panel {side}).\n'
                + self._instruction_text().split('\n')[0]
            )
            self._instr.set_color(_C['warn'])
            self._fig.canvas.draw_idle()
            return

        x, y = event.xdata, event.ydata
        self.points.append((x, y))

        # Marcador de clic
        expected_ax.plot(x, y, '+', markersize=22, markeredgewidth=3,
                         color=color, zorder=10)
        expected_ax.annotate(
            f'{pt_lbl}\n({x:.0f}, {y:.0f})',
            xy=(x, y), xytext=(x + 25, y - 25),
            color=color, fontsize=8, zorder=11,
            arrowprops=dict(arrowstyle='->', color=color, lw=1.2),
        )

        # Vector cuando el par está completo
        if n == 1:    # par inicial completo
            x0, y0 = self.points[0]
            self._axes[0].annotate(
                '', xy=(x, y), xytext=(x0, y0),
                arrowprops=dict(arrowstyle='->', color=_C['initial'], lw=2.5),
                zorder=12,
            )
        elif n == 3:  # par final completo
            x0, y0 = self.points[2]
            self._axes[1].annotate(
                '', xy=(x, y), xytext=(x0, y0),
                arrowprops=dict(arrowstyle='->', color=_C['final'], lw=2.5),
                zorder=12,
            )

        self._instr.set_text(self._instruction_text())
        self._instr.set_color(_C['ok'] if len(self.points) < 4 else '#cba6f7')
        self._fig.canvas.draw_idle()


# ── Cálculo de ángulos ────────────────────────────────────────────────────────

def angle_of_vector(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    """
    Ángulo del vector p1→p2 en grados, rango [-180, 180].
    Coordenadas de imagen: y positivo hacia abajo.
    Se invierte y para obtener convención matemática estándar (CCW positivo).
    """
    dx =   p2[0] - p1[0]
    dy = -(p2[1] - p1[1])   # flip Y → convención estándar
    return math.degrees(math.atan2(dy, dx))


def minimal_delta(a_from: float, a_to: float) -> float:
    """Diferencia de ángulo mínima en [-180, 180]."""
    d = a_to - a_from
    while d >  180: d -= 360
    while d < -180: d += 360
    return d


# ── Búsqueda de vídeos ────────────────────────────────────────────────────────

def find_videos(folder: str) -> list[str]:
    return sorted(
        os.path.join(folder, f)
        for f in os.listdir(folder)
        if Path(f).suffix.lower() in VIDEO_EXTENSIONS
    )


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    # Carpeta de vídeos
    if len(sys.argv) > 1:
        folder = sys.argv[1]
    else:
        folder = input("Ruta a la carpeta de vídeos (Enter = carpeta actual): ").strip()
        if not folder:
            folder = '.'

    folder = os.path.abspath(folder)
    if not os.path.isdir(folder):
        print(f"ERROR: '{folder}' no es un directorio válido.")
        sys.exit(1)

    check_ffmpeg()

    videos = find_videos(folder)
    if not videos:
        print(f"No se encontraron vídeos {VIDEO_EXTENSIONS} en '{folder}'.")
        sys.exit(0)

    print(f"\nEncontrados {len(videos)} vídeo(s):")
    for v in videos:
        print(f"  {os.path.basename(v)}")

    results: list[dict] = []
    tmp_dir = tempfile.mkdtemp(prefix='robot_rot_')

    try:
        for i, video_path in enumerate(videos, 1):
            name = os.path.basename(video_path)
            print(f"\n{'─'*62}")
            print(f"[{i}/{len(videos)}] {name}")

            print("  Extrayendo primer frame...")
            img_first = load_frame(video_path, 'first', tmp_dir)
            print("  Extrayendo último frame...")
            img_last  = load_frame(video_path, 'last',  tmp_dir)

            if img_first is None:
                print("  ERROR: no se pudo extraer el primer frame. Saltando.")
                continue
            if img_last is None:
                print("  ERROR: no se pudo extraer el último frame. Saltando.")
                continue

            print("  Abriendo ventana de anotación...")
            try:
                annotator = FrameAnnotator(img_first, img_last, name)
                pts = annotator.run()
            except RuntimeError as e:
                print(f"  {e}  Saltando.")
                continue

            A_ini, B_ini = pts[0], pts[1]
            A_fin, B_fin = pts[2], pts[3]

            ang_ini = angle_of_vector(A_ini, B_ini)
            ang_fin = angle_of_vector(A_fin, B_fin)
            d_ang   = minimal_delta(ang_ini, ang_fin)

            direction = 'CW (horario)' if d_ang < 0 else 'CCW (antihorario)'
            print(f"  Ángulo inicial : {ang_ini:+.2f}°")
            print(f"  Ángulo final   : {ang_fin:+.2f}°")
            print(f"  Rotación Δ     : {d_ang:+.2f}°  ({direction})")

            results.append({
                'filename':      name,
                'angle_initial': round(ang_ini, 4),
                'angle_final':   round(ang_fin, 4),
                'delta_angle':   round(d_ang,   4),
            })

    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    # ── Guardar CSV ───────────────────────────────────────────────────────────
    if not results:
        print("\nNo se procesó ningún vídeo correctamente.")
        return

    csv_path = os.path.join(folder, CSV_FILENAME)
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(
            f, fieldnames=['filename', 'angle_initial', 'angle_final', 'delta_angle']
        )
        writer.writeheader()
        writer.writerows(results)

    print(f"\n{'═'*62}")
    print(f"CSV guardado en: {csv_path}")
    print(f"{'═'*62}")
    print(f"\n{'Archivo':<36} {'Ini°':>8} {'Fin°':>8} {'Δ°':>10}  Sentido")
    print('─' * 70)
    for r in results:
        d = r['delta_angle']
        sens = 'CW' if d < 0 else 'CCW'
        print(f"{r['filename']:<36} {r['angle_initial']:>8.2f} {r['angle_final']:>8.2f} {d:>10.2f}  {sens}")


if __name__ == '__main__':
    main()
