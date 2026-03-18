# Pipeline de Calibración de Odometría — Tracer Agilex 2.0

## Estructura de carpetas

```
calibracion_odom/
├── media/          ← vídeos .mov del iPhone
├── scripts/
│   ├── calibrar_camara.py
│   ├── extract_aruco.py
│   ├── merge_calibration.py
│   └── calibracion_camara.json
└── data/          ← CSVs de resultados
```

---

## 1. Transferir el vídeo del iPhone a Windows

1. Conecta el iPhone por USB
2. Abre el Explorador de archivos → `Este equipo → Apple iPhone → Internal Storage → DCIM`
3. Copia el `.mov` a `calibracion_odom/media/`

---

## 2. Transferir el CSV del Jetson a Windows

Abre PowerShell y ejecuta:

```powershell
scp jetson@172.16.242.200:/home/jetson/catkin_ws/src/narcis_ws/resultados_calibracion.csv "C:\Users\narci\Documents\GETI\SocialTech Challenge\calibracion_odom\data\resultados_YYYYMMDD.csv"
```

Sustituye `YYYYMMDD` por la fecha, ej: `resultados_20260317.csv`.

> ⚠️ Renombra siempre el CSV con la fecha — el script de ROS siempre genera el mismo nombre `resultados_calibracion.csv` y se sobreescribiría.

---

## 3. Extraer ángulo real con ArUco

```bash
cd "C:\Users\narci\Documents\GETI\SocialTech Challenge\calibracion_odom"

python3 scripts\extract_aruco.py \
    --video "media\NOMBRE_VIDEO.mov" \
    --calib scripts\calibracion_camara.json \
    --output data\aruco_iters.csv
```

### Output esperado
- `data/aruco_iters.csv` — runs reales (Δθ por iteración)
- `data/aruco_iters_sync.csv` — sync moves (para sincronización temporal)

### Verificar que la segmentación es correcta
- Número de runs detectados debe coincidir con el CSV del Jetson
- `Δθ_real` debe estar cerca de `angulo_objetivo` (±5° máximo para ω altas)
- Los 2 sync moves deben aparecer en `aruco_iters_sync.csv`

### Si no detecta iteraciones
Ajusta estos parámetros en el script:
```python
PAUSA_UMBRAL_DEG   = 5.0   # subir si el robot vibra en pausa
PAUSA_MIN_SEG      = 2.0   # bajar si las pausas son cortas
```

---

## 4. Merge ArUco + Odom

```bash
python3 scripts\merge_calibration.py \
    --aruco data\aruco_iters.csv \
    --sync  data\aruco_iters_sync.csv \
    --odom  data\resultados_20260317.csv \
    --out   data\merged_20260317.csv
```

### Verificar sincronización
- `drift` entre sync inicial y final debe ser `< 0.5s` — si es mayor, revisar
- `Matches` debe ser `N/N` — si hay sin match, revisar tolerancia temporal en el script

### Output
```
n, vel_max, acel, angulo_objetivo, direccion, angulo_odom, angulo_real
1, 0.6, 0.5, 360, CCW, 359.952, 360.253
...
```

---

## 5. Abrir en Excel

1. Abre Excel
2. **Datos → Desde texto/CSV**
3. Selecciona `data/merged_20260317.csv`
4. Separador: coma → Cargar

---

## 6. Acumular datasets de múltiples sesiones

Cada sesión genera un `merged_FECHA.csv`. Para combinarlos todos:

```bash
# En PowerShell — combina todos los merged en un dataset final
$header = Get-Content data\merged_20260317.csv | Select-Object -First 1
$header | Out-File data\dataset_completo.csv -Encoding utf8

Get-ChildItem data\merged_*.csv | ForEach-Object {
    Get-Content $_ | Select-Object -Skip 1
} | Add-Content data\dataset_completo.csv
```

> El `dataset_completo.csv` es el que usas para el análisis final y el paper.

---

## Parámetros fijos de referencia

| Parámetro | Valor |
|---|---|
| Marker ID | 0 |
| Diccionario ArUco | DICT_4X4_50 |
| Tamaño marker | 9.58 cm (0.0958 m) |
| Resolución vídeo | 1080x1920 @ 30fps |
| Calibración cámara | `calibracion_camara.json` (RMS=0.05px) |
| Pausa entre runs | 5s |
| Pausa sync move | 2s antes del primer run |

---

## Checklist antes de cada sesión de grabación

- [ ] iPhone cargado y con espacio suficiente (>500 MB por sesión)
- [ ] Trípode fijo, encuadre centrado en el robot
- [ ] ArUco visible y bien iluminado
- [ ] Jetson encendido y ROS corriendo
- [ ] Script de ROS lanzado **antes** de pulsar grabar en el iPhone
- [ ] Grabar → esperar sync move inicial → dejar correr → sync move final → parar grabación