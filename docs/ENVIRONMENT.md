# Inspección del entorno

Inspección inicial realizada el 22 de octubre de 2026, sin instalaciones ni cambios
al venv. Las acciones posteriores autorizadas se registran al final.

| Componente | Observación local | Consecuencia |
| --- | --- | --- |
| Sistema | Fedora 43 Workstation, x86_64, glibc 2.42 | Usar ruedas Linux x86_64 y comprobar imports |
| CPU | Intel i7-10510U, 4 núcleos/8 hilos, AVX2 | Inferencia CPU y modelo pose Lite |
| RAM | 15 GiB reportados, aproximadamente 11 GiB disponibles al inspeccionar | Un frame y sus resultados en memoria; sin grabación |
| Sesión | Wayland, `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0` | OpenCV Qt5 se ejecutará vía XWayland cuando esté disponible |
| Python del sistema | 3.14.3 | No usar `python3` a ciegas |
| Python disponible | `/usr/bin/python3.12`, versión 3.12.15 | Coincide con el objetivo |
| venv existente | `.venv`, Python 3.12.15, pip 26.2.2, aislado del sistema | Reutilizado para comprobaciones; no recreado |
| MediaPipe | 1.1.1 | Imports de `mediapipe.tasks` correctos |
| OpenCV inicial | `opencv-python` y `opencv-contrib-python`, ambos 5.0.0.93; `cv2` reporta 5.0.0 | Colisión de namespace; corregida posteriormente con autorización |
| NumPy | 2.5.4 | Import correcto junto a OpenCV/MediaPipe |
| Backend OpenCV | QT5; V4L2 disponible | Validación GUI pendiente; no instalar headless |
| SQLite | 3.50.2 a través de `sqlite3` | Reservado para la fase de datos |
| MuJoCo | 3.17.0 ya instalado | No importado ni utilizado en Fase 1 |
| Webcam | `/sys/class/video4linux` enumera video0/video1, pero `/dev/video*` no está expuesto aquí | No implica ausencia física; no se puede verificar captura en este entorno |
| Modelos | No había archivos `.task` | Necesitan descarga explícita o provisión manual |

`pip check` no reportó requisitos rotos. Esta herramienta no detecta que dos
distribuciones distintas sobrescriben `cv2`. La guía oficial de OpenCV indica usar
una sola. MediaPipe instalado declara `opencv-contrib-python`; se eligió esa
distribución para `requirements-phase1.txt`.

Se comprobaron localmente `BaseOptions.Delegate.CPU`, las opciones de ambos
landmarkers, las conexiones públicas de articulaciones y la creación de un
`mp.Image` RGB de NumPy. Esto verifica importación/API, todavía no una inferencia
ni el funcionamiento de la ventana o la cámara.

Las versiones directas de Fase 1 están fijadas a las encontradas. No constituyen
un lock completo de dependencias transitivas ni garantizan una instalación nueva;
esta debe verificarse con el resolvedor y `pip check` después de autorizarla.

Fuentes primarias consultadas:

- [Preparación de Python para MediaPipe](https://developers.google.com/edge/mediapipe/solutions/setup_python).
- [API de Pose Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker/python).
- [API de Hand Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python).
- [MediaPipe en PyPI](https://pypi.org/project/mediapipe/1.1.1/).
- [Paquetes OpenCV y namespace compartido](https://pypi.org/project/opencv-python/).

La compatibilidad de Gymnasium y Stable-Baselines3 se revisará en su fase;
su presencia o la existencia de una rueda no demuestra rendimiento adecuado.

## Acciones posteriores autorizadas

- Se descargaron por HTTPS los bundles oficiales versión 1: pose Lite (5 777 746
  bytes) y manos (7 819 105 bytes). Las sumas SHA256 locales coinciden con los
  archivos obtenidos; no se presentan como firmas del proveedor.
- Se descargó primero la rueda binaria de `opencv-contrib-python==5.0.0.93` a un
  directorio temporal. Después se desinstaló únicamente `opencv-python` y se
  reinstaló contrib desde la rueda local con `--no-deps`. No se cambiaron las
  demás dependencias ni Python, ni se instalaron bibliotecas de otras fases.
- Los dos modelos se inicializaron y procesaron tres frames negros en CPU con
  `detect_for_video`, sin pose/manos espurias. MediaPipe confirmó el delegado
  TensorFlow Lite XNNPACK para CPU.
- La captura real y la ventana no se ejecutaron: `/dev/video*` sigue sin estar
  expuesto en este entorno. Pendiente de validación en GNOME.

Durante las comprobaciones, Matplotlib (dependencia transitiva de MediaPipe)
usó una caché temporal porque el sandbox no permite escribir en su directorio
habitual. MediaPipe emitió avisos nativos sobre feedback y proyección de ROI;
no impidieron terminar la inferencia sintética. No se ocultaron esos avisos.

## Comprobaciones de Fase 2 — 23 de octubre de 2026

MuJoCo 3.17.0 importa correctamente con Python 3.12.15. Se cargó un MJCF y se
ejecutaron pasos físicos sin contexto gráfico. Después se validó el humanoide
propio: cuatro bisagras y servos, neutral estable, movimientos y límites.
No se instalaron paquetes ni se modificaron las versiones existentes.

GLFW 2.11.0 usa una biblioteca 3.4 con Wayland, EGL y OSMesa declarados; la
consulta `platform_supported` reportó Wayland sí y X11 no. Mesa/EGL permitió
renderizar el humanoide sin `/dev/dri`: `GL_RENDERER` reportó
`llvmpipe (LLVM 21.1.8, 256 bits)` y `GL_VENDOR` reportó `Mesa`.
Posteriormente se abrió la demo interactiva autorizada durante cinco segundos:
terminó con código 0 tras mover las cuatro articulaciones y regresar a reposo.
Después se separó Qt/visión en un proceso y se implementó render MuJoCo/GLFW
acotado a 640×480 y dos hilos llvmpipe. La prueba autorizada de 15 segundos abrió
ambas ventanas y recibió 58 observaciones (57 consumidas). Visión: ~5–6.5 FPS;
última inferencia 114 ms, retardo 258 ms; simulación: 38.3 FPS al final. No se
completó la calibración humana. Esa verificación y las mediciones prolongadas
siguen pendientes. No se guardaron imágenes de webcam ni se cambiaron paquetes.
