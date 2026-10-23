# ProyectoRobot

Robot virtual orientado a observar movimientos humanos, imitarlos y aprender
tareas progresivamente. El desarrollo prioriza CPU en Fedora 43, i7-10510U,
16 GB RAM, Intel UHD y NVIDIA GeForce MX130, con GNOME Wayland. La visión y
la física usan CPU; el dibujo del robot aprovecha OpenGL acelerado.

**Fase actual: 2.2 — manos articuladas e interacción física con objetos.**
La Fase 1 sigue disponible con el mismo comando. El usuario confirmó su
funcionamiento con la webcam y autorizó Fase 2. Memoria, entrenamiento y agentes
permanecen sin implementar; ver [PLAN.md](PLAN.md).

## Inicio rápido de Fase 2.2

Se reutilizan el venv, MuJoCo y ambos modelos existentes, sin instalaciones ni
descargas. Desde la raíz, comprobar primero sin webcam:

```bash
.venv/bin/python -m proyectorobot.simulation.hands_app --check
.venv/bin/python -m proyectorobot.simulation.hands_app --hand-demo --headless
.venv/bin/python -m proyectorobot.simulation.hands_app --grasp-demo --headless
.venv/bin/python -m proyectorobot.simulation.hands_app --grasp-demo --headless --side right
```

Ver la demo física (19 s), después iniciar imitación con ambas ventanas:

```bash
.venv/bin/python -m proyectorobot.simulation.hands_app --grasp-demo --graphics intel
.venv/bin/python -m proyectorobot.simulation.hands_app --graphics intel
```

Primero calibra los brazos abajo como en Fase 2.1. Para **cada mano**, muestra
la palma abierta con pulgar separado y quieta al menos medio segundo (cinco
observaciones); después cierra el puño y mantenlo otro medio segundo. El panel
indica `CAL: ABRIR MANO`, `CAL: CERRAR PUNO`, recuperación y gesto. Puedes calibrar
cada mano sucesivamente. Mantén muñecas y palmas visibles y evita cruzarlas.
Los ángulos de dedos y separación pulgar/índice controlan apertura continua,
con calibración, filtros y límites.

Para colocar la pinza sobre el cubo sin depender de profundidad monocular:

```bash
.venv/bin/python -m proyectorobot.simulation.hands_app --keyboard --graphics intel
.venv/bin/python -m proyectorobot.simulation.hands_app --assist --graphics intel
```

| Tecla en el visor del robot | Acción |
| --- | --- |
| `1` / `2` | Seleccionar mano izquierda/derecha anatómica |
| `O` / `F` / `P` | Abrir / puño / pinza, progresivamente |
| `B` / `L` / `T` / `H` | Acercar al cubo / elevar / trasladar / postura segura asistida |
| `M` | Alternar brazos asistidos y seguimiento corporal (con webcam) |
| `K` | Retirar órdenes manuales de dedos y seguir gestos |
| `R` / `C` | Parar y abrir / recalibrar y reanudar |
| `Q` / Escape | Salir |

En `--keyboard`: `O`, `B` (esperar 3 s), `P` (esperar 3 s), `L`, `T`, `O`;
espera cada movimiento. Usa `--side right` para situar la mesa a la derecha.
`--assist` mantiene brazos por teclado y dedos por webcam, tras calibrar ambos.
`--no-hands` evita inferencia de dedos para ahorrar CPU: brazos por cámara y
dedos con `O/F/P`.

El agarre físico verificado eleva unos 13,2 cm, traslada 14 cm y suelta sobre la
mesa con ambas manos. Los cubos son cuerpos libres. Ante pérdida de seguimiento,
los dedos se abren suavemente y los brazos retornan a seguro; el cubo puede caer.
La webcam no garantiza posición espacial precisa: usa asistencia para el agarre.
Detalles y límites: [docs/PHASE2_2.md](docs/PHASE2_2.md). Se conservan las
[verificaciones manuales de Fase 1](docs/VALIDATION.md).

Ante líneas mezcladas o cambios de postura: el modo de robot ahora filtra
saltos, longitudes incoherentes y algunas inversiones ambiguas de profundidad.
Los puntos rechazados no se dibujan ni controlan el brazo; se muestran las
causas y recuperación. Izquierda anatómica verde, derecha naranja. Un cruce
gradual de brazos sigue permitido. La Fase 1 conserva su dibujo anterior.

El modo `I:3D D:3D` del robot confirma control con profundidad estimada; `2D`
no distingue delante/detrás. `3D-PAUSA` retiene la última consigna durante una
interrupción breve (máximo 200 ms); `3D-RECUPERANDO` espera muestras coherentes.
Mantén brazos abajo y visibles al pulsar `C`. La webcam proporciona estimaciones,
sin garantizar réplica exacta ni localización precisa de la pinza sobre el cubo.
La calibración 3D ahora se completa automáticamente por brazo si faltó al iniciar:
mantén brazos abajo y estirados, con hombros/codos/muñecas visibles, unos 2–3 s.
Necesita al menos seis muestras coherentes y un segundo de postura estable.
No calibres extendiendo el brazo hacia delante: esa postura se usa después de
que aparezca `3D I:ACTIVA` o `D:ACTIVA`. Si se queda en 2D, la nueva línea 3D
indica el motivo: falta de world, confianza, geometría, postura o recuperación.
En `--assist` los brazos se mueven por teclado; pulsa `M` en el visor para volver
a imitar los brazos con la cámara, o inicia el comando normal sin `--assist`.
Para agarrarlo, usa `--assist`, `B` y el indicador **Pinza-cubo**: cuando la pinza
esté cerca del centro, ciérrala con tu gesto o `P`; espera contactos de pulgar e
índice antes de `L`. El indicador mide solo posiciones virtuales de MuJoCo.

## Inicio rápido de Fase 2.1

MuJoCo 3.17.0 ya estaba instalado en `.venv`; no hace falta reinstalar paquetes.
Primero comprobar el humanoide sin webcam:

```bash
.venv/bin/python -m proyectorobot.simulation --check
.venv/bin/python -m proyectorobot.simulation --demo --headless --duration 24
.venv/bin/python -m proyectorobot.simulation --demo
```

Después abrir cámara y robot simultáneamente:

```bash
.venv/bin/python -m proyectorobot.simulation --camera 0
```

El modo gráfico predeterminado `auto` usa la selección del sistema; en este
equipo se verificó Intel UHD. El visor indica el renderizador real. Para elegir
una tarjeta explícitamente o usar el respaldo por CPU:

```bash
.venv/bin/python -m proyectorobot.simulation --graphics intel
.venv/bin/python -m proyectorobot.simulation --graphics nvidia
.venv/bin/python -m proyectorobot.simulation --graphics software
```

Intel y NVIDIA pasaron dibujo, cámara y redimensionado. La MX130 usa ahora dibujo
directo a la ventana para evitar el fallo de la copia offscreen; esa copia sigue
protegida. Comparadas con la misma escena y resolución, Intel tardó unos 5.2 ms
por ciclo y NVIDIA 12.2 ms, ambas limitadas a 30 FPS. Intel sigue siendo la opción
recomendada para esta escena. No se instalaron paquetes ni se cambiaron
controladores. Evidencia en [docs/GRAPHICS.md](docs/GRAPHICS.md).

La ventana de gestos de Fase 2 abre compacta, a 640×480, con panel inferior.
El perfil rápido procesa como máximo 480×360 y solicita 30 FPS. Sigue hombros,
codos y muñecas; para ahorrar CPU, el modelo adicional de dedos se activa solo
con `--hands`. El seguimiento completo de manos de Fase 1 se conserva.

Ponte de frente con hombros, codos y muñecas visibles; no necesitas mostrar
las caderas en Fase 2. Mantén los brazos abajo y estirados durante al menos un
segundo (6 muestras estables; normalmente 1–2 segundos)
para calibrar; luego levántalos o dóblalos. `R` vuelve suavemente a reposo y pausa
la imitación hasta pulsar `C`, que recalibra y reanuda. `Q`/Escape cierran.
En el robot, izquierdo verde y derecho naranja; son lados anatómicos, no columnas
de pantalla. La cámara puede estar en espejo sin alterar esos lados.

Cada hombro tiene abducción, flexión y rotación; cada codo tiene flexión de 0° a
150°. La demo dura 24 segundos y prueba los ejes sucesivamente. Para observar el
giro axial prepara un codo flexionado. Las flechas giran la vista del robot.

La imitación calcula ángulos de los segmentos detectados. Usa los world landmarks
3D de MediaPipe si superan calibración y controles de calidad; `I:3D D:3D` indica
el modo de cada lado anatómico. `2D-F` conserva la flexión frontal visible, como
formar una «L» a los lados, orientando la bisagra en ese plano. No reconstruye
profundidad. En `2D`, sin evidencia frontal suficiente, flexión y rotación del
hombro vuelven a neutral; elevación y codo se estiman por proyección. Un brazo recto no permite
medir giro axial, que también vuelve a neutral. La profundidad monocular sigue
siendo aproximada; no se añaden muñecas articuladas ni dedos.

El texto del visor es pequeño y se dibuja al pie, sin ampliar sus letras con la
imagen del robot. La pérdida de seguimiento devuelve suavemente a reposo.
Detalles y aceptación manual pendiente en [docs/PHASE2_1.md](docs/PHASE2_1.md).

Las rotaciones entran gradualmente al flexionar el codo; la histéresis evita que
pequeñas oscilaciones cambien repetidamente entre flexión lateral y profundidad.
El filtro responde antes a movimientos claros y conserva las velocidades máximas.
El visor solicita 30 FPS y abre a 800×600. Intel/software dibujan internamente
a 384×288; la MX130 dibuja directamente al tamaño de la ventana. Los
FPS reales de `Vision` y `Robot` se muestran por separado; pedir 30 no garantiza
30 inferencias por segundo en CPU. Para limitar consumo se puede usar:

```bash
.venv/bin/python -m proyectorobot.simulation --fps 20 --render-fps 20
```

El visor repinta de forma opaca al mover/redimensionar la ventana para evitar
estelas y transparencia. Si quieres ejecutar su diagnóstico gráfico sin webcam:
`.venv/bin/python -m scripts.check_robot_repaint`. Para cargar una corrección del
programa, cierra las ventanas anteriores y vuelve a ejecutar el comando de Fase 2.

## Estado inicial y compatibilidad

Ya existe `.venv` con Python 3.12.15, MediaPipe 1.1.1, NumPy 2.5.4 y OpenCV
5.0.0.93. Los imports y la API Tasks funcionan. El Python del sistema es 3.14.3:
usa siempre `.venv/bin/python` o activa expresamente el venv.

La inspección inicial encontró dos distribuciones OpenCV que compartían `cv2`.
Con autorización, se retiró `opencv-python` y se reinstaló la misma versión de
`opencv-contrib-python`, también requerida por MediaPipe, sin modificar otras
dependencias. El diagnóstico detecta futuras colisiones aunque `pip check` tenga
éxito. También se descargaron con autorización los dos modelos oficiales de
Fase 1; las sumas locales y los bundles quedan fuera de Git.

La inspección ve video0/video1 en sysfs, pero no tiene `/dev/video*` accesible.
La captura y la ventana deben probarse en una terminal de la sesión GNOME del
usuario con acceso a su webcam. Detalles en [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md).

## Preparación por etapas

Los siguientes comandos se ejecutan desde la raíz del repositorio. No es
necesario instalar el proyecto con `pip install -e .` para usarlo.

1. Inspeccionar el entorno existente, sin cámara, ventana ni acceso a red:

   ```bash
   .venv/bin/python --version
   .venv/bin/python -m pip --disable-pip-version-check check
   .venv/bin/python -m proyectorobot --check
   ```

   `--check` devuelve 1 si faltan dependencias/modelos o existen distribuciones
   OpenCV en conflicto. La ausencia de dispositivos se informa por separado.

2. Solo si hace falta otro entorno y se autoriza su instalación, crear uno
   separado conservando `.venv` y añadir **solo dependencias de Fase 1**:

   ```bash
   python3.12 -m venv .venv-phase1
   .venv-phase1/bin/python -m pip install --only-binary=:all: -r requirements-phase1.txt
   .venv-phase1/bin/python -m pip check
   ```

   Estos comandos son instrucciones, no se ejecutan automáticamente. No se usa
   `sudo`, no se cambia Python del sistema y no se instalan MuJoCo/Gymnasium/SB3.
   Si se usa este entorno, sustituir `.venv` por `.venv-phase1` en los comandos.
   Evitar instalar `opencv-python` o variantes `headless` junto a contrib.

3. Revisar las URL de los modelos; este comando no descarga nada:

   ```bash
   .venv/bin/python scripts/download_models.py
   ```

   Tras autorizar la descarga, obtener únicamente Pose Lite y Hand Landmarker:

   ```bash
   .venv/bin/python scripts/download_models.py --download
   ```

   Son aproximadamente 14 MiB en total; tamaño orientativo, límite de 40 MiB por
   archivo. También se pueden copiar manualmente los bundles oficiales a
   `models/pose_landmarker_lite.task` y `models/hand_landmarker.task`.
   La aplicación no accede a la red para descargar modelos al arrancar.
   Procedencia y sumas locales en [models/README.md](models/README.md).

4. Comprobar inferencia con los modelos, sin usar webcam ni ventanas:

   ```bash
   .venv/bin/python -m proyectorobot --smoke-test
   ```

## Ejecutar Fase 1

En una terminal de GNOME, con la webcam disponible:

```bash
.venv/bin/python -m proyectorobot
```

Opciones habituales:

```bash
.venv/bin/python -m proyectorobot --camera 0 --width 640 --height 480 --fps 20
.venv/bin/python -m proyectorobot --no-mirror
.venv/bin/python -m proyectorobot --width 480 --height 360 --fps 15
.venv/bin/python -m proyectorobot --window-width 1600 --window-height 900
.venv/bin/python -m proyectorobot --models-dir /ruta/a/modelos
.venv/bin/python -m proyectorobot --help
```

La ventana se abre a 1280×720 y se puede maximizar o redimensionar. Las opciones
`--window-width` y `--window-height` cambian únicamente su tamaño inicial: la
captura y la inferencia permanecen a 640×480 salvo que se indique otra resolución.
La pantalla muestra conexiones de pose en verde y manos en amarillo. Un panel
lateral separado muestra estados independientes, número de manos, FPS del ciclo
e inferencia en milisegundos con letras más grandes y gruesas, sin tapar articulaciones.
El mensaje `Movimiento` distingue `MANO IZQUIERDA ARRIBA`, `MANO DERECHA ARRIBA`,
`AMBAS MANOS ARRIBA` y `MANOS ABAJO`. Si falta seguimiento en un lado, muestra
`SEGUIMIENTO PARCIAL` y conserva el estado individual del lado observable; si
faltan ambos, muestra `SIN SEGUIMIENTO`.
Los FPS solicitados a la cámara pueden ser ignorados por el driver; los FPS
mostrados reflejan el procesamiento observado. El primer valor no es un benchmark.
`Q`, Escape o cerrar la ventana terminan la aplicación. Ctrl+C también libera
modelos y cámara, con código de salida 130. Errores de ejecución devuelven 1;
opciones CLI inválidas devuelven 2.

## Qué significa «brazo arriba»

Se considera elevado cuando la **muñeca está por encima del hombro** al menos
un 20 % de la altura vertical entre hombro y cadera del mismo lado. Para bajarlo,
la diferencia debe caer al 10 % o menos. La banda intermedia conserva el estado
y cancela cambios pendientes. Un cambio requiere al menos 150 ms continuos.
Al iniciar o recuperar seguimiento, el estado permanece desconocido hasta
completar ese tiempo.

El detector valida hombro, muñeca y cadera: presencia y visibilidad >= 0.6,
coordenadas finitas y dentro de la imagen, y torso vertical de al menos el 5 %
de la altura de imagen. Un fallo reinicia inmediatamente ese lado a
`SIN SEGUIMIENTO`; no interpreta una oclusión como brazo bajado.

Izquierda y derecha son **las de la persona**, independientemente del espejo.
La inferencia usa el frame original; únicamente se reflejan presentación y
articulaciones. Las manos se detectan por separado; no se asignan a brazos en
esta fase. Un brazo doblado con muñeca alta cuenta como elevado: aún no se estima
extensión ni se calibra una postura 3D. Usar una vista frontal erguida, con hombros,
muñecas y caderas visibles, buena luz y una sola persona. Perspectiva, inclinación
del cuerpo y oclusiones limitan esta regla 2D.

## API, CPU y arquitectura

Se usa la API actual [MediaPipe Tasks de pose](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker/python)
y [manos](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python),
`BaseOptions.Delegate.CPU`, pose Lite sin máscaras, una persona y hasta dos manos.
Se procesan frames de webcam secuencialmente en `RunningMode.VIDEO` con
`detect_for_video`: permite tracking y resultados de ambos modelos alineados al
mismo frame y timestamp, con menor complejidad inicial que callbacks asíncronos.
El tiempo usa un reloj monotónico estrictamente creciente. OpenCV se limita a
dos hilos; MediaPipe administra sus propios hilos CPU. No se guardan colas de
resultados ni se importa software de aprendizaje/simulación.

```text
proyectorobot/
  app.py             ciclo de aplicación y CLI
  config.py          rutas y nombres de modelos
  diagnostics.py     inspección local
  vision/            captura, Tasks y overlay
  control/           estados, calibración frontal y control seguro
  data/              contratos independientes de MediaPipe
  simulation/        humanoide MJCF, MuJoCo, visor y CLI de Fase 2
  learning/          reservado
  agents/            reservado
models/              bundles locales, fuera de Git
scripts/             preparación explícita
tests/               pruebas automáticas
docs/                entorno y validación
```

La aplicación no graba ni transmite fotogramas. MediaPipe documenta por separado
su política de métricas de uso en su [aviso de privacidad](https://pypi.org/project/mediapipe/1.1.1/).

## Pruebas y aceptación

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Se prueban estados, ruido, oclusiones, timestamps, errores de webcam simulada,
liberación parcial de modelos, conversión RGB, espejo, los mensajes de movimientos
combinados y descargas simuladas.
No se abre una webcam ni una ventana ni se accede a la red. Las pruebas de visión
se omiten si faltan dependencias; la de inferencia real se omite si faltan modelos.
`unittest` viene con Python: no requiere instalar pytest.

La aceptación manual se describe en [docs/VALIDATION.md](docs/VALIDATION.md).
Una prueba sintética correcta no acredita reconocimiento humano ni rendimiento.
El usuario confirmó el funcionamiento básico de Fase 1 y autorizó Fase 2. Las
comprobaciones manuales restantes siguen señaladas; no se infieren de las pruebas
unitarias. La Fase 3 necesita una nueva autorización.

## Problemas frecuentes

- **No abre la cámara:** revisar `ls -l /dev/video*`, elegir otro `--camera`, cerrar
  aplicaciones que la usen y comprobar acceso desde la sesión local o contenedor.
  sysfs puede mostrar una webcam que el sandbox no expone. No ejecutar como root
  ni cambiar permisos a `777`.
- **No abre la ventana en Wayland:** la rueda local usa Qt5. Si hay `DISPLAY`, la
  aplicación selecciona `xcb` en su propio proceso salvo configuración explícita.
  Se necesita XWayland operativo. Si Qt falla al cargar plugins, probar desde la
  sesión GNOME con `QT_QPA_PLATFORM=xcb .venv/bin/python -m proyectorobot` y revisar
  la instalación gráfica antes de autorizar cambios. Un fallo nativo de Qt puede
  abortar antes de que Python lo capture.
- **Faltan modelos o son inválidos:** revisar nombres/rutas; la descarga es separada.
  Un archivo existente se conserva; no se borra automáticamente. Verificar su
  procedencia antes de reemplazarlo.
- **Importaciones fallan:** comprobar el intérprete y `--check`; no añadir una
  segunda distribución OpenCV. La guía [oficial de OpenCV](https://pypi.org/project/opencv-python/)
  indica usar una sola variante.
- **Estado desconocido:** encuadrar también las caderas y dejar 150 ms estables.
- **Pocos FPS:** buena iluminación, una persona, resolución 480×360 y medir. No
  instalar GPU, modelos grandes ni más librerías para compensar sin diagnóstico.
