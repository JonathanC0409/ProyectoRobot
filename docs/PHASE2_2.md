# Fase 2.2: manos y manipulación física

Autorizada por el usuario. Se reutilizan Python 3.12, venv, MuJoCo y los modelos
locales. Sin instalaciones, descargas, grabación ni servicios externos. Fase 3
continúa sin autorización. Los comandos anteriores conservan el modelo original.

## Archivos y arquitectura

| Archivo bajo `proyectorobot/` | Función |
| --- | --- |
| `simulation/assets/hand.xml` | Plantilla MJCF: palma y cinco dedos con segmentos visibles |
| `simulation/hand_model.py` | Reflejo anatómico, brazos originales, mesa y cubos libres |
| `simulation/manipulation.py` | Dinámica de contactos sin corregir posiciones de objetos |
| `simulation/grasp_demo.py` | Secuencia articular y medición del agarre/liberación |
| `simulation/hands_app.py` | Entrada independiente: demos, webcam, teclado y asistencia |
| `control/hands.py` | Consignas normalizadas, límites, velocidad y watchdog |
| `control/gestures.py` | Geometría, asociación anatómica, calibración y filtros |
| `vision/worker.py` | Gestos opcionales en el proceso y cola ya existentes |
| `simulation/viewer.py` | Cola acotada de teclas; conserva render y gráficos anteriores |
| `control/motion.py` | Postura segura configurable, con valor original por defecto |

`tests/test_hands.py` y `tests/test_manipulation.py` verifican geometría,
clasificación, control, física y errores sin cámara/GUI. Los nuevos campos de
`VisionConfig` y `VisionPacket` tienen valores predeterminados compatibles.
El control no importa OpenCV ni MediaPipe. No se añaden procesos ni hilos.

## Modelo y física

Cinco bisagras por mano, límites 0–75° y consignas 0–70°. Pulgar e índice
opuestos; medio/anular/meñique comparten una sinergia mediante dos igualdades
articulares por mano. Los segmentos distales son rígidos respecto al proximal.
Se conservan los ocho ejes y límites de brazos; no se añade muñeca.

Escena: 49 geometrías, 21 articulaciones (8 brazos, 10 dedos y 3 freejoints),
14 servos. Mesa lateral y tres cubos de 4 cm/40 g. Integrador implicitfast,
paso de 2 ms, cono elíptico, fricción y almohadillas. Se activan contactos de
manos con objetos/mesa y de cubos entre sí/suelo. No se añaden autocolisiones
ni contactos del resto del humanoide. Torso anclado. Los contactos suaves
permiten penetración numérica pequeña; no se promete agarre universal ni
separación geométrica perfecta. Referencia: [contactos de MuJoCo](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters).

La posición inicial del cubo se calcula por cinemática **antes de simular**.
Después no se escribe su qpos/qvel, ni se usan welds, mocaps, imanes o agarres
por proximidad. El motor separado evita aplicar las guardas de ocho qpos del
motor anterior a cuerpos libres. Se limitan consignas (brazos 1,5 rad/s, dedos
2 rad/s) y fuerzas; la velocidad física depende de contactos y dinámica, sin
reescribir estados físicos. La postura segura del lado de la mesa queda sobre
ella, con mano abierta.

## Ejecución por etapas

Desde la raíz:

```bash
.venv/bin/python -m proyectorobot.simulation.hands_app --check
.venv/bin/python -m proyectorobot.simulation.hands_app --hand-demo --headless
.venv/bin/python -m proyectorobot.simulation.hands_app --grasp-demo --headless
.venv/bin/python -m proyectorobot.simulation.hands_app --grasp-demo --headless --side right
.venv/bin/python -m proyectorobot.simulation.hands_app --grasp-demo --graphics intel
.venv/bin/python -m proyectorobot.simulation.hands_app --graphics intel
```

Demo de controles: 18 s, seis controles independientes. Demo de agarre: 19 s,
abrir (0–1), acercar (1–4), cerrar (4–7), elevar (7–10), trasladar (10–14),
soltar y esperar reposo (14–19). Es una trayectoria de prueba articular. La
imitación en vivo utiliza landmarks y consignas continuas, sin reproducirla.

La demo devuelve error si no logra elevación >9 cm, traslado >9 cm, contactos
opuestos durante >100 muestras, liberación sin contacto, altura final a <5 mm
de la inicial y norma de velocidades libres final <0,02. La gráfica también
evalúa criterios si completa los 19 s; salir antes no acredita el agarre.

## Calibración y gestos

Primero calibra brazos abajo, con hombros/codos/muñecas visibles, como antes.
Después, por mano: palma abierta y pulgar separado, quieta >=500 ms/cinco
observaciones; luego puño cerrado con igual estabilidad. Se pueden calibrar
sucesivamente. Los brazos siguen funcionando durante calibración de manos;
las manos sin calibrar quedan abiertas. `C` borra ambas calibraciones y brazos.

Las distancias se normalizan con anchura/longitud de palma, corrigiendo aspecto.
Ángulos de falanges y extensión de cadena estiman flexión. Se calibran postura
abierta, cerrada y separación inicial de pinza. El Z local estimado por Hands
solo ayuda a medir ángulos; no es posición mundial medida ni mueve objetos.
Se conserva [MediaPipe Tasks Hand Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python),
VIDEO/CPU y los modelos ya disponibles; no se instala Gesture Recognizer.

Asociación por proximidad a muñecas **anatómicas** de pose, independiente de
orden de resultados y espejo visual. Se rechazan muñecas demasiado próximas,
ausencia de hombros/muñecas fiables, manos pequeñas o geometría degenerada.
Hands no proporciona confianza individual de los puntos en este contrato;
las verificaciones geométricas no garantizan profundidad precisa.

Estados: ABIERTA/PUNO/PINZA/PARCIAL, calibración, recuperación o SIN MANO.
El puño tiene prioridad frente a pequeña distancia pulgar/índice. Filtro de
consignas de 65 ms, permanencia de estados de 80 ms y límites de velocidad.
Recuperación con dos muestras y >=80 ms. La pérdida conserva calibración y
descarta filtros anteriores. Sin datos frescos por 350 ms se abren manos y se
retorna al brazo seguro, mientras la simulación continúa fuera de visión.
Una mano ausente se abre independientemente; conserva brazos observables.
Un objeto sostenido puede caer. `R` pausa/abre; `C` recalibra/reanuda.

## Teclado y asistencia

```bash
.venv/bin/python -m proyectorobot.simulation.hands_app --keyboard --graphics intel
.venv/bin/python -m proyectorobot.simulation.hands_app --assist --graphics intel
.venv/bin/python -m proyectorobot.simulation.hands_app --no-hands --graphics intel
```

Teclas del visor: `1/2` lado; `O/F/P` abrir/puño/pinza; `B/L/T/H` acercar/elevar/
trasladar/postura segura; `M` brazos por cámara/asistidos; `K` retira órdenes de
dedos del teclado; `Q`/Escape sale. En `--keyboard`, M no abre ni activa cámara.
En `--assist`, brazos por teclado y dedos por cámara; primero se calibra visión.
Si desaparece el cuerpo o se atrasa visión, también se suspende asistencia.

Prueba asistida izquierda: `1`, `O`, `B`, esperar 3 s, `P`, esperar 3 s, `L`,
esperar, `T`, esperar, `O`. La mesa queda en el lado elegido con `--side right`
o `left`; seleccionar la otra mano no recoloca objetos. Las poses asistidas
son específicas de la escena; no hay planificación autónoma ni posición exacta
desde una cámara. En modo libre, acertar al cubo no está garantizado.

## Evidencia, rendimiento y pendientes

Pruebas sin webcam: controles independientes, límites y sinergia; agarre físico
izquierdo/derecho, elevación 0,13221 m, traslado 0,14042 m, 2.999 muestras con
contactos opuestos y liberación con error de altura 0,00000852 m. Control negativo
sin contactos de manos: sin elevación ni traslado. Gráfica Intel: ~29,9–30,4 FPS,
mismas distancias y 704 muestras de contacto; liberación y cierre correctos.

Inferencia real con ambos modelos: smoke-test de tres imágenes negras correcto.
Webcam/visor reales, 18 s: 95 observaciones y cero manos calibradas consumidas;
no se completó calibración humana de brazos. Visión estable ~6,4–8,3 FPS, inferencia
media de intervalos ~106–140 ms, dibujo ~30 FPS. No se afirma precisión de gestos
humanos. Qt avisó sobre fuentes y el proceso de visión necesitó terminación
acotada al cerrar. La prueba posterior reabrió correctamente la cámara durante
10 s con `--no-hands`: 72 observaciones, ~13–13,7 FPS de visión, inferencia media
estable de 58–60 ms y ~29–30 FPS de dibujo; cierre normal sin terminación forzada.
No hubo calibración humana. Es una medición breve, no una garantía sostenida.

Visión <=480×360, cámara solicitada/real 640×480, ventana robot 800×600. Textos
compactos y panel fuera de imagen. Se muestran FPS de dibujo/visión, inferencia
y edad. `--no-hands` evita cargar/ejecutar Hands: conserva brazos y dedos por
teclado. Ambos modelos cuestan CPU y reducen FPS respecto a pose sola; no se
prometen 30 FPS de inferencia. Dibujo Intel acelerado, física/visión en CPU.

Ejecutar suite:

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Validación inicial de Fase 2.2: 156 pruebas correctas, sin omisiones; incluye transporte real
de gestos por IPC, rechazo de datos antiguos/futuros, pausa y recalibración desde
cualquiera de las ventanas. `pip check` sin incompatibilidades. Ambas demos
headless y smoke-test de los modelos existentes terminaron con código 0.

Pendiente humano: calibrar cada mano y comprobar abierta/puño/pinza progresiva,
manos independientes y brazos simultáneos; tamaño/legibilidad, espejo y cruces;
oclusiones, timeout/recuperación, R/C, cierres y reapertura; agarre asistido.
Medir CPU/RAM/FPS durante dos minutos. Mantener los pendientes de
[Fase 1](VALIDATION.md) y [Fase 2.1](PHASE2_1.md). Sin avance a Fase 3.

## Corrección de cruces y profundidad inestable

Tras el reporte del usuario, `control/pose_quality.py` valida observaciones
antes de retargeting, gestos y dibujo de Fase 2. Los mismos puntos aprobados
alimentan el control y las líneas. No se modifica la inferencia/dibujo de Fase 1,
el modelo físico ni sus ocho ejes de brazos.

Se comprueban confianza >=0,7, continuidad respecto al centro/anchura de hombros,
longitudes proyectadas y cambios de signo de profundidad sin movimiento visible
compatible. La referencia de huesos se aprende con seis muestras de brazos
relajados hacia abajo, sin usar brazos levantados. Se suavizan pequeños cambios
con constante de 25 ms. Solo se enmascaran codo/muñeca del lado dudoso, también
en world; las manos no se asignan a una muñeca invalidada. No se corrigen etiquetas
intercambiándolas automáticamente. Los cruces graduales se permiten.

Si un antebrazo apunta hacia la webcam, su proyección puede ser muy corta; se
permite cuando la estimación world supera los filtros. Sin world coherente, no
se inventa profundidad de ese segmento. Los rechazos requieren tres muestras
coherentes antes de recuperar el brazo y muestran SALTO, OCULTO, GEOMETRIA,
HUESO INCOHERENTE, PROFUNDIDAD AMBIGUA o SIN PROFUNDIDAD.

La conversión usada por el proceso de visión tiene `depth_grace_ms=200`: mantiene
brevemente una consigna 3D si la imagen del brazo sigue observable; el estado es
3D-PAUSA. Si world vuelve pero faltan sus muestras de recuperación, el estado es
3D-RECUPERANDO y el brazo retorna a seguro, evitando un paso artificial por 2D.
La oclusión no usa esa retención. El watchdog por edad de captura sigue en 350 ms;
la retención no genera imágenes ni extrapola movimientos. La API directa conserva
por defecto retención cero, para compatibilidad de consumidores anteriores.

En cámara compacta, brazo izquierdo verde y derecho naranja, iguales al robot.
Los segmentos rechazados desaparecen del dibujo. Se muestra el diagnóstico por
lado; `3D ESTIMADA` describe world filtrado, mientras I:3D/D:3D en el estado del
robot confirma que se usa tras calibración. 2D/2D-F no resuelven delante/detrás.

El visor añade distancia entre el site de pinza y el centro de cube_0, y los
dedos con fuerzas de contacto. Son medidas del mundo virtual. Para agarrar:
`--assist`, calibrar brazos/manos, `1` (o `2` con --side right), abrir, `B`, esperar
que la distancia baje, pinza por gesto o `P`, confirmar contactos pulgar/índice,
`L`, `T`, abrir. No se corrigen posiciones ni se pega el cubo. En modo libre,
los ángulos humanos no garantizan coincidencia espacial con el objeto.

Limitaciones: son heurísticas conservadoras que pueden pausar movimientos
rápidos válidos; no eliminan todos los errores de MediaPipe. Una proyección
monocular puede admitir distintas poses 3D. La mano simplificada tampoco tiene
muñeca ni todas las articulaciones humanas. No se promete réplica al 100% ni
resolver oclusiones. La GPU dibuja el robot; cambiar GPU no elimina esas
ambigüedades. No se instalan modelos ni dependencias. Pendiente comparar estos
casos con una persona, especialmente cruces, movimientos hacia cámara y agarre.

Validación del ajuste: 170 pruebas correctas, sin omisiones (14 nuevas respecto
a la versión inicial), incluido el agarre físico de ambos lados. Se verifican
intercambio brusco de codos/muñecas, saltos, cruces graduales, normalización de
escala, recuperación independiente, aprendizaje de referencia con brazos abajo,
inversión de profundidad con igual imagen, profundidad válida al avanzar y
retención/recuperación 3D. El dibujo tiene colores anatómicos comprobados.

Prueba real de integración de 12 s con Intel y ambos modelos: 46 observaciones,
visión ~5,8–6,4 FPS y dibujo ~29,9 FPS. No completó calibración humana ni produjo
manos calibradas. Cerró con código 0; se conservan los avisos conocidos de Qt.
No acredita que los errores reportados desaparezcan con una persona. Sintaxis,
enlaces locales y `git diff --check` correctos; sin nuevas instalaciones.

## Corrección de sesión bloqueada en 2D

El usuario confirmó que aún veía solo 2D al intentar alcanzar hacia delante.
Se encontró una condición reproducible: `SpatialMapper.calibrate` se llamaba
solo al terminar la calibración inicial de imagen. Si faltaba world o no reunía
sus muestras válidas, la referencia 3D quedaba ausente el resto de la sesión.
Los world válidos posteriores no podían activarla. Los tests anteriores de
profundidad usaban world válido durante el arranque y no cubrían ese caso.

Ahora la referencia pendiente se completa automáticamente **por brazo**, con
imagen y world coherentes en postura relajada hacia abajo, >=6 muestras y >=1 s.
La referencia de un brazo listo no cambia mientras se calibra el otro. La ventana
de 90 muestras permite cumplir el segundo incluso a 30 FPS. No se usa una
postura de alcance como neutral. C aún permite recalibrar todo explícitamente.

La API instalada de MediaPipe representa visibilidad/presencia de world como
campos opcionales. El adaptador anterior convertía un campo ausente en cero.
`convert_world_landmarks` completa solo ausencias con el score del **mismo punto**
de imagen. Un score world explícitamente bajo, cero o no finito sigue fallando;
sin score de imagen tampoco se inventa confianza. Se conservan límites de
geometría, proyección y confianza. Esta es una corrección de compatibilidad;
no se afirma que todas las sesiones del usuario tuvieran campos ausentes.

`inspect_world` devuelve causas de rechazo además de brazos filtrados, manteniendo
`measure_world` compatible. `RetargetResult.depth_status` y `VisionPacket.depth_status`
añaden progreso y causa a cámara, visor y logs sin imágenes. Mensajes: SIN WORLD,
SIN POSE, CONFIANZA WORLD HOMBROS, CONFIANZA BRAZO, ESCALA WORLD HOMBROS,
TORSO LATERAL, LONGITUD WORLD, PROPORCION WORLD, PROYECCION WORLD,
PROYECCION NO COINCIDE, BAJAR BRAZO PARA 3D, CALIBRAR 3D, LONGITUD INESTABLE,
SALTO 3D, RECUPERANDO y ACTIVA. No se etiqueta 3D una consigna puramente 2D.

Para probar imitación, cerrar la versión anterior y abrir sin --assist:

```bash
.venv/bin/python -m proyectorobot.simulation.hands_app --graphics intel
```

Mantener brazos abajo/estirados y articulaciones visibles 2–3 s. Esperar estado
3D ACTIVA antes de mover el brazo hacia delante. `--no-hands` permite aislar pose
y ahorrar inferencia, pero desactiva gestos de dedos. En --assist, los brazos
responden al teclado: pulsar M para pasar a cámara. Esta elección ahora también
se explica al iniciar. No garantiza precisión espacial suficiente para agarrar
sin asistencia: calibración angular y localización del objeto son cosas distintas.

Pruebas nuevas en `tests/test_depth_recovery.py` y `tests/test_vision.py`:
arranque 2D y activación tardía sin reinicio, alcance frontal tras calibrar,
independencia de brazos, referencia anterior intacta, rechazo de falsa postura
neutral, cadena adaptador/filtro/control con scores world ausentes, diagnóstico
de rechazos, conservación de scores bajos y ventana de calibración a 30 FPS.
La comprobación humana de estos casos sigue siendo necesaria.

Validación de esta corrección: 179 pruebas correctas, sin omisiones. En una
ejecución real de 18 s con --no-hands e Intel se recibieron 180 observaciones,
visión estable ~12,6–13,6 FPS y dibujo ~29,9 FPS. El estado permaneció CALIBRAR:
AMBOS BRAZOS, principalmente con CONFIANZA BRAZO y una muestra de PROPORCION WORLD;
no se completó calibración humana ni hubo control 3D aceptado. La cámara, nuevas
líneas de diagnóstico y el cierre funcionaron (código 0). No se declara resuelta
la aceptación humana del alcance. Los mensajes se calculan después del filtro
de calidad; revisar también OCULTO/SALTO/etc. en esa línea para distinguir puntos
rechazados del detector de puntos enmascarados por continuidad. No se bajaron
umbrales sin evidencia. Sintaxis, enlaces y git diff --check correctos.
