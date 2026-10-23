# Fase 2.1: movilidad avanzada de brazos

Autorizada por el usuario. Se conserva la Fase 1 y la captura aislada de Fase 2.
No se instalaron paquetes ni se descargaron modelos. No se graba ni transmite
video; la inferencia y la simulación son locales, en CPU.

## Ejecutar y calibrar

Desde la raíz, usando el venv existente:

```bash
.venv/bin/python -m proyectorobot.simulation --check
.venv/bin/python -m proyectorobot.simulation --demo --headless --duration 24
.venv/bin/python -m proyectorobot.simulation --demo
.venv/bin/python -m proyectorobot.simulation --camera 0
```

La demo es independiente de la webcam: prueba ocho ejes en 24 segundos. Para
mostrar la rotación del hombro prepara el codo a 60°; durante el giro mantiene
ese codo fijo. Estos movimientos de prueba solo se utilizan en `--demo`.

Para imitación, permanecer de frente con hombros, codos y muñecas visibles,
brazos abajo y codos extendidos. No es necesario mostrar caderas ni piernas.
Mantenerse estable durante al menos un segundo y seis muestras (normalmente
1–2 segundos). El panel muestra el progreso. Mantener la cámara quieta.

El modo de cada lado aparece como `I:3D D:3D`, `2D-F`, `2D` o `--`. Los lados son
anatómicos: verde izquierdo y naranja derecho. `2D-F` conserva la flexión frontal
visible; en `2D`, los ejes de profundidad están en neutral. Si no había suficientes
world landmarks válidos durante la calibración, ese brazo permanece en respaldo
2D/2D-F hasta una nueva calibración con `C`.

Controles, con el foco en cualquiera de las dos ventanas:

- `R`: retorno suave a reposo y pausa de la imitación; permanece así hasta `C`.
- `C`: descarta la calibración y consignas anteriores, calibra y reanuda.
- `Q` o Escape: cerrar. También se puede cerrar una ventana o usar Ctrl+C.
- En el visor: flechas para cambiar vista y rueda para acercar/alejar.

Para observar los codos hacia delante conviene girar ligeramente la vista del
robot con las flechas; la persona debe seguir de frente a la webcam.
`--no-mirror` cambia la presentación de la cámara, sin intercambiar articulaciones.

## Modelo y coordenadas

Los cambios principales están en `simulation/assets/humanoid.xml`,
`control/motion.py`, `control/retargeting.py` y el nuevo `control/spatial.py`.
`data/types.py` y `vision/detector.py` transportan los puntos world;
`vision/worker.py`, `simulation/app.py` y `simulation/viewer.py` integran estados,
pausa y texto. `simulation/demo.py` prueba los ejes sin webcam. El nuevo
`tests/test_spatial.py` se complementa con las regresiones existentes ampliadas.

El torso está anclado. Los segmentos miden 0.30 m y 0.27 m; las manos son
geometrías simplificadas conectadas al antebrazo. Se conservan geometrías
primitivas, compensación de gravedad y ausencia de contactos/autocolisiones.
No se simula equilibrio, locomoción, pronosupinación independiente ni muñecas.

El marco del robot tiene X hacia su izquierda, Y hacia atrás y Z hacia arriba.
Por lado, las bisagras se componen en el siguiente orden local; `s=+1` para
izquierdo y `s=-1` para derecho. Ángulos internos en radianes, rangos MJCF en grados.

| Eje por brazo | Nombre | Eje local | Límites | Neutral |
| --- | --- | --- | --- | --- |
| Abducción/aducción | `{side}_shoulder` | `(0,-s,0)` | -30° a 170° | 10° |
| Flexión/extensión | `{side}_shoulder_flexion` | `(-1,0,0)` | -50° a 160° | 0° |
| Rotación axial | `{side}_shoulder_rotation` | `(0,0,-s)` | -100° a 100° | 0° |
| Flexión/extensión de codo | `{side}_elbow` | `(-1,0,0)` | 0° a 150° | 0° |

Son límites aproximados de un modelo simplificado, no validación biomecánica.
Cada eje dispone de un servo de posición, control y fuerza limitados.

`ArmAngles` conserva los argumentos anteriores `shoulder, elbow` y añade
`flexion, rotation`. Su propiedad `values` ofrece el orden físico de la tabla.
`JOINTS` y el MJCF tienen primero los cuatro ejes izquierdos y luego los derechos.
El motor comprueba cantidad, orden, límites, rangos de servos y postura neutral.

## Conversión y calidad

`Detector` conserva `pose_world_landmarks` de MediaPipe Tasks en
`Observation.pose_world`. No se añade otro modelo ni se utiliza el Z normalizado
de imagen como profundidad métrica. Los puntos world admiten coordenadas fuera
de [0,1]; se verifica confianza y finitud con `Landmark.confident`.

`control/spatial.py` forma los vectores hombro→codo y codo→muñeca. La línea de
hombros y la vertical óptica definen un marco frontal ortogonal; no exige caderas
ocultas ni deduce la orientación del tronco desde ellas. La calibración establece
offsets de hombro/codo y proporciones de segmentos por lado.

Se valida presencia/visibilidad >=0.65 en imagen y world, coordenadas finitas,
separación de hombros plausible, vista frontal, proporciones de segmentos y
concordancia entre las direcciones proyectadas de imagen y world. Tras calibrar,
las proporciones deben permanecer entre 70% y 130% de la referencia; cambios
demasiado rápidos de orientación invalidan esa muestra. Son heurísticas: MediaPipe
no proporciona una garantía independiente de exactitud de profundidad.

La cinemática inversa obtiene abducción y flexión del vector del brazo, la flexión
de codo del ángulo entre los dos segmentos y la rotación del plano de flexión.
Se elige una rama angular continua dentro de los límites para evitar invertir
el hombro al pasar por encima. Si el brazo está casi alineado con el eje frontal,
abducción y giro axial van a neutral porque esa descomposición es ambigua.
Con el codo casi estirado tampoco es observable el giro axial, que va a neutral.

El modo 3D necesita al menos seis muestras de calibración válidas, estables y
proporciones consistentes. Después de perder calidad exige tres muestras válidas
y un mínimo de 80 ms en la aplicación (150 ms por defecto en el dominio).
A 12 FPS, reunir tres muestras requiere aproximadamente 167 ms aunque el mínimo
sea 80 ms. Sin 3D fiable se conserva la proyección 2D de elevación y flexión de
codo. Si ambos segmentos conservan entre 80% y 130% de su longitud proyectada
de calibración y la flexión supera 5°, el modo
`2D-F` representa el gesto en el plano frontal. El signo de flexión orienta la
bisagra mediante una rotación de hombro que aumenta gradualmente de 0° a -90°
o +90° entre 5° y 25° de flexión de codo, manteniendo flexión de
hombro neutral. Así conserva una «L» arriba o abajo en vez de doblarla hacia
delante. Es una representación frontal observada, no una medición de giro axial
real ni reconstrucción de profundidad. Fuera de esas condiciones se mantienen
flexión y rotación de hombro neutrales. Si tampoco hay proyección válida, se
neutraliza todo ese brazo.

Antes de aceptar 3D, una flexión frontal clara también comprueba las longitudes
proyectadas: se descarta world si su proyección relativa es incompatible con la
imagen, usando una tolerancia de `1.6 * proyección_world + 0.1` respecto a la
longitud de referencia. Evita que una falsa profundidad casi paralela a la cámara
convierta una «L» visible en un codo casi recto. Las poses de profundidad coherentes
con la imagen siguen usando el modo 3D; cada brazo puede usar un modo diferente.

Todos los objetivos pasan por suavizado adaptativo: 80 ms para errores pequeños
y hasta 32 ms para movimientos claros, conservando el límite de 1.5 rad/s.
Se conservan las guardas físicas de 2 rad/s, límites articulares,
timeout monotónico de 350 ms y colas acotadas. El contador de guardas descarta
diferencias numéricas menores de 1e-9 rad; se siguen recortando esas diferencias.
La simulación responde aunque la inferencia o la comunicación se bloqueen.

## Visualización y rendimiento

Se conservan ambas ventanas, procesamiento máximo de 480×360, pose sin modelo
adicional de dedos y captura negociada a 640×480. `--hands` sigue siendo opcional.
La física usa pasos de 5 ms; el dibujo del robot usa 384×288 y solicita 30 FPS,
con Mesa por software y dos hilos. La ventana del robot abre a 800×600 y la de
cámara conserva 640×480. El texto usa fuente 100 y se dibuja al pie en la
ventana final; no crece al ampliar la imagen del robot. Se muestran seguimiento,
calibración, FPS y edad de la observación. La edad no mide la latencia interna
de exposición/buffering del dispositivo ni la latencia completa de movimiento.
El panel distingue FPS de visión, FPS reales dibujados y Hz del ciclo principal;
estos últimos no son los pasos físicos de 200 Hz. `--fps` limita la visión y
`--render-fps` limita el dibujo. La cadencia del visor conserva plazos y descarta
deuda de dibujo tras bloqueos, sin intentar recuperar fotogramas atrasados.

## Evidencias y aceptación pendiente

Se validaron primero los ocho ejes sin webcam. La demo headless de 24 segundos
recorrió unos 68° por hombro de abducción/flexión, 58° de rotación y 67.7° de codo,
con retorno a `10°,0°,0°,0°` por lado. La demo gráfica completó también los ocho
ejes en GNOME y cerró con código 0. La preparación posterior de codo flexionado
facilita observar el giro axial.

La prueba real de integración abrió ambas ventanas durante 20 segundos:
208 observaciones recibidas, 207 consumidas, aproximadamente 12–14 FPS tras
inicialización, última inferencia 54 ms, edad 74 ms y ciclo de simulación 55.3 FPS.
Terminó con `CALIBRAR: AMBOS BRAZOS` y cero frames con control activo. Esto
comprueba captura, ejecución y cierre; no acredita imitación humana 3D calibrada.
Los avisos de fuentes Qt y ROI/feedback de MediaPipe persistieron sin impedir
la ejecución. No se cambiaron paquetes para ocultarlos.

Las pruebas automáticas no abren webcam ni GUI:

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Incluyen cinemática inversa contrastada con posiciones reales del MJCF, ambos
lados, movimientos diferentes, calibración, cambio de rama por encima del hombro,
giro no observable, pérdida de profundidad y oclusión, recuperación, referencias
3D ausentes, escala/traslación, límites, velocidad, watchdog, pausa y regresiones
de las dos fases previas. MCP y los servidores de la skill no se utilizan.

Resultado final: **102 pruebas superadas sin omisiones** en 10.876 s. Se verificó
la sintaxis compatible con Python 3.12, el XML y los enlaces locales. El diagnóstico
gráfico `python -m scripts.check_robot_repaint`, ejecutado con el venv, pasó
22 presentaciones en tres tamaños de ventana; conserva opacidad y repintado.

Corrección posterior de codos laterales: el usuario confirmó profundidad hacia
delante y reportó que la «L» lateral se perdía. Dos regresiones nuevas reprodujeron
el fallo antes de corregirlo: respaldo 2D con plano incorrecto y world con falsa
profundidad que borraba un codo de 90°. La suite final pasa **108 pruebas sin
omisiones** en 14.179 s. Se verifica con física real que las muñecas suban/bajen
al menos 24 cm respecto al codo y permanezcan en el plano frontal, ambos lados,
profundidad en un brazo y «L» en el otro, rechazo de proyección acortada y retorno
seguro por timeout. No se cambió el MJCF, el visor ni las dependencias en esta
corrección. La aceptación con webcam del gesto corregido sigue pendiente.

Para cargar la corrección, cerrar las ventanas anteriores con `Q`, volver a
ejecutar el comando de imitación y calibrar de nuevo con los brazos abajo.

## Rotaciones y respuesta más rápida

Se conserva la «L» lateral y la profundidad. Cerca del codo extendido, el giro
axial entra gradualmente entre 5° y 25° en vez de saltar al cruzar un umbral.
Cuando el brazo apunta casi a la cámara, abducción y giro se atenúan de forma
continua según su observabilidad. La elección de rama también considera el
rango de giro. Si el giro excede ±100°, mantiene el último valor seguro: evita
alternar entre límites opuestos por ruido angular alrededor de ±180°.

El modo frontal usa histéresis: entra con longitudes entre 80% y 130% y se mantiene
entre 72% y 140%, con salida por debajo de 3° de flexión. Eso evita alternancias
de plano por oscilaciones pequeñas en los landmarks. Se conservan comprobaciones
de confianza, pérdida de seguimiento, límites de velocidad y retorno seguro.
La clasificación visual usa 80 ms de permanencia en Fase 2; Fase 1 conserva 150 ms.
La calibración inicial sigue exigiendo un segundo y seis muestras estables.

La comparación probó entrada de 384×288 y render con un solo hilo. La inferencia
siguió alrededor de 60–70 ms y el hilo único empeoró el dibujo, por lo que se
conservan entrada 480×360 y dos hilos. Se redujo únicamente el dibujo del robot,
sin bajar la resolución de los landmarks. El perfil solicita hasta 30 FPS, con
dependencias, modelos y delegado CPU existentes. No se añadieron procesos.

La lectura serie medida consumía 8–13 ms por ciclo. `CameraStream`, usado solo
en Fase 2, añade un lector V4L2 dentro del proceso de visión. Ese hilo abre, lee
y libera la cámara; mantiene una única imagen pendiente y sustituye las antiguas.
Así la espera del dispositivo se solapa con inferencia. Fase 1 sigue usando la
captura síncrona anterior. Se conserva el timestamp monotónico de entrega del
fotograma; la inferencia se cronometra por separado y la edad incluye la espera
local antes de procesarlo. No mide el instante de exposición del sensor.

No se liberan recursos V4L2 concurrentemente con una lectura bloqueada. Cerrar el
lector espera como máximo un segundo; si el driver continúa bloqueado, el proceso
aislado conserva el cierre/terminación acotados de `VisionStream`. El watchdog del
robot sigue independiente y devuelve la postura segura si falta una pose fresca.

### Mediciones del ajuste

Pruebas locales breves, con ambas ventanas y sin guardar imágenes. Se excluye
el primer informe de arranque de los rangos de visión. Las personas/poses y la
carga de escritorio no son idénticas entre pruebas; no equivalen a un benchmark
controlado de precisión o latencia completa.

| Medición | Visión observada | Dibujo del robot | Última edad de pose |
| --- | --- | --- | --- |
| Perfil anterior, 15 s | 12.1–12.6 FPS | límite anterior de 20 FPS; sin contador real | 119 ms |
| Perfil final con lector acotado, 18 s | 11.5–12.3 FPS | 29.5 FPS al final | 132 ms |

La ejecución final recibió 164 observaciones, consumió 163 y aplicó control en
138, completó calibración y terminó en `SIGUIENDO | I:2D D:2D`, sin guardas físicas.
El último tiempo de inferencia fue 71 ms; las medias de intervalos estables
fueron 64–69 ms. El aumento demostrado es de fluidez de dibujo. No se acredita
una mejora de FPS de inferencia ni de latencia de webcam frente a la línea base;
la limitación sigue en CPU. Los plazos de clasificación y el filtro sí son más
cortos y se verifican con secuencias sintéticas. Confirmar manualmente calidad
de rotación, profundidad y «L» tras reiniciar; esta ejecución no certifica su
precisión visual ni seguimiento 3D sostenido.

Verificación final: **121 pruebas superadas sin omisiones** en 16.024 s. Las
nuevas pruebas cubren continuidad de giro, singularidad frontal, histéresis,
ángulos no representables, recuperación de tres muestras/80 ms, reducción del
error de respuesta con velocidad limitada, supresión de ruido pequeño y cadencia
de 30 FPS bajo control de 50 Hz. `tests/test_camera_stream.py` verifica reemplazo
de imágenes antiguas, timestamp, propiedad del recurso en un único hilo, error,
cierre y driver bloqueado sin liberar recursos concurrentemente. La demo física
completa usa el filtro adaptativo y regresa a neutral. El diagnóstico gráfico
pasó 26 presentaciones en tres tamaños, conservando repintado y opacidad.

La API Tasks en modo VIDEO ya utiliza seguimiento para evitar repetir la
detección en todos los fotogramas; se conserva esa API, sin parches privados.
Referencia: [guía oficial de Pose Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker/python).

Pendiente de aceptación con una persona en GNOME:

1. Completar calibración y confirmar `3D` por lado con buena iluminación.
2. Elevar, abrir, adelantar brazos y flexionar codos, primero por separado y después
   con movimientos distintos. Observar también el robot desde una vista oblicua.
3. Acercar manos al pecho/cara manteniendo visibles codos y muñecas: una oclusión
   puede activar el retorno seguro. El robot no evita penetrar su propio torso.
4. Repetir con/sin espejo; comprobar `R`, pausa, `C`, recuperación y todos los cierres.
5. Medir dos minutos de FPS, edad, CPU y RAM; valorar estabilidad del modo 3D y
   posibles cambios a 2D. Los FPS solicitados no se garantizan con este hardware.
6. Conservar los pendientes de Fase 1 en [VALIDATION.md](VALIDATION.md).

No iniciar Fase 3 sin autorización. Fuentes primarias:
[MediaPipe Pose Landmarker Python](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker/python)
y [MJCF, ejes y orden de articulaciones](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint).
