# Plan de ProyectoRobot

El objetivo es un robot virtual que observe, imite, almacene demostraciones,
aprenda tareas y, posteriormente, se entrene por refuerzo. Todas las fases se
diseñan para CPU y simulación ligera en el equipo disponible.

Cada fase requiere presentar evidencias, limitaciones y resultados al usuario.
El paso a la siguiente requiere su autorización. Fases 1 y 2 autorizadas;
el usuario autorizó expresamente Fase 2 el 23 de octubre de 2026.

## Fase 0 — Inspección inicial

Aceptación: identificar sistema, CPU/RAM, Python y venv, paquetes, API disponible,
backend gráfico y visibilidad de webcam, sin instalar ni modificar el sistema.

Resultado: Fedora 43 x86_64, i7-10510U, 15 GiB reportados, Wayland, Python del
sistema 3.14.3 y `.venv` preexistente con Python 3.12.15. Importaciones de Tasks,
NumPy y OpenCV correctas; detalles y limitaciones en `docs/ENVIRONMENT.md`.
La colisión inicial entre dos distribuciones OpenCV se corrigió con autorización,
conservando únicamente `opencv-contrib-python` en su versión existente.

## Fase 1 — Webcam, pose, manos y brazos (funcional)

Entregables: captura V4L2, Pose Landmarker Lite y Hand Landmarker de Tasks en CPU,
una persona y hasta dos manos, estados independientes, articulaciones dibujadas,
FPS/latencia, errores accionables, diagnóstico y pruebas sin hardware.

Criterios automáticos:

- Los estados distinguen izquierda/derecha anatómicas, ambos brazos y pérdida de pose.
- Histéresis y permanencia de 150 ms filtran oscilaciones breves; no hay estado antiguo tras perder puntos.
- Baja presencia/visibilidad, puntos fuera de imagen, NaN y torso inválido producen `SIN SEGUIMIENTO`.
- Timestamps estrictamente crecientes; pose y manos reciben el mismo frame RGB y tiempo.
- Los fallos de webcam y de inicialización parcial liberan recursos.
- La ejecución no descarga modelos ni instala paquetes; el descargador requiere acción explícita.
- La inferencia real opcional procesa frames sintéticos con los dos modelos sin GUI.

Criterios manuales en GNOME (pendientes hasta observarlos):

- Mostrar video, pose y hasta dos manos con sus articulaciones en la webcam disponible.
- Verificar brazo izquierdo, derecho, ambos, bajada y salida del encuadre; repetir con/sin espejo.
- `Q`, Escape, cierre de ventana y Ctrl+C liberan la cámara y permiten abrirla de nuevo.
- Mantener una sesión de 2 minutos sin errores; registrar FPS, latencia y consumo de RAM.
- Objetivo inicial orientativo: 15 FPS a 640×480. Si no se alcanza, medir a 480×360
  y acordar el compromiso antes de añadir complejidad. No se promete rendimiento sin medición real.

Estado: el usuario confirmó captura de webcam en Fedora, manos y estados de brazos
correctos con el cuerpo completamente visible. Se conserva la lista de pruebas
manuales no confirmadas en `docs/VALIDATION.md`: robustez, espejo, cierres,
legibilidad y mediciones de rendimiento. La autorización explícita permite Fase 2.

## Fase 2 — Robot virtual e imitación ligera (base implementada)

MuJoCo 3.17 ya estaba instalado; no se instalaron paquetes ni cambiaron versiones.

Etapa 2A, antes de conectar webcam: humanoide MJCF anclado, cuatro bisagras,
servos de posición, límites y demo independiente. Verificados modelo, estabilidad,
movimiento de todas las articulaciones y render Mesa llvmpipe por software.

Etapa 2B: calibración inicial estable y transformación continua de hombros/codos/
muñecas a ángulos frontales. Filtrado, consignas <=1.5 rad/s y guardas físicas
<=2 rad/s, vigilancia de 350 ms y recuperación estable tras oclusión. La captura
corre en un proceso acotado; la física sigue activa si se bloquea visión.

Perfil compacto posterior: ventana de gestos 640×480, procesamiento <=480×360,
pose corporal sin modelo de dedos por defecto (`--hands` lo habilita). Calibración
con hombros/codos/muñecas, sin exigir caderas, seis muestras estables y un segundo;
filtro de 80 ms, render 480×360/20 Hz. Los límites de seguridad se conservan.
La suite ampliada pasa 79 pruebas. La medición breve alcanzó ~10 FPS de visión,
63 ms de inferencia y 143 ms de retardo, pendiente de validación humana calibrada.

Criterios automáticos: geometría con aspecto/escala/inclinación correctos,
calibración, lados anatómicos, ángulos firmados de codo, articulaciones y servos
consistentes, límites, retorno seguro, IPC y regresión de Fase 1.

Cámara y dos ventanas verificadas en una ejecución autorizada breve. Hubo 57
observaciones consumidas en 15 segundos, pero no calibración/control humano activo.
Las conversiones y movimiento físico se prueban con poses sintéticas.

Criterios manuales pendientes: confirmar visualmente demo y dos ventanas; imitación
izquierda/derecha/ambos y flexión continua; recalibración; oclusiones y cierres;
mediciones de FPS, latencia, CPU/RAM y funcionamiento del visor bajo GNOME.
Guía y evidencia en `docs/PHASE2.md`. No afirmar aceptación integral por una
imagen sintética ni iniciar Fase 3.

## Fase 2.1 — Movilidad avanzada de brazos (actual, autorizada)

Etapa A: ampliar a tres bisagras de hombro y una de codo por lado, con ejes
locales explícitos, ocho servos, segmentos conectados y torso anclado. Validar
primero sin webcam: límites, estabilidad, lateralidad y cada eje por separado.
La demo completa dura 24 s; el giro axial se muestra con el codo flexionado.

Etapa B: conservar la visión Tasks y transportar world landmarks sin añadir
inferencia. Calibrar offsets, verificar confianza, proporciones, coherencia con
imagen y continuidad. Recuperar el modo 3D con al menos tres muestras y 80 ms
mínimos en el perfil de aplicación (el dominio conserva 150 ms por defecto);
sin calidad suficiente, usar respaldo 2D y neutralizar los ejes de profundidad.
El respaldo `2D-F` conserva una flexión frontal claramente visible orientando
su bisagra en ese plano, sin inferir profundidad.
Un codo extendido no proporciona giro axial observable. El espejo sigue siendo
solo de presentación. Conservar filtro, límites y vigilancia temporal de Fase 2.

Aceptación automática: cinemática inversa comparada con el MJCF real, ambos
lados, poses distintas, cambio de rama de hombro, calidad y pérdida de profundidad,
calibración, límites de los ocho ejes, suavizado, timeout, IPC y regresión de Fase 1.
El texto se reduce y se dibuja en la ventana final; `R` retorna a reposo y pausa,
`C` recalibra y reanuda. No cambiar dependencias ni fases posteriores.

Estado: demo física y gráfica de ocho ejes completadas antes de abrir la webcam;
conversión 3D y retorno seguro verificados con poses sintéticas. La prueba de
20 s abrió las dos ventanas y consumió 207 observaciones, pero no completó la
calibración humana (estado `CALIBRAR: AMBOS BRAZOS`). No declarar aceptación
manual completa. Evidencias y guía en `docs/PHASE2_1.md`.
Suite completa tras corregir la flexión lateral: 108 pruebas superadas sin omisiones. Diagnóstico gráfico previo:
22 presentaciones válidas en tres tamaños de ventana, sin pérdida de opacidad.

El usuario confirmó movimiento hacia delante con profundidad y reportó pérdida
de la flexión lateral en «L». Se reprodujo la regresión con poses sintéticas:
el respaldo 2D perdía el signo del codo y su plano de flexión. Se corrigió con
`2D-F`, se rechazan estimaciones de profundidad que contradicen segmentos
completos en imagen y se verifican movimientos laterales y de profundidad
simultáneos, cada brazo independiente y retorno seguro. Pendiente: confirmar
la «L» corregida con la webcam del usuario, reiniciando el programa.

Ajuste posterior de respuesta y rotaciones: transiciones continuas cerca de
codos extendidos y singularidades del hombro, histéresis del respaldo frontal,
retención de giro seguro ante ángulos fuera del rango representable, filtro
adaptativo de 32–80 ms y estados de Fase 2 con permanencia de 80 ms. Se mantienen
1.5 rad/s de consigna, 2 rad/s físicos, límites y watchdog de 350 ms.
Visión solicita 30 FPS con procesamiento 480×360; el visor solicita 30 FPS,
mantiene plazos de dibujo sin perder cadencia, usa render 384×288 y ventana
800×600. Mostrar los FPS reales de visión, dibujo y frecuencia del control por
separado. Conservar 640×480 para la ventana de cámara y la Fase 1 sin cambios.
Validar con mediciones breves reales, sin prometer FPS de inferencia que la CPU
no alcance, y pruebas de oscilación, continuidad, respuesta y recuperación.
La espera de captura medida de 8–13 ms justificó un lector V4L2 en el proceso
existente: un hilo propietario de cámara, buffer de una imagen y descarte de
fotogramas antiguos. Se conserva el tiempo de entrega del frame para el watchdog;
no se sustituye por la hora de inferencia ni se transfieren imágenes por IPC.
Prueba final de 18 s: 164 observaciones recibidas, 163 consumidas, 138 con control,
aproximadamente 12 FPS de visión y 29.5 FPS de dibujo al final; cero guardas físicas.
La prueba breve no certifica rotaciones precisas ni una mejora de inferencia a
30 FPS. Comprobar manualmente giro, «L», profundidad, espejo y calidad sostenida.
Verificación final del ajuste: 121 pruebas superadas, demo física completa con
el filtro adaptativo y 26 presentaciones gráficas válidas en tres tamaños.

## Fase 2.1 — Aceleración del dibujo (autorizada)

Respaldo previo confirmado: `a2289ea`, rama `main`, coincidente con `origin/main`
en el repositorio del usuario. El usuario autorizó después aprovechar las GPU
existentes para liberar CPU del dibujo. MediaPipe y física permanecen en CPU.

Se eliminó el forzado permanente de software. `--graphics auto|intel|nvidia|software`
permite selección por PCI y comprobación del contexto GLFW real. Auto usa la
selección del sistema y respeta preferencias externas; el respaldo software
sigue disponible. No se cambian controladores ni se instalan paquetes.

Prueba aislada del visor, 90 fotogramas a 30 FPS por modo: CPU del proceso 129.4%
de un núcleo en software, 15.1% con Intel y 29.8% con NVIDIA; mediana de ciclo
26.19, 5.47 y 12.51 ms respectivamente. Intel es la selección normal del equipo.
NVIDIA NV118/nouveau falló después en la prueba de redimensionado (pushbuf rechazado,
salida 139). La investigación posterior aisló el fallo de la copia offscreen:
también falló en XWayland, mientras que el dibujo directo pasó. NVIDIA usa ahora
la ventana directamente y solo se rechaza su copia offscreen. No se reconfiguró
el sistema ni se cambió el controlador.

Integración Intel de 18 s: 207 observaciones recibidas, 206 consumidas, visión
estable 12.8–16.2 FPS, robot 29.7 FPS al final, inferencia final 50 ms y edad 106 ms.
No hubo control humano activo; terminó `SEGURO: SIN TORSO`. Comprobar manualmente
imitación calibrada, giro, «L», espejo y rendimiento sostenido. No iniciar Fase 3.
Evidencia y comandos en `docs/GRAPHICS.md`.
Aceptación automática del ajuste: 128 pruebas superadas; repintado/opacidad con
26 presentaciones en Intel y 26 en software; inicio automático Intel y cierre
correctos. Después se habilitó NVIDIA con dibujo directo, opacidad comprobada y
242 presentaciones correctas en diez ciclos de tres tamaños.

Comparación equivalente posterior: dos mediciones por tarjeta, 150 fotogramas
a 30 FPS y dibujo directo a 800×600, orden Intel/NVIDIA/NVIDIA/Intel. Intel:
5.21–5.22 ms de mediana y 15.0–15.3% de un núcleo; NVIDIA: 12.12–12.18 ms y
24.4–24.7%. Ambas mantuvieron 30 FPS. Intel sigue recomendada para esta carga;
la prueba limitada a 30 FPS no estima el máximo de cada GPU ni su potencia general.
Integración NVIDIA de 18 s: 220 observaciones recibidas, 219 consumidas, visión
estable 14.4–15.9 FPS y dibujo final 30.7 FPS, sin control humano calibrado ni
guardas físicas. Se habilita `--graphics nvidia` sin omitir la protección offscreen.
Validación final: 129 pruebas superadas; 242 presentaciones verificadas durante
30 cambios de tamaño con NVIDIA en modo directo. Pendiente aceptación humana
calibrada y una sesión sostenida; no se afirma resuelto el defecto del controlador.

Pendiente manual: calibración y modos 3D con una persona, movimientos hacia delante,
manos al pecho/cara sin oclusión, cada lado y espejo, pausas/recalibración, cierres
y sesión de dos minutos con CPU/RAM/FPS. Mantener los pendientes de Fase 1 en
`docs/VALIDATION.md`. La profundidad es una estimación monocular, no una medida
precisa. El modelo no evita autocolisiones. No iniciar Fase 3 sin autorización.

## Fase 2.2 — Manos e interacción física (actual, autorizada)

Etapa A: cinco dedos visibles por mano, una bisagra por dedo y tres consignas
(pulgar, índice y sinergia de medio/anular/meñique). Conservar ocho ejes de brazos.
Mesa y tres cubos libres; motor de contacto separado del motor original de ocho
qpos. No welds/mocaps ni correcciones de posición de objetos durante ejecución.

Aceptación A: seis controles independientes, límites y lateralidad; demostrar
aproximación articular, cierre, contactos opuestos, elevación y traslado >9 cm,
liberación y reposo sobre la mesa. Verificar que quitar los contactos impide el
agarre. Validar esta etapa antes de conectar visión.

Resultado A: ambos lados superan la secuencia determinista de 19 s: elevación
13,22 cm, traslado 14,04 cm, contactos de pulgar/índice y liberación con error de
altura <0,01 mm. Control negativo sin contactos: sin elevación ni traslado.
Demo gráfica Intel: ~30 FPS y cumplimiento de los criterios físicos.

Etapa B: reutilizar Hand Landmarker CPU y modelos locales. Asociación mediante
muñecas anatómicas, distancias relativas a palma y ángulos locales; calibración
abierta/puño independiente, filtrado, límites, pérdida y recuperación. Rechazar
ambigüedad, geometría degenerada y datos antiguos. No inferir posición mundial
precisa con el Z de mano ni convertir gestos en animaciones predefinidas.

Etapa C: dos ventanas compactas, FPS/inferencia/edad; modos de demo, teclado,
brazos asistidos y --no-hands. R pausa y abre; C recalibra manos y brazos. El
watchdog permanece fuera del proceso de visión. Cerrar recursos ante errores.

Aceptación B/C: pruebas geométricas y gestos sintéticos, lateralidad/espejo,
filtrado, límites, timeout y recuperación; inferencia real separada. Confirmar
manualmente abierta/puño/pinza por lado, brazos simultáneos, agarre asistido,
oclusiones y cierres. No atribuir precisión humana a pruebas sintéticas.
Resultados y comandos en `docs/PHASE2_2.md`; conservar los pendientes de Fase 1
en `docs/VALIDATION.md` y Fase 2.1 en `docs/PHASE2_1.md`. Sin memoria, SQLite,
aprendizaje ni agentes. Fase 3 no autorizada.

Validación: 156 pruebas automáticas; demos físicas y gráfica de agarre superadas.
Webcam 18 s con ambos modelos: 95 observaciones, visión ~6–8 FPS y dibujo ~30 FPS,
sin completar calibración humana. Reapertura 10 s con --no-hands: 72 observaciones,
visión ~13 FPS y dibujo ~30 FPS, cierre normal. Aceptación humana de gestos e
interacción pendiente; no afirmar seguimiento de manos calibrado con esa prueba.

Corrección posterior al reporte de líneas cruzadas: filtro previo al control
con continuidad temporal normalizada por hombros, confianza >=0,7, referencia
de longitudes con brazos abajo, recuperación y rechazo de inversiones bruscas
de profundidad con imagen casi idéntica. No reetiqueta brazos ni elimina cruces
graduales. Permite segmentos muy cortos en imagen si world es coherente.
El robot retiene 3D hasta 200 ms ante caída breve de profundidad y evita pasar
por 2D mientras 3D se recupera. Colores anatómicos, diagnóstico de calidad,
distancia virtual pinza-cubo y contactos reales para apoyar el agarre asistido.
Validar con pruebas de saltos/intercambios, profundidad ambigua, pérdida 3D,
profundidad válida, brazos independientes y regresión de agarre. Confirmación
humana del ajuste pendiente; no prometer profundidad métrica ni réplica al 100%.
Resultado del ajuste: 170 pruebas superadas, sin omisiones, incluidas las físicas
de agarre. Integración real de 12 s: 46 observaciones, visión ~6 FPS, dibujo
~30 FPS y cierre correcto; sin calibración humana completada. La aceptación
de cruces/profundidad e interacción con una persona sigue pendiente.

Corrección de bloqueo permanente en 2D: completar de forma independiente las
referencias 3D que faltaron durante el arranque, con postura neutral fresca,
seis muestras y un segundo. No recalibrar brazos ya listos ni convertir una
postura de alcance en neutral. Los scores world opcionales ausentes heredan
solo el score del mismo landmark de imagen; conservar scores bajos explícitos
y filtros geométricos. Añadir causa de rechazo y progreso/estado 3D a ambas
ventanas y logs, y aclarar que --assist controla brazos por teclado (M cambia).
Regresiones: arranque sin world, llegada tardía, independencia, brazo hacia
delante, confianza opcional y ventana de calibración a 30 FPS. Aceptación de
profundidad con la persona pendiente; no forzar 3D cuando los datos no son válidos.
Resultado: 179 pruebas superadas. Integración real sin dedos, 18 s: 180
observaciones y cierre normal, pero calibración incompleta (CONFIANZA BRAZO).
No afirmar control 3D humano validado; usar el diagnóstico para comprobarlo.

## Fase 3 — Demostraciones y SQLite (sin iniciar)

Grabación explícita de articulaciones, estados, timestamps y metadatos de versión,
reproducción determinista y sesiones SQLite. Usar `sqlite3` de la biblioteca estándar.
Los videos solo se almacenarán si se solicita expresamente.

Aceptación: guardar/cargar/reproducir una demostración; timestamps y esquema
validados; errores y sesiones incompletas recuperables; datos fuera de Git.

## Fase 4 — Aprendizaje por demostración (sin iniciar)

Definir una tarea pequeña, separar sesiones de entrenamiento y evaluación, medir
una línea base de imitación y entrenar un modelo apropiado para CPU.

Aceptación: evaluación sobre demostraciones no vistas, comparación con la línea
base, semillas/configuración guardadas y errores cuantificados.

## Fase 5 — Refuerzo en simulación (sin iniciar)

Añadir Gymnasium y Stable-Baselines3 tras revisar compatibilidad. Entorno MuJoCo
pequeño, observaciones/acciones limitadas, recompensa explicable y presupuesto de
pasos y tiempo acordado. Primeros experimentos con una sola instancia en CPU.

Aceptación: entorno pasa comprobadores, episodios terminan correctamente,
resultados reproducibles con varias semillas y mejora frente a una política base.
No iniciar entrenamientos prolongados sin autorización.

## Fase 6 — Agentes de coordinación (sin iniciar)

Añadir coordinación de tareas y selección de habilidades verificadas. Los agentes
deben operar mediante interfaces limitadas y observables.

Aceptación: trazabilidad, límites de acción, cancelación y pruebas de fallos.
Cualquier servicio externo o modelo adicional requiere aprobación previa.
