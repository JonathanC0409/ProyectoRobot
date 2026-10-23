# Validación de Fase 1

## Evidencia automática

Ejecutar desde la raíz:

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m proyectorobot --check
.venv/bin/python -m proyectorobot --smoke-test
```

Registrar número de pruebas correctas/omitidas, motivo de omisión y código de
salida. `--check` falla ante modelos ausentes o colisión OpenCV; no abre la cámara.
`--smoke-test` requiere ambos bundles y hace inferencia sobre tres frames negros.
No permite concluir precisión ni velocidad con personas.

Resultado local del 22 de octubre de 2026: 35 pruebas correctas, incluida la
inferencia con ambos modelos reales en CPU, sin omisiones. El comando separado
`--smoke-test` también terminó con código 0: tres frames negros, sin detecciones.
Se verificaron las sumas locales de los dos modelos. La sintaxis Python 3.12,
configuración TOML, enlaces locales y espacios finales se comprobaron sin errores.
La corrección autorizada de OpenCV conserva únicamente la distribución contrib.
Después de esa corrección se repitió la suite: 35 pruebas correctas, sin omisiones;
`--check` y `pip check` terminaron con código 0. El diagnóstico conserva el aviso
de que no hay dispositivos de video expuestos en este entorno.
Los resultados no acreditan todavía captura de webcam, reconocimiento humano,
funcionamiento de la ventana ni objetivos de rendimiento.

Verificación posterior de la misma Fase 1: 37 pruebas correctas, sin omisiones;
`--check` y `pip check` con código 0. Se añadieron pruebas de los nueve estados
combinados posibles y de los mensajes en pantalla con/sin espejo. El panel ahora
identifica explícitamente mano izquierda, derecha, ambas, manos abajo y
seguimiento parcial, sin atribuir un estado a un brazo oculto. Se reutilizaron
el venv y los modelos existentes, sin nuevas instalaciones ni descargas.

Ajuste de legibilidad: ventana inicial de 1280×720, estados en un panel lateral
y texto más grande y grueso. Las opciones `--window-width` y `--window-height`
permiten ampliar solo la visualización, conservando la resolución de inferencia.
Resultado: 38 pruebas correctas, sin omisiones, incluida una comprobación de que
el texto cabe en el panel incluso con captura de 320×240. Falta confirmar el
tamaño y la legibilidad en la pantalla del usuario al volver a abrir la aplicación.

## Confirmación del usuario y comprobaciones restantes

El 23 de octubre de 2026 el usuario confirmó que Fase 1 funciona con su webcam
en Fedora 43, detecta manos y reconoce estados de brazos con el cuerpo completo
visible. Autorizó expresamente Fase 2. Se conserva pendiente lo que no fue
comprobado individualmente: espejo, robustez/oclusiones, cierres, legibilidad
después del ajuste y mediciones prolongadas. La validación de Fase 2 está en
[PHASE2.md](PHASE2.md).

Usar la webcam ya disponible en una sesión GNOME con iluminación suficiente y
el torso completo en imagen. Cerrar aplicaciones que puedan ocupar la webcam.
No grabar videos durante esta comprobación.

| Acción | Resultado esperado | Resultado observado |
| --- | --- | --- |
| Abrir la aplicación | Video y articulaciones superpuestas sin errores | Confirmado por el usuario |
| Brazos abajo durante >150 ms | Ambos `ABAJO` | Pendiente |
| Levantar solo el izquierdo | `MANO IZQUIERDA ARRIBA` y solo izquierdo `ARRIBA` | Pendiente |
| Levantar solo el derecho | `MANO DERECHA ARRIBA` y solo derecho `ARRIBA` | Pendiente |
| Levantar ambos | `AMBAS MANOS ARRIBA` y ambos `ARRIBA` | Pendiente |
| Bajar cada brazo | Cambio independiente después del tiempo de estabilidad | Pendiente |
| Oscilar brevemente cerca del hombro | Sin parpadeo continuo de estados | Pendiente |
| Ocultar muñeca o salir del encuadre | `SIN SEGUIMIENTO`, sin conservar arriba | Pendiente |
| Ocultar solo un lado | `SEGUIMIENTO PARCIAL` y estado del lado observable | Pendiente |
| Volver a entrar | Recuperación tras una observación estable | Pendiente |
| Mostrar una/dos manos | Articulaciones de manos y contador correcto | Detección confirmada; contador individual pendiente |
| Repetir con `--no-mirror` | Lados anatómicos iguales y dibujo alineado | Pendiente |
| Salir con Q/Escape/cierre de ventana/Ctrl+C | Cámara liberada, reapertura correcta | Pendiente |
| Mantener 2 minutos a 640×480 | Sin errores; registrar FPS/latencia/RAM | Pendiente |

Anotar fecha, intérprete, versiones, resolución solicitada/real y FPS observados.
Para observar CPU/RAM usar el Monitor del sistema de GNOME. Objetivo orientativo:
15 FPS; si el equipo no lo alcanza, comparar con 480×360 y presentar mediciones.

La Fase 2 fue autorizada por el usuario tras su confirmación del funcionamiento
con webcam, no por las pruebas unitarias únicamente. Fase 3 no autorizada.
