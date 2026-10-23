# Aceleración gráfica de ProyectoRobot

El usuario confirmó la subida del respaldo `a2289ea` a
`https://github.com/JonathanC0409/ProyectoRobot` antes de autorizar este ajuste.
Se comprobó que `HEAD` y `origin/main` coinciden con esa revisión.

MediaPipe conserva el delegado CPU y MuJoCo calcula la física en CPU. El cambio
acelera el dibujo OpenGL con las GPU existentes. No instala paquetes, modelos,
controladores ni CUDA. No modifica GNOME ni configuración gráfica global.
No graba ni transmite imágenes.

## Ejecutar

La selección normal es automática; en este equipo corresponde a Intel UHD:

```bash
.venv/bin/python -m proyectorobot.simulation --camera 0
```

Para seleccionar explícitamente una GPU o el respaldo por CPU:

```bash
.venv/bin/python -m proyectorobot.simulation --graphics intel
.venv/bin/python -m proyectorobot.simulation --graphics nvidia
.venv/bin/python -m proyectorobot.simulation --graphics software
```

`auto` respeta preferencias externas como `DRI_PRIME` y `LIBGL_ALWAYS_SOFTWARE`.
Si el entorno fuerza software, el visor lo indica; `--graphics intel` lo reemplaza
por selección explícita de hardware. Las variables afectan solo al proceso actual
y sus hijos; MediaPipe sigue en CPU.

Los modos explícitos Intel/NVIDIA consultan los dispositivos PCI por fabricante,
sin fijar direcciones de una computadora. Usan `DRI_PRIME=pci-…` y desactivan el
forzado de software antes de inicializar OpenGL. El visor lee `GL_VENDOR`,
`GL_RENDERER` y `GL_VERSION` del contexto real y rechaza discrepancias. El proveedor
Mesa por sí solo no significa software: NVIDIA también lo utiliza.

`--check` y la demo headless sin snapshot no abren contexto ni requieren GPU.
Ante un fallo gráfico se puede usar `--graphics software`. No se instala ni
reconfigura un controlador automáticamente.

## Tarjetas y estabilidad verificadas

Fedora 43, GNOME Wayland, Python 3.12.15, MuJoCo 3.17.0 y Mesa 25.3.6.

| Tarjeta | Controlador | Renderizador GLFW/MuJoCo | Resultado |
| --- | --- | --- | --- |
| Intel UHD, Comet Lake | i915 | `Mesa Intel(R) UHD Graphics (CML GT2)` | Dibujo, cámara y redimensionado correctos |
| NVIDIA GeForce MX130 | nouveau | `NV118` | Dibujo directo, cámara y redimensionado correctos; evitar copia offscreen |
| CPU | Mesa software | `llvmpipe (LLVM 21.1.8, 256 bits)` | Respaldo verificado |

Se comprobaron los contextos del propio visor, además de consultar `glxinfo` y
`switcherooctl`. No se asumió que una prueba GLX demuestre por sí sola el uso de
esa tarjeta por GLFW en Wayland.

La copia offscreen de NVIDIA produjo `kernel rejected pushbuf: No such device`
y salida 139 al redimensionar. El kernel registró un canal nouveau deshabilitado.
La prueba con la variante X11/XWayland de GLFW también falló: cambiar únicamente
el sistema de ventanas no lo resolvió. Ya estaban instaladas ambas variantes de
GLFW y XWayland; no fue necesario instalar paquetes ni cambiar la sesión GNOME.

El dibujo directo a la ventana evitó el fallo observado. Primero pasó una prueba
aislada de 120 fotogramas y cuatro tamaños; después pasó el verificador de píxeles
y diez ciclos de redimensionado. El visor selecciona automáticamente `window`
para NV118 y `offscreen` para Intel/software. El modo directo mantiene limpieza
de color/profundidad, texto pequeño y alfa opaco, sin ejecutar `mjr_blitBuffer`.
La protección `require_stable_context` continúa bloqueando únicamente la copia
offscreen NV118. No se eliminó la comprobación de GPU ni se omiten límites de
seguimiento. Esta es una solución comprobada al nivel de aplicación, no un
diagnóstico completo del defecto interno de Mesa/nouveau.

No forzar `--draw-target offscreen` con NVIDIA en los diagnósticos: sigue siendo
una ruta conocida como problemática. No se modificó nouveau ni se instaló el
controlador propietario; tampoco se cambió gestión de energía o frecuencias.

## Comparación aislada inicial de dibujo

Comandos de diagnóstico seguros para los modos verificados, sin webcam ni capturas:

```bash
.venv/bin/python -m scripts.check_robot_graphics --graphics intel
.venv/bin/python -m scripts.check_robot_graphics --graphics software
```

Hay ocho ciclos de calentamiento y luego 90 fotogramas a 30 FPS por defecto.
La medición incluye actualización física/render y espera OpenGL con `glFinish`.
El resultado JSON registra CPU, FPS y tiempos de ciclo. `--frames` y `--fps`
permiten ajustar la duración. Cerrar una ventana antes de terminar produce un
error y no se considera una medición válida.

| Modo probado | FPS | CPU del proceso, un núcleo = 100% | Mediana del ciclo | Percentil 95 |
| --- | --- | --- | --- | --- |
| Software | 30.0 | 129.4% | 26.19 ms | 29.98 ms |
| Intel | 30.0 | 15.1% | 5.47 ms | 6.83 ms |
| NVIDIA, prueba inicial | 30.0 | 29.8% | 12.51 ms | 18.81 ms |

El proceso consumió 3.882 s de CPU en software, 0.454 s con Intel y 0.895 s con
NVIDIA durante tres segundos por modo. Intel redujo aproximadamente 88% el tiempo
de CPU de esta prueba del visor. No representa 88% menos de CPU de toda la aplicación:
no se ejecutó MediaPipe aquí. Más del 100% indica varios hilos sobre más de un
núcleo; no es el porcentaje total del equipo.

La elección de Intel se basa en esta escena pequeña y la estabilidad observada;
no implica una clasificación general de rendimiento de tarjetas. Las cifras
pueden variar por escritorio, temperatura, tamaño de ventana y controlador.

## Comparación equivalente con NVIDIA corregida

Dos repeticiones por tarjeta en orden Intel/NVIDIA/NVIDIA/Intel, procesos
separados, misma escena, ventana 800×600 y dibujo directo para ambas GPU.
Cada repetición incluye ocho ciclos de calentamiento y 150 fotogramas a 30 FPS.

```bash
.venv/bin/python -m scripts.check_robot_graphics --graphics intel --draw-target window --frames 150
.venv/bin/python -m scripts.check_robot_graphics --graphics nvidia --draw-target window --frames 150
```

| Tarjeta y repetición | FPS | CPU, un núcleo = 100% | Mediana del ciclo | Percentil 95 |
| --- | --- | --- | --- | --- |
| Intel 1 | 30.0 | 15.0% | 5.21 ms | 6.73 ms |
| NVIDIA 1 | 30.0 | 24.4% | 12.12 ms | 13.33 ms |
| NVIDIA 2 | 30.0 | 24.7% | 12.18 ms | 12.87 ms |
| Intel 2 | 30.0 | 15.3% | 5.22 ms | 7.96 ms |

Intel requirió menos tiempo de ciclo y CPU en esta carga y conserva la selección
predeterminada del sistema. NVIDIA puede usarse mediante dibujo directo con sus
controladores actuales. Ambas cumplieron el límite de 30 FPS: este ensayo no
mide su máximo de FPS ni demuestra que Intel sea mejor en otros programas.

## Integración con webcam

```bash
.venv/bin/python -m proyectorobot.simulation --graphics intel --duration 18
```

Se recibieron 207 observaciones y se consumieron 206. Tras el arranque, visión
registró 12.8, 14.5, 14.5, 13.6, 13.8 y 16.2 FPS. Al final: dibujo 29.7 FPS,
ciclo principal 59.5 Hz, inferencia 50 ms y edad de pose 106 ms. Terminó con código
0 y sin guardas físicas. La edad no mide exposición ni buffering interno del sensor.

No hubo control humano activo: terminó `SEGURO: SIN TORSO`. Esto comprueba cámara
y visor acelerado, no precisión de imitación. La escena humana y carga no fueron
idénticas a pruebas software anteriores; no se garantiza una ganancia concreta
de inferencia ni 30 FPS de visión.

La prueba posterior con `--graphics nvidia --duration 18` recibió 220 observaciones
y consumió 219, con 14.4–15.9 FPS de visión tras el arranque. Al final: 30.7 FPS
de dibujo, 59.4 Hz de control, inferencia 49 ms y edad 65 ms; código 0, sin guardas
físicas y estado `SEGURO: SIN TORSO`. Una ventana corta de conteo puede superar
ligeramente el límite nominal de FPS. No hubo imitación calibrada; no atribuir
las diferencias de inferencia a la GPU, pues la escena humana no estuvo controlada.

## Verificación y pendientes

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m scripts.check_robot_repaint --graphics intel
.venv/bin/python -m scripts.check_robot_repaint --graphics nvidia --cycles 10
.venv/bin/python -m scripts.check_robot_repaint --graphics software
```

Las pruebas sin GUI cubren PCI, preferencias externas, respaldo software,
identificación Intel/NVIDIA/software, contexto ausente, discrepancia de selección
y protección frente al fallo nativo conocido. El diagnóstico gráfico comprueba
opacidad y repintado al redimensionar. No cambia seguimiento ni límites articulares.

Validación inicial: 128 pruebas superadas sin omisiones en 10.587 s; Intel y software
pasaron 26 presentaciones cada uno en tres tamaños de ventana. La demo de inicio
normal, sin indicar GPU, abrió con Intel y terminó con código 0 y postura segura.
La prueba inicial NVIDIA con copia offscreen falló y no se presenta como pasada.
Tras corregir la ruta, NVIDIA pasó 26 presentaciones en tres tamaños y después
242 presentaciones en diez ciclos, sin fallos de opacidad ni cierre. Las pruebas
sin GUI también verifican que el modo directo no copie entre búferes y restaure
el alfa/recorte aunque el renderizador altere el estado OpenGL.

Validación final del dibujo directo: 129 pruebas superadas sin omisiones en
16.054 s. La prueba de NVIDIA recorrió diez ciclos de tres tamaños (30 cambios),
con 242 presentaciones comprobadas. El fallo de la copia offscreen sigue
documentado y esa ruta permanece bloqueada en el visor normal de NV118.

Pendiente con una persona: calibración, profundidad, «L», giro, espejo y pérdida
de seguimiento; sesión de dos minutos con FPS/CPU/RAM, cierre y reapertura.
Se conservan los pendientes de Fase 1 en [VALIDATION.md](VALIDATION.md).
No se inicia Fase 3.

Fuentes primarias:
[selección de GPU en Mesa](https://docs.mesa3d.org/envvars.html#dri-prime) y
[contextos de GLFW en Wayland](https://www.glfw.org/docs/latest/window_guide.html#window_hints_ctx),
[selección de variante pyGLFW X11/Wayland](https://github.com/FlorianRhiem/pyGLFW).
