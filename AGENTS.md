# ProyectoRobot: instrucciones de trabajo

## Alcance y seguridad

- Implementar únicamente la fase autorizada. Presentar resultados y limitaciones antes de avanzar.
- Priorizar CPU, memoria acotada y simulación ligera en Fedora/GNOME Wayland.
- No comprar ni presuponer hardware físico; usar la webcam disponible únicamente para visión.
- Pedir autorización antes de instalar/desinstalar paquetes, descargar modelos, modificar el sistema o realizar cambios importantes de arquitectura.
- Inspeccionar el venv existente antes de modificarlo. Nunca usar `sudo pip`, cambiar el Python del sistema ni instalar todas las fases juntas.
- No grabar, persistir ni transmitir imágenes o demostraciones por defecto. Las descargas deben ser explícitas, independientes de la ejecución.

## Arquitectura

Paquete Python `proyectorobot/`, ejecutable desde la raíz sin instalación editable:

- `vision/`: webcam, MediaPipe Tasks, adaptación a tipos propios y dibujo.
- `control/`: interpretación de articulaciones y estados; sin dependencias de OpenCV/MediaPipe.
- `simulation/`: humanoide MJCF, motor MuJoCo y aplicación de Fase 2.
- `control/retargeting.py`: calibración y conversión de pose frontal a ángulos.
- `control/spatial.py`: cinemática 3D, calidad de world landmarks y respaldo conservador.
- `control/pose_quality.py`: validación temporal y geométrica previa al control
  y dibujo de Fase 2; enmascarar puntos dudosos sin intercambiar lados.
- `control/motion.py`: suavizado, límites y vigilancia de seguimiento.
- `control/hands.py`, `control/gestures.py`: consignas, geometría y calibración
  independiente de manos, sin OpenCV/MediaPipe.
- `simulation/hand_model.py`, `assets/hand.xml`: manos MJCF y escena de mesa.
- `simulation/manipulation.py`, `grasp_demo.py`, `hands_app.py`: Fase 2.2 aislada;
  no aplicar las guardas de ocho qpos de `engine.py` a objetos libres.
- `learning/`: reservado para aprendizaje por demostración y refuerzo.
- `agents/`: reservado para coordinación futura; sin servicios externos por defecto.
- `data/`: contratos de datos; SQLite y grabación se incorporarán en su fase.
- `app.py`: ciclo de captura → inferencia → estados → visualización.
- `diagnostics.py`: comprobación local sin abrir cámara, ventanas ni red.
- `models/`: modelos locales ignorados por Git; `scripts/`: herramientas explícitas.
- `tests/`: pruebas `unittest` sin webcam ni GUI; separar inferencia real opcional.

El dominio usa coordenadas normalizadas de imagen (x hacia la derecha, y hacia abajo),
tiempo monotónico en milisegundos y lados anatómicos de la persona. El espejo es solo
de presentación. No convertir coordenadas de imagen en órdenes de articulaciones
del robot sin calibración y límites en la fase de control.
`Observation.pose_world` conserva coordenadas métricas estimadas, sin aplicar los
límites [0,1] de imagen. Fase 2.1 autoriza ocho ejes de brazos y uso condicionado de
esa estimación monocular. Sin calidad suficiente, flexión/rotación de hombro neutras,
salvo el respaldo frontal `2D-F`: orienta la bisagra según el signo del codo visible
cuando ambos segmentos conservan longitud proyectada. Representa el plano frontal,
sin afirmar que se ha reconstruido profundidad o medido giro axial real.

## Convenciones

- Python 3.12 en `.venv`, PEP 8, cuatro espacios, tipos en interfaces y dataclasses para contratos.
- Identificadores en inglés; documentación y mensajes de usuario en español. Evitar variables de una letra.
- Funciones pequeñas, errores accionables y recursos administrados con `with`/`finally`.
- Usar API `mediapipe.tasks`, nunca `mp.solutions`. Delegado CPU explícito.
- No añadir comentarios inline; explicar decisiones en docstrings o documentación.
- Una sola distribución OpenCV por venv; preferir `opencv-contrib-python` porque MediaPipe la requiere. No usar variantes headless para ventanas.
- Mantener dependencias por fase. No importar MuJoCo, Gymnasium, SB3 o agentes desde Fase 1. La Fase 2 está autorizada; las siguientes no.
- No introducir grabación, redes, hilos, GPU o infraestructura sin una necesidad validada.
- La aceleración gráfica de Fase 2 está autorizada: render OpenGL con Intel/NVIDIA
  existentes, contexto real verificado y respaldo software explícito. MediaPipe
  y física permanecen en CPU. No instalar ni cambiar controladores sin autorización.
  NV118/MX130 usa dibujo directo a ventana; mantener bloqueada la copia offscreen
  que falló al redimensionar. Comparar GPU con la misma resolución y carga.

## Verificación

Desde la raíz: `.venv/bin/python -m unittest discover -s tests -v`.
`--check` no abre webcam; `--smoke-test` prueba modelos con imágenes sintéticas sin GUI.
Documentar separadamente pruebas automáticas, inferencia real y aceptación manual.
No declarar aceptada la Fase 1 hasta comprobar webcam, lateralidad, manos y cierre
en la sesión gráfica del usuario. No hacer commits ni cambiar de fase sin solicitud.
El usuario confirmó captura y reconocimiento con cuerpo visible y autorizó Fase 2;
conservar en documentación las comprobaciones manuales de Fase 1 no confirmadas.
La simulación debe seguir respondiendo aunque se bloquee visión: vigilancia por
tiempo monotónico, cola acotada y retorno suave a postura segura. No guardar video.
En Fase 2, `CameraStream` solapa lectura V4L2 con inferencia tras medir su espera:
un hilo propietario de la webcam y una sola imagen pendiente, con timestamp real
de entrega y cierre acotado. Fase 1 conserva `Camera` síncrona. No añadir más
paralelismo sin nuevas mediciones; el proceso aislado mantiene la vigilancia.
La Fase 2.2 está autorizada. Cubos con freejoints, contactos y fricción: no escribir
sus qpos durante ejecución, añadir welds/mocaps ni simular agarre por proximidad.
Las igualdades de dedos solo acoplan medio/anular/meñique. Validar el agarre físico
antes de visión. La pérdida de visión abre las manos y retorna a postura segura;
un objeto sostenido puede caer. El teclado es un modo asistido explícito, no una
reconstrucción de posición monocular. Conservar los comandos y modelos previos.
