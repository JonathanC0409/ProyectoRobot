# Skill local de MuJoCo para agentes

Instalada el 23 de octubre de 2026 en
`.agents/skills/mujoco-python/`, dentro de este proyecto.

- Origen: <https://github.com/prathje/agentic_mujoco_skills>.
- Revisión: `dfcfc5cce47af784c0a2d68e039775561dd5d6e8`.
- Directorio de origen: `skills/mujoco-python/`.
- Licencia: Apache-2.0; copia incluida en el directorio instalado.
- Instalación: clonación Git, revisión local y copia mediante `skill-installer`,
  fijada a la revisión indicada. Los archivos originales se conservan sin cambios.

## Contenido y activación

`SKILL.md` contiene los metadatos y las instrucciones de uso de la API Python.
`scripts/` conserva `sim_server.py`, `sim_client.py` y `sim_mcp.py`.
`references/` conserva `SETUP.md`, `HTTP_API.md` y `MCP_SERVER.md`.

Codex descubre las skills de `.agents/skills` y puede seleccionarlas cuando la
tarea coincide con su descripción. Estará disponible en el próximo turno.
También se puede solicitar explícitamente con `$mujoco-python`.
Si no aparece, reiniciar Codex. Este comportamiento se describe en la
[documentación oficial](https://developers.openai.com/codex/skills/).

La instalación proporciona instrucciones y utilidades al agente. Los scripts
HTTP y MCP no se ejecutan automáticamente. No se registró un servidor MCP,
no se abrió ningún puerto y no se modificó la aplicación ni sus dependencias.
Se mantienen las restricciones de `AGENTS.md`, incluida la autorización por fase,
el uso de CPU y los límites de control del robot.

## Compatibilidad y límites

La comprobación local usa Python 3.12.15 y MuJoCo 3.17.0 del venv existente.
El servidor HTTP utiliza la biblioteca estándar, MuJoCo y NumPy; las capturas
requieren Pillow. El cliente utiliza la biblioteca estándar. La integración MCP
requiere el paquete adicional `mcp`, que no está instalado y no se añadió.

La guía original incluye ejemplos genéricos de otros robots, capturas y descargas.
Su presencia no autoriza a ejecutarlos. Los ejemplos que suponen una base libre
o una correspondencia directa entre articulaciones y actuadores deben adaptarse
al MJCF del proyecto. El renderizado depende del contexto OpenGL disponible;
instalar la skill no comprueba EGL ni el comportamiento de ventanas en Wayland.

## Verificación de esta instalación

- Comparación de los siete archivos de la skill con el clon de la revisión fijada.
- Validación de metadatos, referencias locales y sintaxis de los tres scripts.
- Prueba del servidor importado como biblioteca con el humanoide del proyecto:
  estado, control, avance físico, lectura de articulaciones y reinicio.
- La prueba del servidor llama directamente a sus manejadores, sin sockets,
  webcam, ventanas, capturas ni servicios en segundo plano.
- El script MCP se verifica por sintaxis; su ejecución requiere una instalación
  y configuración independientes, no incluidas en esta tarea.
- Suite existente: `.venv/bin/python -m unittest discover -s tests -v`;
  83 pruebas superadas.

La detección de la skill por la interfaz de Codex se comprueba en el próximo
turno. La aceptación manual de la webcam y la simulación conserva el estado
documentado en `PLAN.md` y `README.md`.
