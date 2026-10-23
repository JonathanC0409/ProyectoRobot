"""Prueba gráfica real de repintado, sin webcam, capturas ni archivos de video."""

import logging
import os
import time
from unittest.mock import patch


def main() -> int:
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
    os.environ.setdefault("LP_NUM_THREADS", "2")
    import mujoco
    import numpy as np
    from proyectorobot.simulation.engine import RobotSimulation
    from proyectorobot.simulation.viewer import RobotViewer

    failures = []
    checked = 0
    with RobotViewer(RobotSimulation()) as viewer:
        from OpenGL import GL

        glfw = viewer._glfw
        original_swap = glfw.swap_buffers

        def check_and_swap(window) -> None:
            nonlocal checked
            width, height = glfw.get_framebuffer_size(window)
            if min(width, height) <= 0:
                original_swap(window)
                return
            original_buffer = viewer.context.currentBuffer
            if original_buffer != mujoco.mjtFramebuffer.mjFB_WINDOW:
                failures.append("Se presenta con el framebuffer offscreen activo")
            if GL.glIsEnabled(GL.GL_SCISSOR_TEST):
                failures.append("El recorte OpenGL sigue activo al presentar")
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, viewer.context)
            for horizontal in (0, width // 2, width - 1):
                for vertical in (0, height // 2, height - 1):
                    raw = GL.glReadPixels(horizontal, vertical, 1, 1, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE)
                    pixel = np.frombuffer(raw, dtype=np.uint8)
                    if tuple(pixel[:3]) == (255, 0, 255):
                        failures.append("Quedaron píxeles del frame anterior fuera del área repintada")
                    if pixel[3] != 255:
                        failures.append("El frame contiene transparencia")
            mujoco.mjr_setBuffer(original_buffer, viewer.context)
            checked += 1
            original_swap(window)

        with patch.object(glfw, "swap_buffers", side_effect=check_and_swap):
            for size in ((960, 720), (700, 700), (900, 540)):
                glfw.set_window_size(viewer.window, *size)
                for _ in range(6):
                    if not viewer.running():
                        raise RuntimeError("La ventana de prueba se cerró antes de terminar")
                    glfw.poll_events()
                    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, viewer.context)
                    GL.glDisable(GL.GL_SCISSOR_TEST)
                    GL.glColorMask(True, True, True, True)
                    GL.glClearColor(1.0, 0.0, 1.0, 1.0)
                    GL.glClear(GL.GL_COLOR_BUFFER_BIT)
                    viewer.update(1 / 60, time.monotonic(), "Prueba de repintado", sync=True)
                    time.sleep(0.03)
    for failure in sorted(set(failures)):
        logging.error("%s", failure)
    logging.info("Frames comprobados: %s; tamaños de ventana: 3.", checked)
    return 1 if failures or checked < 18 else 0


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    raise SystemExit(main())
