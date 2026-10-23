"""Ventana GLFW con render MuJoCo acotado; sin el ciclo gráfico de simulate."""

import logging
import os
from collections import deque
from threading import Event
from types import TracebackType
from typing import TYPE_CHECKING, Self

from proyectorobot.simulation.graphics import default_draw_target, inspect_context, require_stable_context

if TYPE_CHECKING:
    from proyectorobot.simulation.engine import RobotSimulation


class RobotViewer:
    def __init__(self, simulation: "RobotSimulation", allow_unstable_graphics: bool = False,
                 draw_target: str = "auto") -> None:
        if draw_target not in ("auto", "window", "offscreen"):
            raise ValueError("Destino de dibujo inválido.")
        self.simulation = simulation
        self.window = None
        self.context = None
        self.camera = None
        self.scene = None
        self._glfw = None
        self._gl = None
        self._status = "INICIANDO"
        self._drawing = False
        self.render_width = 384
        self.render_height = 288
        self.graphics_info = None
        self.allow_unstable_graphics = allow_unstable_graphics
        self.draw_target = draw_target
        self.quit = Event()
        self.recalibrate = Event()
        self.reset = Event()
        self.key_events: deque[int] = deque(maxlen=16)

    def on_key(self, key: int) -> None:
        self.key_events.append(key)
        if key in (ord("q"), ord("Q"), 256):
            self.quit.set()
        if key in (ord("c"), ord("C")):
            self.recalibrate.set()
        if key in (ord("r"), ord("R")):
            self.reset.set()
        if self.camera is not None:
            if key == self._glfw.KEY_LEFT:
                self.camera.azimuth -= 5
            elif key == self._glfw.KEY_RIGHT:
                self.camera.azimuth += 5
            elif key == self._glfw.KEY_UP:
                self.camera.elevation = min(75, self.camera.elevation + 5)
            elif key == self._glfw.KEY_DOWN:
                self.camera.elevation = max(-75, self.camera.elevation - 5)

    def _key_callback(self, window, key: int, scan: int, action: int, modifiers: int) -> None:
        if action in (self._glfw.PRESS, self._glfw.REPEAT):
            self.on_key(key)

    def _scroll_callback(self, window, horizontal: float, vertical: float) -> None:
        self.camera.distance = max(1.0, min(8.0, self.camera.distance - 0.15 * vertical))

    def _refresh_callback(self, window) -> None:
        self.redraw()

    def _resize_callback(self, window, width: int, height: int) -> None:
        if min(width, height) > 0:
            self.redraw()

    def __enter__(self) -> Self:
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("Sin sesión gráfica. Prueba --demo --headless primero.")
        import glfw
        import mujoco

        self._glfw = glfw

        if not glfw.init():
            raise RuntimeError(
                "GLFW no pudo abrir la sesión gráfica. Ejecuta desde GNOME; "
                "comprueba Wayland/XWayland y los mensajes de GLFW."
            )
        try:
            glfw.window_hint(glfw.SAMPLES, 0)
            glfw.window_hint(glfw.DOUBLEBUFFER, glfw.TRUE)
            glfw.window_hint(glfw.TRANSPARENT_FRAMEBUFFER, glfw.FALSE)
            glfw.window_hint(glfw.ALPHA_BITS, 0)
            self.window = glfw.create_window(800, 600, "ProyectoRobot - Humanoide MuJoCo", None, None)
            if not self.window:
                raise RuntimeError("No se pudo crear la ventana OpenGL de MuJoCo.")
            glfw.make_context_current(self.window)
            from OpenGL import GL

            self._gl = GL
            self.graphics_info = inspect_context(GL, os.environ.get("PROYECTOROBOT_GRAPHICS", "auto"))
            if self.draw_target == "auto":
                self.draw_target = default_draw_target(self.graphics_info)
            logging.info("OpenGL: %s | %s | %s", self.graphics_info.label,
                         self.graphics_info.renderer, self.graphics_info.version)
            if not self.allow_unstable_graphics:
                require_stable_context(self.graphics_info, self.draw_target)
            if self.graphics_info.device == "software":
                logging.warning("El dibujo usa CPU por software; no hay aceleración GPU activa en este contexto.")
            glfw.swap_interval(0)
            glfw.set_key_callback(self.window, self._key_callback)
            glfw.set_scroll_callback(self.window, self._scroll_callback)
            self.camera = mujoco.MjvCamera()
            self.camera.lookat[:] = (0, 0, 1.05)
            self.camera.distance = 3.1
            self.camera.azimuth = 90
            self.camera.elevation = -8
            self.scene = mujoco.MjvScene(self.simulation.model, maxgeom=128)
            self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            self.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
            self.options = mujoco.MjvOption()
            self.context = mujoco.MjrContext(self.simulation.model, mujoco.mjtFontScale.mjFONTSCALE_100)
            glfw.set_window_refresh_callback(self.window, self._refresh_callback)
            glfw.set_framebuffer_size_callback(self.window, self._resize_callback)
            self.redraw()
            logging.info("Visor MuJoCo/GLFW: destino %s, izquierda verde, derecha naranja.", self.draw_target)
            return self
        except BaseException:
            self.close()
            raise

    def running(self) -> bool:
        return self.window is not None and not self._glfw.window_should_close(self.window) and not self.quit.is_set()

    def update(self, elapsed: float, now: float, status: str, sync: bool = True) -> None:
        self._status = status
        self.simulation.advance(elapsed, now)
        self._glfw.poll_events()
        if sync:
            self.redraw()

    def redraw(self) -> None:
        """Repinta el estado actual sin avanzar física ni consumir observaciones."""
        if self._drawing or self.window is None or self.context is None:
            return
        self._drawing = True
        try:
            self._draw_frame()
        finally:
            self._drawing = False

    def _draw_frame(self) -> None:
        if self.draw_target == "window":
            self._draw_window_frame()
            return
        import mujoco

        width, height = self._glfw.get_framebuffer_size(self.window)
        if min(width, height) <= 0:
            return
        self._glfw.make_context_current(self.window)
        viewport = mujoco.MjrRect(0, 0, self.render_width, self.render_height)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)
        mujoco.mjv_updateScene(self.simulation.model, self.simulation.data, self.options,
                               None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)
        scale = min(width / self.render_width, height / self.render_height)
        destination_width = max(1, round(self.render_width * scale))
        destination_height = max(1, round(self.render_height * scale))
        destination = mujoco.MjrRect((width - destination_width) // 2, (height - destination_height) // 2,
                                     destination_width, destination_height)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.context)
        self._gl.glDisable(self._gl.GL_SCISSOR_TEST)
        self._gl.glColorMask(True, True, True, True)
        self._gl.glDepthMask(True)
        self._gl.glClearColor(0.03, 0.03, 0.03, 1.0)
        self._gl.glClear(self._gl.GL_COLOR_BUFFER_BIT | self._gl.GL_DEPTH_BUFFER_BIT)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)
        mujoco.mjr_blitBuffer(viewport, destination, 1, 0, self.context)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.context)
        overlay = mujoco.MjrRect(8, 8, max(1, width - 16), max(1, height - 16))
        device = self.graphics_info.label if self.graphics_info is not None else "OpenGL sin verificar"
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                          overlay, device + "\n" + self._status + "\nR: reposo | C: recalibrar | Q / ESC: salir",
                          "", self.context)
        self._gl.glColorMask(False, False, False, True)
        self._gl.glClearColor(0.03, 0.03, 0.03, 1.0)
        self._gl.glClear(self._gl.GL_COLOR_BUFFER_BIT)
        self._gl.glColorMask(True, True, True, True)
        self._glfw.swap_buffers(self.window)

    def _draw_window_frame(self) -> None:
        """Render directo sin blit offscreen; mantiene limpieza y presentación opacas."""
        import mujoco

        width, height = self._glfw.get_framebuffer_size(self.window)
        if min(width, height) <= 0:
            return
        self._glfw.make_context_current(self.window)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.context)
        self._gl.glDisable(self._gl.GL_SCISSOR_TEST)
        self._gl.glColorMask(True, True, True, True)
        self._gl.glDepthMask(True)
        self._gl.glClearColor(0.03, 0.03, 0.03, 1.0)
        self._gl.glClear(self._gl.GL_COLOR_BUFFER_BIT | self._gl.GL_DEPTH_BUFFER_BIT)
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(self.simulation.model, self.simulation.data, self.options,
                              None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)
        overlay = mujoco.MjrRect(8, 8, max(1, width - 16), max(1, height - 16))
        device = self.graphics_info.label if self.graphics_info is not None else "OpenGL sin verificar"
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                          overlay, device + "\n" + self._status + "\nR: reposo | C: recalibrar | Q / ESC: salir",
                          "", self.context)
        self._gl.glColorMask(False, False, False, True)
        self._gl.glDisable(self._gl.GL_SCISSOR_TEST)
        self._gl.glClearColor(0.03, 0.03, 0.03, 1.0)
        self._gl.glClear(self._gl.GL_COLOR_BUFFER_BIT)
        self._gl.glColorMask(True, True, True, True)
        self._glfw.swap_buffers(self.window)

    def close(self) -> None:
        if self.window is not None:
            self._glfw.make_context_current(self.window)
            if self.context is not None:
                self.context.free()
                self.context = None
            self.scene = None
            self._glfw.destroy_window(self.window)
            self.window = None
        if self._glfw is not None:
            self._glfw.terminate()

    def __exit__(self, exc_type: type[BaseException] | None,
                 exc_value: BaseException | None, traceback: TracebackType | None) -> None:
        try:
            self.simulation.settle_safe()
        finally:
            self.close()
