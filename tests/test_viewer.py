"""Regresiones de composición opaca y repintado sin abrir ventanas."""

from contextlib import ExitStack
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

try:
    import mujoco
    MUJOCO_AVAILABLE = True
except (ImportError, OSError):
    MUJOCO_AVAILABLE = False

from proyectorobot.simulation.viewer import RobotViewer


@unittest.skipUnless(MUJOCO_AVAILABLE, "MuJoCo no disponible")
class ViewerTests(unittest.TestCase):
    def make_viewer(self):
        viewer = RobotViewer(MagicMock())
        viewer.window = object()
        viewer.context = SimpleNamespace(currentBuffer=mujoco.mjtFramebuffer.mjFB_WINDOW)
        viewer.scene = object()
        viewer.camera = object()
        viewer.options = object()
        viewer._glfw = MagicMock()
        viewer._glfw.get_framebuffer_size.return_value = (960, 720)
        viewer._gl = MagicMock()
        viewer._gl.GL_SCISSOR_TEST = 0x0C11
        viewer._gl.GL_COLOR_BUFFER_BIT = 0x4000
        viewer._gl.GL_DEPTH_BUFFER_BIT = 0x0100
        return viewer

    def test_direct_render_never_blits_and_presents_opaque_after_renderer_state_changes(self):
        viewer = self.make_viewer()
        viewer.draw_target = "window"
        state = {"alpha": 0, "clear_alpha": 0, "mask": (True,) * 4, "clipped": True, "clears": 0}

        def set_buffer(buffer, context):
            self.assertEqual(buffer, mujoco.mjtFramebuffer.mjFB_WINDOW)
            context.currentBuffer = buffer

        def clear(mask):
            self.assertFalse(state["clipped"])
            if state["mask"] == (True,) * 4:
                self.assertTrue(mask & viewer._gl.GL_DEPTH_BUFFER_BIT)
                state["clears"] += 1
            if state["mask"][3]:
                state["alpha"] = state["clear_alpha"]

        def render(rectangle, scene, context):
            self.assertEqual((rectangle.width, rectangle.height), (960, 720))
            state.update(alpha=0, clear_alpha=0, clipped=True)

        def present(window):
            self.assertEqual(state["alpha"], 1)
            self.assertEqual(state["mask"], (True,) * 4)
            self.assertFalse(state["clipped"])
            self.assertEqual(viewer.context.currentBuffer, mujoco.mjtFramebuffer.mjFB_WINDOW)

        viewer._gl.glDisable.side_effect = lambda flag: state.update(clipped=False)
        viewer._gl.glColorMask.side_effect = lambda *mask: state.update(mask=mask)
        viewer._gl.glClearColor.side_effect = lambda red, green, blue, alpha: state.update(clear_alpha=alpha)
        viewer._gl.glClear.side_effect = clear
        viewer._glfw.swap_buffers.side_effect = present
        with ExitStack() as stack:
            stack.enter_context(patch("mujoco.mjr_setBuffer", side_effect=set_buffer))
            stack.enter_context(patch("mujoco.mjv_updateScene"))
            stack.enter_context(patch("mujoco.mjr_render", side_effect=render))
            stack.enter_context(patch("mujoco.mjr_overlay"))
            blit = stack.enter_context(patch("mujoco.mjr_blitBuffer"))
            viewer.redraw()
            viewer.redraw()
            blit.assert_not_called()
        self.assertEqual(state["clears"], 2)
        self.assertEqual(viewer._glfw.swap_buffers.call_count, 2)

    def test_every_present_is_fully_cleared_opaque_and_on_window_buffer(self) -> None:
        viewer = self.make_viewer()
        state = {"clipped": True, "mask": (True,) * 4, "alpha": 0, "full_clears": 0}

        def set_buffer(buffer, context):
            context.currentBuffer = buffer

        def disable(capability):
            self.assertEqual(capability, viewer._gl.GL_SCISSOR_TEST)
            state["clipped"] = False

        def clear(mask):
            self.assertFalse(state["clipped"])
            self.assertEqual(viewer.context.currentBuffer, mujoco.mjtFramebuffer.mjFB_WINDOW)
            if state["mask"] == (True,) * 4:
                self.assertTrue(mask & viewer._gl.GL_DEPTH_BUFFER_BIT)
                state["full_clears"] += 1
            if state["mask"][3]:
                state["alpha"] = 1

        def blit(*args):
            self.assertFalse(state["clipped"])
            self.assertEqual(viewer.context.currentBuffer, mujoco.mjtFramebuffer.mjFB_OFFSCREEN)
            state["alpha"] = 0

        def present(window):
            self.assertEqual(state["alpha"], 1)
            self.assertEqual(state["mask"], (True,) * 4)
            self.assertEqual(viewer.context.currentBuffer, mujoco.mjtFramebuffer.mjFB_WINDOW)

        viewer._gl.glDisable.side_effect = disable
        viewer._gl.glColorMask.side_effect = lambda *mask: state.update(mask=mask)
        viewer._gl.glClear.side_effect = clear
        viewer._glfw.swap_buffers.side_effect = present
        with ExitStack() as stack:
            stack.enter_context(patch("mujoco.mjr_setBuffer", side_effect=set_buffer))
            stack.enter_context(patch("mujoco.mjr_blitBuffer", side_effect=blit))
            for function in ("mjv_updateScene", "mjr_render", "mjr_overlay"):
                stack.enter_context(patch.object(mujoco, function))
            viewer.redraw()
            viewer.redraw()
        self.assertEqual(state["full_clears"], 2)
        self.assertEqual(viewer._glfw.swap_buffers.call_count, 2)

    def test_expose_repaints_latest_status_without_advancing_physics(self) -> None:
        viewer = self.make_viewer()
        with patch.object(viewer, "_draw_frame") as draw:
            viewer.update(0.02, 10, "SIGUIENDO", sync=False)
            draw.assert_not_called()
            viewer._refresh_callback(viewer.window)
            self.assertEqual(viewer._status, "SIGUIENDO")
            draw.assert_called_once()
        viewer.simulation.advance.assert_called_once_with(0.02, 10)

    def test_small_overlay_is_drawn_on_final_window_without_scene_upscaling(self):
        viewer = self.make_viewer()
        viewer._status = "SIGUIENDO I:3D D:3D"
        state = {"buffer": None}
        def set_buffer(buffer, context):
            state["buffer"] = buffer
        with ExitStack() as stack:
            stack.enter_context(patch("mujoco.mjr_setBuffer", side_effect=set_buffer))
            for function in ("mjv_updateScene", "mjr_render", "mjr_blitBuffer"):
                stack.enter_context(patch.object(mujoco, function))
            def check_overlay(font, position, rectangle, label, right, context):
                self.assertEqual(state["buffer"], mujoco.mjtFramebuffer.mjFB_WINDOW)
                self.assertEqual(position, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT)
                self.assertEqual(rectangle.width, 944)
                self.assertIn("I:3D D:3D", label)
                self.assertIn("R: reposo", label)
            overlay = stack.enter_context(patch("mujoco.mjr_overlay", side_effect=check_overlay))
            viewer.redraw()
            overlay.assert_called_once()

    def test_resize_repaints_but_zero_size_and_closed_viewer_do_not_draw(self) -> None:
        viewer = self.make_viewer()
        with patch.object(viewer, "_draw_frame") as draw:
            viewer._resize_callback(viewer.window, 720, 720)
            draw.assert_called_once()
            viewer._resize_callback(viewer.window, 0, 0)
            self.assertEqual(draw.call_count, 1)
            viewer.window = None
            viewer._refresh_callback(None)
            self.assertEqual(draw.call_count, 1)
        viewer.simulation.advance.assert_not_called()

    def test_refresh_during_render_cannot_recursively_render(self) -> None:
        viewer = self.make_viewer()
        with patch.object(viewer, "_draw_frame", side_effect=lambda: viewer._refresh_callback(viewer.window)) as draw:
            viewer.redraw()
            draw.assert_called_once()
        self.assertFalse(viewer._drawing)
