"""Selección de GPU y detección de respaldo real sin contexto gráfico ni red."""

from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from contextlib import ExitStack

from proyectorobot.simulation.graphics import (
    GraphicsError, GraphicsInfo, configure_graphics, default_draw_target, find_gpu,
    inspect_context, require_stable_context,
)


class GraphicsTests(unittest.TestCase):
    def test_viewer_rejects_nv118_offscreen_before_allocating_mujoco_render_resources(self):
        try:
            import glfw
            import mujoco
            from OpenGL import GL
            from proyectorobot.simulation.viewer import RobotViewer
        except (ImportError, OSError):
            self.skipTest("Dependencias gráficas no disponibles")
        values = {GL.GL_VENDOR: b"Mesa", GL.GL_RENDERER: b"NV118", GL.GL_VERSION: b"4.3 Mesa 25.3.6"}
        with ExitStack() as stack:
            stack.enter_context(patch.dict("os.environ", {"DISPLAY": ":0", "PROYECTOROBOT_GRAPHICS": "nvidia"}))
            stack.enter_context(patch.object(glfw, "init", return_value=True))
            stack.enter_context(patch.object(glfw, "window_hint"))
            stack.enter_context(patch.object(glfw, "create_window", return_value=object()))
            stack.enter_context(patch.object(glfw, "make_context_current"))
            destroyed = stack.enter_context(patch.object(glfw, "destroy_window"))
            terminated = stack.enter_context(patch.object(glfw, "terminate"))
            stack.enter_context(patch.object(GL, "glGetString", side_effect=lambda value: values[value]))
            context = stack.enter_context(patch.object(mujoco, "MjrContext"))
            with self.assertRaisesRegex(GraphicsError, "modo no está habilitado"):
                RobotViewer(None, draw_target="offscreen").__enter__()
            context.assert_not_called()
            destroyed.assert_called_once()
            terminated.assert_called_once()

    def test_known_nv118_native_crash_is_blocked_before_normal_rendering(self):
        with self.assertRaisesRegex(GraphicsError, "--graphics intel"):
            require_stable_context(GraphicsInfo("Mesa", "NV118", "4.3 Mesa 25.3.6"))
        nvidia = GraphicsInfo("Mesa", "NV118", "4.3 Mesa 25.3.6")
        self.assertEqual(default_draw_target(nvidia), "window")
        require_stable_context(nvidia, "window")
        self.assertEqual(default_draw_target(GraphicsInfo("Intel", "Mesa Intel UHD", "4.6")), "offscreen")
        self.assertEqual(default_draw_target(GraphicsInfo("Mesa", "llvmpipe", "4.5")), "offscreen")
        require_stable_context(GraphicsInfo("Intel", "Mesa Intel UHD", "4.6"))
        require_stable_context(GraphicsInfo("Mesa", "llvmpipe", "4.5"))

    def test_auto_respects_system_and_explicit_external_preferences(self):
        environment = {"DISPLAY": ":0", "DRI_PRIME": "1", "LIBGL_ALWAYS_SOFTWARE": "1"}
        with patch.dict("os.environ", environment, clear=True):
            configure_graphics("auto")
            import os

            self.assertEqual(os.environ["DRI_PRIME"], "1")
            self.assertEqual(os.environ["LIBGL_ALWAYS_SOFTWARE"], "1")
        with patch.dict("os.environ", {}, clear=True):
            configure_graphics("auto")
            self.assertNotIn("LIBGL_ALWAYS_SOFTWARE", os.environ)
            self.assertNotIn("DRI_PRIME", os.environ)

    def test_explicit_gpu_overrides_only_process_selection_and_software_flag(self):
        with patch.dict("os.environ", {"LIBGL_ALWAYS_SOFTWARE": "1", "DISPLAY": ":0"}, clear=True):
            with patch("proyectorobot.simulation.graphics.find_gpu", return_value="pci-0000_01_00_0"):
                configure_graphics("nvidia")
            import os

            self.assertEqual(os.environ["DRI_PRIME"], "pci-0000_01_00_0")
            self.assertEqual(os.environ["LIBGL_ALWAYS_SOFTWARE"], "0")
            self.assertEqual(os.environ["DISPLAY"], ":0")
            configure_graphics("software")
            self.assertEqual(os.environ["LIBGL_ALWAYS_SOFTWARE"], "1")

    def test_pci_discovery_uses_display_class_and_vendor_without_hardcoded_bus(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            for address, vendor, device_class in (
                ("0000:00:02.0", "0x8086", "0x030000"),
                ("0000:03:00.0", "0x10de", "0x030200"),
                ("0000:01:00.0", "0x10de", "0x040300"),
            ):
                device = root / address
                device.mkdir()
                (device / "vendor").write_text(vendor)
                (device / "class").write_text(device_class)
            self.assertEqual(find_gpu("intel", root), "pci-0000_00_02_0")
            self.assertEqual(find_gpu("nvidia", root), "pci-0000_03_00_0")
        with TemporaryDirectory() as directory:
            with self.assertRaisesRegex(GraphicsError, "No se detecta"):
                find_gpu("nvidia", Path(directory))

    def test_mesa_vendor_does_not_mean_software_and_nv118_is_nvidia(self):
        for vendor, renderer, expected in (
            ("Intel", "Mesa Intel(R) UHD Graphics (CML GT2)", "intel"),
            ("Mesa", "NV118", "nvidia"),
            ("NVIDIA Corporation", "GeForce MX130", "nvidia"),
            ("Mesa/X.org", "llvmpipe (LLVM 21.1.8)", "software"),
            ("Mesa", "softpipe", "software"),
            ("Other", "Unidentified accelerator", "unknown"),
        ):
            self.assertEqual(GraphicsInfo(vendor, renderer, "4.3").device, expected)

    def test_context_is_checked_before_claiming_explicit_gpu_is_active(self):
        gl = SimpleNamespace(GL_VENDOR=1, GL_RENDERER=2, GL_VERSION=3,
                             glGetString=lambda value: {1: b"Mesa", 2: b"NV118", 3: b"4.3"}[value])
        self.assertEqual(inspect_context(gl, "nvidia").device, "nvidia")
        with self.assertRaisesRegex(GraphicsError, "Se solicitó intel"):
            inspect_context(gl, "intel")
        gl.glGetString = lambda parameter: b"llvmpipe"
        self.assertEqual(inspect_context(gl).device, "software")
        with self.assertRaises(GraphicsError):
            inspect_context(gl, "nvidia")
        gl.glGetString = lambda parameter: None
        with self.assertRaisesRegex(GraphicsError, "contexto válido"):
            inspect_context(gl)
