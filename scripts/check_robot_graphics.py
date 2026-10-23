"""Comparación gráfica reproducible con MuJoCo real; sin webcam ni capturas."""

import argparse
import json
import logging
import math
from statistics import median
import subprocess
import sys
import time

from proyectorobot.simulation.graphics import GRAPHICS_MODES, configure_graphics


def measure(mode: str, frames: int, fps: int, draw_target: str = "auto") -> dict:
    if mode == "nvidia" and draw_target == "offscreen":
        logging.warning("Diagnóstico experimental: la copia offscreen NV118/nouveau falló al redimensionar.")
    configure_graphics(mode)
    from proyectorobot.simulation.engine import RobotSimulation
    from proyectorobot.simulation.viewer import RobotViewer
    from proyectorobot.simulation.demo import demo_targets

    simulation = RobotSimulation()
    draws = []
    with RobotViewer(simulation, allow_unstable_graphics=(mode == "nvidia" and draw_target == "offscreen"),
                     draw_target=draw_target) as viewer:
        for index in range(8):
            viewer.update(1 / fps, index / fps, "Preparando prueba grafica")
            viewer._gl.glFinish()
        wall_start = time.monotonic()
        cpu_start = time.process_time()
        for index in range(frames):
            if not viewer.running():
                raise RuntimeError("El visor se cerró antes de terminar la medición.")
            simulated = (index + 8) / fps
            simulation.controller.set_targets(demo_targets(simulated), simulated, simulated)
            start = time.monotonic()
            viewer.update(1 / fps, simulated, "Prueba grafica sin webcam")
            viewer._gl.glFinish()
            draws.append((time.monotonic() - start) * 1000)
            time.sleep(max(0.0, wall_start + (index + 1) / fps - time.monotonic()))
        cpu_seconds = time.process_time() - cpu_start
        wall_seconds = time.monotonic() - wall_start
        info = viewer.graphics_info
        target = viewer.draw_target
        window_system = viewer._glfw.get_version_string().decode()
    ordered = sorted(draws)
    return {
        "requested": mode, "device": info.device, "renderer": info.renderer,
        "draw_target": target, "window_system": window_system,
        "frames": frames, "fps": round(frames / wall_seconds, 2),
        "cpu_percent_one_core": round(100 * cpu_seconds / wall_seconds, 1),
        "frame_median_ms": round(median(draws), 2),
        "frame_p95_ms": round(ordered[max(0, math.ceil(len(ordered) * 0.95) - 1)], 2),
        "cpu_seconds": round(cpu_seconds, 3), "wall_seconds": round(wall_seconds, 3),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graphics", choices=(*GRAPHICS_MODES, "all"), default="auto")
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--draw-target", choices=("auto", "window", "offscreen"), default="auto")
    args = parser.parse_args(argv)
    if args.frames < 1 or not 1 <= args.fps <= 60:
        parser.error("Se requieren frames >= 1 y FPS entre 1 y 60.")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    if args.graphics == "all":
        failures = 0
        for mode in ("software", "intel", "nvidia"):
            try:
                result = subprocess.run([
                    sys.executable, "-m", "scripts.check_robot_graphics", "--graphics", mode,
                    "--frames", str(args.frames), "--fps", str(args.fps),
                    "--draw-target", args.draw_target,
                ], check=False, timeout=max(30, args.frames / args.fps + 20))
                failures += result.returncode != 0
            except subprocess.TimeoutExpired:
                logging.error("La prueba %s no terminó dentro del límite.", mode)
                failures += 1
        return 1 if failures else 0
    try:
        print(json.dumps(measure(args.graphics, args.frames, args.fps, args.draw_target), ensure_ascii=False), flush=True)
    except Exception as error:
        logging.error("Prueba gráfica %s: %s", args.graphics, error)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
