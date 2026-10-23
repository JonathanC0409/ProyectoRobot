"""Fase 2: demo del humanoide y entrada independiente de la Fase 1."""

import argparse
import logging
from math import degrees, isfinite
from pathlib import Path
import sys
import time

from proyectorobot.app import camera_index, positive_int
from proyectorobot.config import DEFAULT_MODELS_DIR


def positive_seconds(value: str) -> float:
    seconds = float(value)
    if not isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("Se requiere un número positivo y finito.")
    return seconds


class RenderClock:
    """Conserva el ritmo de dibujo sin acumular deuda ni redondear cada intervalo."""

    def __init__(self, fps: int) -> None:
        if fps <= 0:
            raise ValueError("Los FPS de dibujo deben ser positivos.")
        self.interval = 1.0 / fps
        self.next_frame: float | None = None

    def due(self, now: float) -> bool:
        if self.next_frame is None:
            self.next_frame = now + self.interval
            return True
        if now + 1e-9 < self.next_frame:
            return False
        missed = max(0, int((now - self.next_frame) / self.interval))
        self.next_frame += (missed + 1) * self.interval
        return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m proyectorobot.simulation",
                                     description="Humanoide MuJoCo: ocho ejes e imitación 2D/3D en CPU.")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--demo", action="store_true", help="Movimientos de prueba sin webcam.")
    modes.add_argument("--check", action="store_true", help="Verifica MuJoCo y el MJCF sin ventanas.")
    modes.add_argument("--check-camera", action="store_true", help="Comprueba captura e IPC sin ventanas.")
    parser.add_argument("--headless", action="store_true", help="Demo física sin ventanas.")
    parser.add_argument("--duration", type=positive_seconds, help="Duración opcional en segundos.")
    parser.add_argument("--snapshot", type=Path, help="PNG del robot al final de una demo headless; requiere OpenGL.")
    parser.add_argument("--camera", type=camera_index, default=0)
    parser.add_argument("--width", type=positive_int, default=480)
    parser.add_argument("--height", type=positive_int, default=360)
    parser.add_argument("--fps", type=positive_int, default=30)
    parser.add_argument("--render-fps", type=positive_int, default=30,
                        help="Límite de dibujo del robot; independiente de los FPS de visión.")
    parser.add_argument("--graphics", choices=("auto", "intel", "nvidia", "software"), default="auto",
                        help="Dibujo: selección del sistema, Intel, NVIDIA o respaldo por CPU.")
    parser.add_argument("--hands", action="store_true", help="Activar dedos detallados; requiere más CPU.")
    parser.add_argument("--tracking-timeout", type=positive_seconds, default=0.35,
                        help="Tiempo máximo sin pose fresca antes del retorno seguro.")
    parser.add_argument("--window-width", type=positive_int, default=640)
    parser.add_argument("--window-height", type=positive_int, default=480)
    parser.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    parser.add_argument("--mirror", action=argparse.BooleanOptionalAction, default=True)
    return parser


def save_snapshot(simulation, path: Path) -> None:
    import cv2
    import mujoco

    camera = mujoco.MjvCamera()
    camera.lookat[:] = (0, 0, 1.05)
    camera.distance = 3.1
    camera.azimuth = 90
    camera.elevation = -8
    with mujoco.Renderer(simulation.model, height=480, width=640) as renderer:
        renderer.update_scene(simulation.data, camera=camera)
        rgb = renderer.render()
        if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise OSError(f"No se pudo escribir la vista sintética: {path}")
    logging.info("Vista sintética del robot: %s", path)


def run_demo(args: argparse.Namespace, simulation) -> None:
    from proyectorobot.simulation.demo import DEMO_DURATION, demo_joint, demo_targets
    from proyectorobot.simulation.viewer import RobotViewer

    duration = args.duration if args.duration is not None else DEMO_DURATION
    minimum = list(simulation.positions)
    maximum = minimum.copy()

    def advance(elapsed: float, now: float, viewer=None) -> None:
        simulation.controller.set_targets(demo_targets(now), now, now)
        if viewer is None:
            simulation.advance(elapsed, now)
        else:
            from proyectorobot.control.motion import JOINTS

            viewer.update(elapsed, now, f"DEMO: {JOINTS[demo_joint(now)].name}")
        for index, angle in enumerate(simulation.positions):
            minimum[index] = min(minimum[index], angle)
            maximum[index] = max(maximum[index], angle)

    if args.headless:
        for index in range(int(duration / simulation.timestep)):
            advance(simulation.timestep, index * simulation.timestep)
    else:
        with RobotViewer(simulation) as viewer:
            start = previous = time.monotonic()
            while viewer.running():
                now = time.monotonic()
                if now - start >= duration:
                    break
                advance(now - previous, now - start, viewer)
                previous = now
                time.sleep(max(0.0, 1 / 60 - (time.monotonic() - now)))
    logging.info("Recorrido observado por articulación (grados): %s",
                 [round(degrees(high - low), 1) for low, high in zip(minimum, maximum)])
    if args.snapshot:
        save_snapshot(simulation, args.snapshot)
    simulation.settle_safe()
    logging.info("Demo finalizada; postura segura (grados): %s",
                 [round(degrees(value), 1) for value in simulation.positions])


def run_imitation(args: argparse.Namespace, simulation) -> None:
    from proyectorobot.simulation.viewer import RobotViewer
    from proyectorobot.vision.worker import VisionConfig, VisionStream, VisionStreamClosed, VisionWorkerError

    config = VisionConfig(args.camera, args.width, args.height, args.fps, args.models_dir,
                          True, args.window_width, args.window_height, args.mirror, args.hands)
    captured_at = None
    inference_ms = 0.0
    vision_fps = 0.0
    simulation_fps = 0.0
    render_fps = 0.0
    status = "INICIANDO VISION"
    failure = None
    failure_deadline = None
    received_frames = 0
    tracked_frames = 0
    stop_reason = "visor cerrado"
    age = 0.0
    stream = None
    try:
        with RobotViewer(simulation) as viewer:
            with VisionStream(config) as stream:
                start = previous = time.monotonic()
                render_clock = RenderClock(args.render_fps)
                render_start = start
                render_count = 0
                generation = stream.generation
                logging.info("Calibración: hombros, codos y muñecas visibles; brazos abajo durante 1-2 s.")
                while viewer.running():
                    loop_start = time.monotonic()
                    if args.duration is not None and loop_start - start >= args.duration:
                        stop_reason = "duracion alcanzada"
                        break
                    packet = None
                    if failure is None:
                        try:
                            packet = stream.poll()
                        except VisionStreamClosed:
                            stop_reason = "ventana de camara cerrada"
                            break
                        except VisionWorkerError as error:
                            failure = error
                            failure_deadline = loop_start + 2.5
                            logging.error("%s; retornando a postura segura.", error)
                            simulation.controller.invalidate()
                    now = time.monotonic()
                    if viewer.reset.is_set():
                        viewer.reset.clear()
                        stream.request_reset()
                    if viewer.recalibrate.is_set():
                        viewer.recalibrate.clear()
                        stream.request_recalibration()
                    if generation != stream.generation:
                        generation = stream.generation
                        simulation.controller.invalidate()
                        status = ("REPOSO: C PARA CALIBRAR" if generation < 0 else "CALIBRANDO: 0%")
                    if packet is not None:
                        received_frames += 1
                        age = max(0.0, now - packet.captured_at)
                        inference_ms = packet.inference_ms
                        vision_fps = packet.fps
                        captured_at = packet.captured_at
                        if generation < 0:
                            simulation.controller.invalidate()
                        elif age <= simulation.controller.timeout and packet.generation == generation:
                            if packet.targets.left is not None or packet.targets.right is not None:
                                tracked_frames += 1
                            simulation.controller.set_targets(packet.targets, captured_at, now)
                            status = packet.status
                        else:
                            simulation.controller.invalidate()
                    age = max(0.0, now - captured_at) if captured_at is not None else 0.0
                    stale = captured_at is None or age > simulation.controller.timeout
                    if stale:
                        if captured_at is not None:
                            status = "SEGURO: DATOS ANTIGUOS"
                    if generation < 0:
                        status = "REPOSO: C PARA CALIBRAR"
                    if failure is not None:
                        status = "SEGURO: ERROR DE VISION"
                    if captured_at is None and now - start > 15 and failure is None:
                        failure = VisionWorkerError("Visión no entregó imágenes en 15 s.")
                        failure_deadline = now + 2.5
                        simulation.controller.invalidate()
                        logging.error("%s", failure)
                    sync = render_clock.due(now)
                    if now - previous >= 0.001:
                        instantaneous_sim = 1 / (now - previous)
                        simulation_fps = instantaneous_sim if simulation_fps == 0 else (
                            0.9 * simulation_fps + 0.1 * instantaneous_sim
                        )
                    viewer.update(now - previous, now,
                                  f"{status}\nVision: {vision_fps:.1f} FPS | Retardo: {age * 1000:.0f} ms\n"
                                  f"Robot: {render_fps:.1f} FPS | Control: {simulation_fps:.0f} Hz", sync=sync)
                    previous = now
                    if sync:
                        render_count += 1
                    if now - render_start >= 1:
                        render_fps = render_count / (now - render_start)
                        render_start = now
                        render_count = 0
                    if failure_deadline is not None and now >= failure_deadline:
                        stop_reason = "error de vision"
                        break
                    time.sleep(max(0.0, 1 / 60 - (time.monotonic() - loop_start)))
    finally:
        simulation.settle_safe()
        logging.info("Imitación cerrada (%s): %s frames, %s con control activo; guardas físicas: %s.",
                     stop_reason, received_frames, tracked_frames, simulation.guard_count)
        logging.info("Última inferencia %.0f ms, retardo %.0f ms, ciclo de simulación %.1f FPS.",
                     inference_ms, age * 1000, simulation_fps)
        logging.info("Robot dibujado: %.1f FPS; límite solicitado: %s FPS.", render_fps, args.render_fps)
        logging.info("Frames recibidos por IPC: %s.", getattr(stream, "received_count", 0))
        logging.info("Estado final: %s.", status)
    if failure is not None:
        raise failure
    if stop_reason == "duracion alcanzada" and received_frames == 0:
        raise VisionWorkerError("La prueba terminó sin recibir frames; revisar inicialización e IPC.")


def check_camera(args: argparse.Namespace) -> None:
    from proyectorobot.vision.worker import VisionConfig, VisionStream, VisionWorkerError

    config = VisionConfig(args.camera, args.width, args.height, args.fps, args.models_dir,
                          track_hands=args.hands)
    count = 0
    inference_ms = age_ms = 0.0
    duration = args.duration if args.duration is not None else 8.0
    with VisionStream(config) as stream:
        start = time.monotonic()
        while time.monotonic() - start < duration:
            packet = stream.poll()
            if packet is not None:
                count += 1
                inference_ms = packet.inference_ms
                age_ms = (time.monotonic() - packet.captured_at) * 1000
            time.sleep(0.01)
        logging.info("Prueba cámara: %s frames consumidos, %s recibidos por IPC, inferencia %.0f ms, retardo %.0f ms.",
                     count, stream.received_count, inference_ms, age_ms)
    if count == 0:
        raise VisionWorkerError("No se recibieron frames en la prueba aislada de cámara/IPC.")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.headless and not args.demo:
        parser.error("--headless requiere --demo.")
    if args.snapshot and not (args.demo and args.headless):
        parser.error("--snapshot requiere --demo --headless.")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    if sys.version_info[:2] != (3, 12):
        logging.error("Usa .venv/bin/python (Python 3.12).")
        return 1
    try:
        if not (args.check or args.check_camera or (args.headless and args.snapshot is None)):
            from proyectorobot.simulation.graphics import configure_graphics

            configure_graphics(args.graphics)
        from proyectorobot.control.motion import MotionController
        from proyectorobot.simulation.engine import RobotSimulation

        simulation = RobotSimulation(MotionController(timeout=args.tracking_timeout, smoothing=0.08,
                                                       adaptive=True))
        if args.check:
            import mujoco

            logging.info("MuJoCo %s; MJCF correcto: %s cuerpos, %s articulaciones, %s servos.",
                         mujoco.__version__, simulation.model.nbody, simulation.model.njnt,
                         simulation.model.nu)
        elif args.check_camera:
            check_camera(args)
        elif args.demo:
            run_demo(args, simulation)
        else:
            run_imitation(args, simulation)
    except KeyboardInterrupt:
        logging.info("Interrumpido; cerrando recursos de Fase 2.")
        return 130
    except Exception as error:
        logging.error("Fase 2 (%s): %s", type(error).__name__, error)
        logging.error("Consulta docs/PHASE2.md; prueba primero --check y --demo --headless.")
        return 1
    return 0
