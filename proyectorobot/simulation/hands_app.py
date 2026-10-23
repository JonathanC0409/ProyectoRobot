"""Fase 2.2: manos físicas, pruebas reproducibles y control corporal opcional."""

import argparse
from contextlib import ExitStack
from dataclasses import asdict
import json
import logging
from math import sin, pi
from pathlib import Path
import time
from typing import TYPE_CHECKING

from proyectorobot.app import camera_index, positive_int
from proyectorobot.config import DEFAULT_MODELS_DIR
from proyectorobot.control.hands import FIST, OPEN, PINCH, HandCommand, HandTargets
from proyectorobot.control.motion import ArmTargets
from proyectorobot.simulation.app import RenderClock, positive_seconds
from proyectorobot.simulation.grasp_demo import (
    DURATION, GraspMonitor, command_at, run_headless, transfer_pose,
)
from proyectorobot.simulation.hand_model import task_targets

if TYPE_CHECKING:
    from proyectorobot.simulation.manipulation import ManipulationSimulation
    from proyectorobot.vision.worker import VisionPacket


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fase 2.2: manos y objetos con contacto físico.")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--grasp-demo", action="store_true", help="Agarre físico completo, 19 s.")
    modes.add_argument("--hand-demo", action="store_true", help="Prueba de seis controles, 18 s.")
    modes.add_argument("--keyboard", action="store_true", help="Modo asistido sin webcam.")
    modes.add_argument("--check", action="store_true", help="Carga el modelo sin cámara/ventanas.")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--side", choices=("left", "right"), default="left",
                        help="Lado anatómico de la mesa y del agarre de prueba.")
    parser.add_argument("--camera", type=camera_index, default=0)
    parser.add_argument("--no-hands", action="store_true", help="Desactiva inferencia de dedos.")
    parser.add_argument("--assist", action="store_true", help="Brazos por teclado, dedos por visión.")
    parser.add_argument("--mirror", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--fps", type=positive_int, default=30)
    parser.add_argument("--render-fps", type=positive_int, default=30)
    parser.add_argument("--width", type=positive_int, default=480)
    parser.add_argument("--height", type=positive_int, default=360)
    parser.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    parser.add_argument("--graphics", choices=("auto", "intel", "nvidia", "software"), default="auto")
    parser.add_argument("--duration", type=positive_seconds)
    return parser


def hand_demo_targets(elapsed: float) -> HandTargets:
    """Acciona cada control por separado; medio/anular/meñique comparten sinergia."""
    index = min(5, int(elapsed / 3))
    phase = min(1.0, (elapsed % 3) / 2)
    values = [0.0] * 6
    values[index] = sin(pi * phase) ** 2
    return HandTargets(HandCommand(*values[:3]), HandCommand(*values[3:]))


class AssistedControl:
    """Consignas explícitas del usuario; nunca cambia la pose de un objeto."""

    def __init__(self, side: str, assisted: bool = False) -> None:
        self.side = side
        self.assisted = assisted
        self.paused = False
        self.arm_targets = task_targets(65, side)
        self.overrides: dict[str, HandCommand] = {}

    def on_key(self, key: int) -> None:
        key = chr(key).upper() if 0 <= key < 256 else ""
        if key in ("1", "2"):
            self.side = "left" if key == "1" else "right"
        elif key in ("O", "F", "P"):
            self.overrides[self.side] = {"O": OPEN, "F": FIST, "P": PINCH}[key]
        elif key == "K":
            self.overrides.clear()
        elif key == "M":
            self.assisted = not self.assisted
        elif key in ("B", "L", "H", "T"):
            self.assisted = True
            self.arm_targets = (ArmTargets(**{self.side: transfer_pose(self.side)}) if key == "T"
                                else task_targets({"B": 45, "L": 75, "H": 65}[key], self.side))
        elif key == "R":
            self.paused = True
            self.overrides.clear()
        elif key == "C":
            self.paused = False
            self.overrides.clear()

    def hands(self, detected: HandTargets = HandTargets()) -> HandTargets:
        return HandTargets(*(self.overrides.get(side, getattr(detected, side))
                             for side in ("left", "right")))

    def change_generation(self, generation: int) -> None:
        """R/C desde cualquiera de las ventanas descarta órdenes manuales antiguas."""
        self.paused = generation < 0
        self.overrides.clear()


def packet_usable(packet: "VisionPacket | None", generation: int, now: float,
                  timeout: float = 0.35) -> bool:
    """El modo asistido también requiere cuerpo observado y captura reciente."""
    return (packet is not None and generation >= 0 and packet.generation == generation
            and -0.05 <= now - packet.captured_at <= timeout
            and (packet.targets.left is not None or packet.targets.right is not None))


def run_graphical(args: argparse.Namespace, simulation: "ManipulationSimulation") -> None:
    from proyectorobot.simulation.viewer import RobotViewer
    from proyectorobot.vision.worker import VisionConfig, VisionStream, VisionStreamClosed

    demo = args.grasp_demo or args.hand_demo
    live = not (demo or args.keyboard)
    monitor = GraspMonitor(simulation)
    assisted = AssistedControl(args.side, args.assist or args.keyboard)
    frames = 0
    tracked_hands = 0
    packet = None
    applied_capture = None
    generation = 0
    inference_ms = vision_fps = 0.0
    status = "INICIANDO VISION" if live else "TECLADO"
    render_count = 0
    render_fps = 0.0
    logging.info("1/2: lado | O/F/P: abrir/puño/pinza | B/L/T/H: acercar/elevar/trasladar/reposo")
    logging.info("M: brazos asistidos/cámara | K: dedos por cámara | R: parar | C: calibrar | Q: salir")
    if args.assist:
        logging.info("Modo asistido: brazos por teclado. Pulsa M en el robot para imitar brazos con la cámara.")
    with ExitStack() as stack:
        viewer = stack.enter_context(RobotViewer(simulation))
        stream = None
        if live:
            config = VisionConfig(args.camera, args.width, args.height, args.fps, args.models_dir,
                                  show_camera=True, mirror=args.mirror,
                                  track_hands=not args.no_hands, recognize_hands=not args.no_hands)
            stream = stack.enter_context(VisionStream(config))
        start = previous = report_start = time.monotonic()
        last_delivery = start
        render_clock = RenderClock(args.render_fps)
        deadline = DURATION if args.grasp_demo else 18.0 if args.hand_demo else args.duration
        try:
            while viewer.running():
                now = time.monotonic()
                elapsed = now - start
                if deadline is not None and elapsed >= deadline:
                    break
                while viewer.key_events:
                    assisted.on_key(viewer.key_events.popleft())
                if args.keyboard:
                    assisted.assisted = True
                if viewer.reset.is_set():
                    viewer.reset.clear()
                    if stream is not None:
                        stream.request_reset()
                if viewer.recalibrate.is_set():
                    viewer.recalibrate.clear()
                    if stream is not None:
                        stream.request_recalibration()
                if stream is not None:
                    try:
                        fresh = stream.poll()
                    except VisionStreamClosed:
                        break
                    if generation != stream.generation:
                        generation = stream.generation
                        assisted.change_generation(generation)
                        packet = None
                        applied_capture = None
                        last_delivery = now
                        simulation.controller.invalidate()
                        simulation.hands.invalidate()
                    if fresh is not None:
                        packet = fresh
                        last_delivery = now
                        frames += 1
                        inference_ms, vision_fps = packet.inference_ms, packet.fps
                        status = packet.status
                        tracked_hands += sum(command is not None for command in (
                            packet.gestures.targets.left, packet.gestures.targets.right))
                    if packet is None and now - last_delivery > 15:
                        raise RuntimeError("No hay resultados de visión en 15 s; comprueba cámara y modelos.")
                age = max(0.0, now - packet.captured_at) if packet is not None else float("inf")
                valid = not live or packet_usable(packet, generation, now, simulation.controller.timeout)
                if assisted.paused or not valid:
                    simulation.controller.invalidate()
                    simulation.hands.invalidate()
                    applied_capture = None
                    status = ("PARADO: C PARA CALIBRAR" if assisted.paused or generation < 0 else
                              packet.status if packet is not None and age <= simulation.controller.timeout
                              else "SEGURO: SIN VISION RECIENTE")
                elif args.grasp_demo:
                    arms, hands, status = command_at(elapsed, args.side)
                    simulation.controller.set_targets(arms, now, now)
                    simulation.hands.set_targets(hands, now, now)
                elif args.hand_demo:
                    status = "DEMO: CONTROL " + str(min(6, int(elapsed / 3) + 1))
                    simulation.hands.set_targets(hand_demo_targets(elapsed), now, now)
                else:
                    arms = assisted.arm_targets if assisted.assisted else packet.targets
                    detected = packet.gestures.targets if live else HandTargets()
                    hands = assisted.hands(detected)
                    captured_at = packet.captured_at if live else now
                    if not live or applied_capture != captured_at:
                        simulation.controller.set_targets(arms, captured_at, now)
                        simulation.hands.set_targets(hands, captured_at, now)
                        applied_capture = captured_at
                    if assisted.assisted:
                        source = ("DEDOS CAMARA" if live and not args.no_hands and not assisted.overrides
                                  else "TECLADO")
                        status = "BRAZOS ASISTIDOS | " + source
                labels = ""
                if packet is not None:
                    labels = (f"I: {packet.gestures.left} | D: {packet.gestures.right}"
                              f"\n{packet.quality_status}\n{packet.depth_status}")
                age_label = f"Edad {age * 1000:.0f} ms | " if packet is not None else ""
                side_label = "Izq" if assisted.side == "left" else "Der"
                distance = sum((float(first) - float(second)) ** 2 for first, second in zip(
                    simulation.data.site(f"{assisted.side}_grip").xpos,
                    simulation.data.body("cube_0").xpos,
                )) ** 0.5
                finger_names = {"thumb": "pulgar", "index": "indice", "middle": "medio",
                                "ring": "anular", "little": "menique", "palm": "palma"}
                contact_label = "/".join(finger_names.get(name, name) for name in sorted(
                    simulation.finger_contacts(side=assisted.side))) or "--"
                overlay = (f"{status}\n{labels}\nVision {vision_fps:.1f} | Dibujo {render_fps:.1f} FPS"
                           f" | Inf {inference_ms:.0f} ms\n{age_label}"
                           f"Lado {side_label} | O/F/P B/L/T/H M K 1/2"
                           f"\nPinza-cubo {distance * 100:.1f} cm | Contactos: {contact_label}")
                draw = render_clock.due(now)
                viewer.update(max(0.0, now - previous), now, overlay, sync=draw)
                previous = now
                render_count += int(draw)
                if args.grasp_demo:
                    monitor.observe(simulation, elapsed)
                if now - report_start >= 2:
                    render_fps = render_count / (now - report_start)
                    logging.info("Dibujo %.1f FPS | Visión %.1f FPS | Inferencia %.0f ms | %s",
                                 render_fps, vision_fps, inference_ms, status)
                    render_count = 0
                    report_start = now
                time.sleep(max(0.0, 1 / 120 - (time.monotonic() - now)))
            if args.grasp_demo and time.monotonic() - start >= DURATION:
                report = monitor.report(simulation)
                print(json.dumps(asdict(report), indent=2))
                if not report.passed:
                    raise RuntimeError(
                        "El agarre gráfico no cumplió los criterios; revisa la cadencia física."
                    )
        finally:
            simulation.controller.invalidate()
            simulation.hands.invalidate()
            logging.info("Fase 2.2 cerrada: %s observaciones; %s manos calibradas consumidas.",
                         frames, tracked_hands)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.headless and not (args.grasp_demo or args.hand_demo or args.check):
        parser.error("--headless requiere --grasp-demo, --hand-demo o --check.")
    if args.duration is not None and (args.grasp_demo or args.hand_demo):
        parser.error("Las demos tienen duración fija para completar la verificación.")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    try:
        if not (args.headless or args.check):
            from proyectorobot.simulation.graphics import configure_graphics
            configure_graphics(args.graphics)
        from proyectorobot.simulation.manipulation import ManipulationSimulation
        if args.grasp_demo and args.headless:
            report = run_headless(args.side)
            print(json.dumps({**asdict(report), "passed": report.passed}, indent=2))
            return 0 if report.passed else 1
        simulation = ManipulationSimulation(args.side)
        if args.check:
            print(f"MuJoCo: {simulation.model.njnt} articulaciones, {simulation.model.nu} servos, "
                  f"{simulation.model.ngeom} geometrías, dt={simulation.timestep}s.")
        elif args.hand_demo and args.headless:
            for step in range(round(18 / simulation.timestep)):
                now = step * simulation.timestep
                simulation.hands.set_targets(hand_demo_targets(now), now, now)
                simulation.advance(simulation.timestep, now)
            print("Prueba física de seis controles de mano completada sin webcam.")
        else:
            run_graphical(args, simulation)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        logging.error("%s: %s", type(error).__name__, error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
