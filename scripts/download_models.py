"""Descarga explícita de dos modelos oficiales; no instala paquetes ni ejecuta modelos."""

import argparse
import hashlib
from pathlib import Path
import tempfile
from urllib.error import URLError
from urllib.request import urlopen
import zipfile


MODELS = {
    "pose": (
        "pose_landmarker_lite.task",
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
        "pose_landmarker_lite/float16/1/pose_landmarker_lite.task",
    ),
    "hands": (
        "hand_landmarker.task",
        "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
        "hand_landmarker/float16/1/hand_landmarker.task",
    ),
}
DEFAULT_DIRECTORY = Path(__file__).resolve().parents[1] / "models"
MAX_MODEL_BYTES = 40 * 1024 * 1024


def download_model(filename: str, url: str, directory: Path) -> Path:
    destination = directory / filename
    if destination.exists():
        raise FileExistsError(f"Ya existe {destination}; no se sobrescribe.")
    directory.mkdir(parents=True, exist_ok=True)
    temporary = None
    digest = hashlib.sha256()
    total = 0
    try:
        with tempfile.NamedTemporaryFile(dir=directory, suffix=".part", delete=False) as output:
            temporary = Path(output.name)
            with urlopen(url, timeout=30) as response:
                while chunk := response.read(64 * 1024):
                    total += len(chunk)
                    if total > MAX_MODEL_BYTES:
                        raise ValueError("Modelo demasiado grande; descarga cancelada.")
                    output.write(chunk)
                    digest.update(chunk)
        if not zipfile.is_zipfile(temporary):
            raise ValueError("La descarga no es un bundle .task válido (ZIP esperado).")
        destination.hardlink_to(temporary)
        destination.with_suffix(".task.sha256").write_text(
            f"{digest.hexdigest()}  {filename}\n", encoding="utf-8",
        )
        print(f"Descargado: {destination} | {total} bytes | SHA256: {digest.hexdigest()}")
        return destination
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--download", action="store_true", help="Autorizar explícitamente la descarga.")
    parser.add_argument("--model", choices=("pose", "hands", "all"), default="all")
    parser.add_argument("--directory", type=Path, default=DEFAULT_DIRECTORY)
    args = parser.parse_args(argv)
    selected = MODELS if args.model == "all" else {args.model: MODELS[args.model]}
    if not args.download:
        for filename, url in selected.values():
            print(f"{filename}: {url}")
        print("Sin cambios. Ejecuta con --download solo después de autorizar la descarga.")
        return 0
    try:
        for filename, url in selected.values():
            target = args.directory / filename
            if target.is_file() and target.stat().st_size > 0:
                print(f"Conservado: {target}")
                continue
            download_model(filename, url, args.directory)
    except (OSError, URLError, ValueError) as error:
        print(f"ERROR: {error}. No se instalan paquetes; puedes reintentar la descarga.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
