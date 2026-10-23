"""La preparación de modelos debe ser explícita y preservar archivos existentes."""

from contextlib import redirect_stdout
import io
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from urllib.error import URLError
import zipfile

from scripts.download_models import download_model, main


def bundle_response() -> io.BytesIO:
    response = io.BytesIO()
    with zipfile.ZipFile(response, "w") as archive:
        archive.writestr("test.tflite", b"synthetic test data")
    response.seek(0)
    return response


class DownloadTests(unittest.TestCase):
    def test_default_is_read_only(self) -> None:
        with patch("scripts.download_models.urlopen") as network, redirect_stdout(io.StringIO()):
            self.assertEqual(main([]), 0)
        network.assert_not_called()

    def test_valid_bundle_and_digest_are_published_after_download(self) -> None:
        with TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            with patch("scripts.download_models.urlopen", return_value=bundle_response()):
                target = download_model("test.task", "https://example.test/model", Path(directory))
            self.assertTrue(zipfile.is_zipfile(target))
            self.assertTrue(target.with_suffix(".task.sha256").is_file())
            self.assertEqual(list(Path(directory).glob("*.part")), [])

    def test_invalid_response_leaves_no_model_or_partial_file(self) -> None:
        with TemporaryDirectory() as directory:
            with patch("scripts.download_models.urlopen", return_value=io.BytesIO(b"error html")):
                with self.assertRaisesRegex(ValueError, "bundle"):
                    download_model("test.task", "https://example.test/model", Path(directory))
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_network_failure_removes_partial_file(self) -> None:
        with TemporaryDirectory() as directory:
            with patch("scripts.download_models.urlopen", side_effect=URLError("offline")):
                with self.assertRaises(URLError):
                    download_model("test.task", "https://example.test/model", Path(directory))
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_existing_file_is_never_overwritten(self) -> None:
        with TemporaryDirectory() as directory:
            destination = Path(directory) / "test.task"
            destination.write_bytes(b"original")
            with patch("scripts.download_models.urlopen") as network:
                with self.assertRaises(FileExistsError):
                    download_model("test.task", "https://example.test/model", Path(directory))
            self.assertEqual(destination.read_bytes(), b"original")
            network.assert_not_called()

    def test_oversized_download_is_not_published(self) -> None:
        with TemporaryDirectory() as directory:
            with patch("scripts.download_models.MAX_MODEL_BYTES", 2):
                with patch("scripts.download_models.urlopen", return_value=bundle_response()):
                    with self.assertRaisesRegex(ValueError, "grande"):
                        download_model("test.task", "https://example.test/model", Path(directory))
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_file_created_during_download_is_preserved(self) -> None:
        with TemporaryDirectory() as directory:
            destination = Path(directory) / "test.task"

            def concurrent_response(*args, **kwargs) -> io.BytesIO:
                destination.write_bytes(b"concurrent original")
                return bundle_response()

            with patch("scripts.download_models.urlopen", side_effect=concurrent_response):
                with self.assertRaises(FileExistsError):
                    download_model("test.task", "https://example.test/model", Path(directory))
            self.assertEqual(destination.read_bytes(), b"concurrent original")
            self.assertEqual(list(Path(directory).glob("*.part")), [])
