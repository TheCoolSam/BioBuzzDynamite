"""Subprocess regression checks, including optimized Python; all outputs use temporary folders."""
import json
from contextlib import contextmanager
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

VALIDATOR = Path(__file__).resolve().parents[1]/'fusion/BioBuzzHoodTools/validate_print_meshes.py'


@contextmanager
def temporary_folder():
    root = (Path(__file__).resolve().parents[2]/'build/mesh-validator-tests').resolve()
    root.mkdir(parents=True, exist_ok=True)
    temp = tempfile.TemporaryDirectory(dir=root)
    target = Path(temp.name).resolve()
    if target.parent != root:
        raise RuntimeError('Test cleanup target escaped the intended workspace directory')
    try:
        yield str(target)
    finally:
        # Verify the actual absolute target again immediately before recursive cleanup.
        if Path(temp.name).resolve().parent != root:
            raise RuntimeError('Refusing cleanup outside the test directory')
        temp.cleanup()


def tetrahedron():
    a, b, c, d = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    return [(a, c, b), (a, b, d), (a, d, c), (b, c, d)]


def stl(path, faces):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = bytes(80)+struct.pack('<I', len(faces))
    for face in faces:
        data += struct.pack('<12fH', 0, 0, 0, *(v for p in face for v in p), 0)
    path.write_bytes(data)


class MeshValidationTests(unittest.TestCase):
    def run_validator(self, folder, success, *args):
        # Check every behavior in both interpreter modes, where assert formerly disappeared.
        for optimized in (False, True):
            with self.subTest(optimized=optimized):
                cmd = [sys.executable]+(['-O'] if optimized else [])+[str(VALIDATOR), str(folder), *args]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode == 0, success, result.stderr)
                report = json.loads((folder/'mesh_validation.json').read_text(encoding='utf-8'))
                if success:
                    self.assertIsInstance(report, list)
                    self.assertGreater(len(report), 0)
                else:
                    self.assertEqual(report['status'], 'FAILED')

    def test_empty_replaces_stale_success(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            (folder/'mesh_validation.json').write_text('[]')
            self.run_validator(folder, False)

    def test_valid_closed_shell(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            stl(folder/'part.stl', tetrahedron())
            self.run_validator(folder, True)
            report = json.loads((folder/'mesh_validation.json').read_text())
            self.assertAlmostEqual(report[0]['volume_mm3'], 1/6)

    def test_recursive_selection(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            stl(folder/'fit/part.STL', tetrahedron())
            self.run_validator(folder, False)
            self.run_validator(folder, True, '--recursive')

    def test_manifest_requires_each_file(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            stl(folder/'fit/part.stl', tetrahedron())
            manifest = folder/'manifest.json'
            manifest.write_text('["fit/part.stl"]')
            self.run_validator(folder, True, '--manifest', str(manifest))
            manifest.write_text('["fit/part.stl", "missing.stl"]')
            self.run_validator(folder, False, '--manifest', str(manifest))

    def test_manifest_empty_duplicate_and_escape(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            manifest = folder/'manifest.json'
            for content in ('[]', '["a.stl", "a.stl"]', '["../outside.stl"]', '{}'):
                with self.subTest(content=content):
                    manifest.write_text(content)
                    self.run_validator(folder, False, '--manifest', str(manifest))

    def test_truncated_empty_and_bad_length(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            for data in (b'', bytes(80)+struct.pack('<I', 0), bytes(80)+struct.pack('<I', 4)):
                with self.subTest(length=len(data)):
                    (folder/'broken.stl').write_bytes(data)
                    self.run_validator(folder, False)

    def test_open_shell(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            stl(folder/'open.stl', tetrahedron()[:-1])
            self.run_validator(folder, False)

    def test_reversed_winding(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            faces = tetrahedron()
            faces[0] = tuple(reversed(faces[0]))
            stl(folder/'wrong.stl', faces)
            self.run_validator(folder, False)

    def test_negative_volume(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            stl(folder/'inside_out.stl', [tuple(reversed(face)) for face in tetrahedron()])
            self.run_validator(folder, False)

    def test_disconnected_shells(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            offset = [tuple(tuple(v+5 for v in p) for p in face) for face in tetrahedron()]
            stl(folder/'two.stl', tetrahedron()+offset)
            self.run_validator(folder, False)

    def test_nonfinite_and_degenerate(self):
        with temporary_folder() as temp:
            folder = Path(temp)
            for face in (((float('nan'), 0, 0), (1, 0, 0), (0, 1, 0)),
                         ((0, 0, 0), (0, 0, 0), (0, 1, 0)),
                         ((0, 0, 0), (1, 0, 0), (2, 0, 0))):
                stl(folder/'degenerate.stl', [face])
                self.run_validator(folder, False)


if __name__ == '__main__':
    unittest.main()
