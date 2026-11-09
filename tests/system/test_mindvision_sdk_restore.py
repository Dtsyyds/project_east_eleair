import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    "restore_mindvision_sdk", Path(__file__).resolve().parents[2] / "deploy/restore_mindvision_sdk.py")
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


def distribution(root, binding=b"matching binding", machine=183):
    python = root / "demo/python_demo/mvsdk.py"
    python.parent.mkdir(parents=True)
    python.write_bytes(binding)
    library = root / "lib/arm64/libMVSDK.so"
    library.parent.mkdir(parents=True)
    library.write_bytes(b"\x7fELF\x02\x01" + bytes(12) + machine.to_bytes(2, "little") + b"fixture")
    return library


def test_restore_missing_library_and_repeat(tmp_path):
    source, target = tmp_path / "source", tmp_path / "target"
    original = distribution(source)
    distribution(target).unlink()
    library, checksum = installer.restore(source, target, "aarch64")
    assert library.read_bytes() == original.read_bytes()
    assert installer.restore(source, target, "aarch64") == (library, checksum)


@pytest.mark.parametrize("mismatch", ["binding", "architecture", "existing"])
def test_restore_refuses_incompatible_or_existing_library(tmp_path, mismatch):
    source, target = tmp_path / "source", tmp_path / "target"
    distribution(source, machine=62 if mismatch == "architecture" else 183)
    library = distribution(target, binding=b"different" if mismatch == "binding" else b"matching binding")
    library.write_bytes(b"preserve this installed library")
    with pytest.raises(ValueError):
        installer.restore(source, target, "aarch64")
    assert library.read_bytes() == b"preserve this installed library"
