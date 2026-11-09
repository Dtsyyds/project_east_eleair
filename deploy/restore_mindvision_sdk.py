#!/usr/bin/env python3
"""Restore the ignored native SDK from an explicitly selected local distribution."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def restore(source, destination, machine=None):
    machine = machine or os.uname().machine.lower()
    architectures = {"aarch64": ("arm64", 183), "arm64": ("arm64", 183),
                     "x86_64": ("x64", 62), "amd64": ("x64", 62)}
    if machine not in architectures:
        raise ValueError(f"Unsupported architecture: {machine}")
    arch, elf_machine = architectures[machine]
    binding = Path("demo/python_demo/mvsdk.py")
    if digest(source / binding) != digest(destination / binding):
        raise ValueError("SDK Python bindings differ; select the matching SDK 2.1.0.50 distribution")
    library = source / "lib" / arch / "libMVSDK.so"
    with library.open("rb") as stream:
        header = stream.read(20)
    if (len(header) != 20 or header[:6] != b"\x7fELF\x02\x01"
            or int.from_bytes(header[18:20], "little") != elf_machine):
        raise ValueError(f"Not a {arch} ELF64 library: {library}")
    target = destination / "lib" / arch / library.name
    checksum = digest(library)
    if target.exists():
        if digest(target) != checksum:
            raise ValueError(f"Refusing to replace a different existing SDK library: {target}")
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as temporary:
            staged = Path(temporary.name)
        try:
            shutil.copyfile(library, staged)
            staged.chmod(0o644)
            if digest(staged) != checksum:
                raise ValueError("SDK copy checksum mismatch")
            # Publish atomically without overwriting a concurrently installed library.
            os.link(staged, target)
        finally:
            staged.unlink()
    return target, checksum


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="SDK root containing demo/python_demo and lib")
    args = parser.parse_args()
    try:
        target, checksum = restore(args.source.resolve(), ROOT / "third_party/mindvision/2.1.0.50")
    except (OSError, ValueError) as error:
        parser.exit(2, f"MindVision SDK restore failed: {error}\n")
    print(f"MindVision SDK ready: {target}\nSHA256: {checksum}")


if __name__ == "__main__":
    main()
