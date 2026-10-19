"""Read-only discovery of possible MindVision SDK installations."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import os
from pathlib import Path
import platform


@dataclass(slots=True)
class MindVisionSdkProbe:
    available: bool
    headers: list[str] = field(default_factory=list)
    libraries: list[str] = field(default_factory=list)
    machine_architecture: str = ""
    architecture_warnings: list[str] = field(default_factory=list)
    reason: str = ""

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _roots(extra_roots: list[str] | None = None) -> list[Path]:
    values = list(extra_roots or [])
    env_root = os.environ.get("MINDVISION_SDK_ROOT")
    if env_root:
        values.append(env_root)
    values.extend(("/opt", "/usr/local", "/usr"))
    result: list[Path] = []
    for value in values:
        path = Path(value)
        if path not in result and path.exists():
            result.append(path)
        if path.is_dir():
            try:
                vendor_children = [
                    child
                    for child in path.iterdir()
                    if child.is_dir() and any(token in child.name.lower() for token in ("mind", "mvs", "camera"))
                ]
            except OSError:
                vendor_children = []
            for child in vendor_children:
                if child not in result:
                    result.append(child)
    return result


def probe_mindvision_sdk(extra_roots: list[str] | None = None) -> MindVisionSdkProbe:
    headers: set[Path] = set()
    libraries: set[Path] = set()
    for root in _roots(extra_roots):
        direct_headers = (root / "include" / "CameraApi.h", root / "CameraApi.h")
        direct_libraries = (root / "lib" / "libMVSDK.so", root / "lib64" / "libMVSDK.so")
        headers.update(path for path in direct_headers if path.is_file())
        libraries.update(path for path in direct_libraries if path.is_file())
        headers.update(path for path in (root / "include").glob("*/CameraApi.h") if path.is_file())
        libraries.update(path for path in root.glob("lib*/libMVSDK.so*") if path.is_file())
        libraries.update(path for path in root.glob("lib/*/libMVSDK.so*") if path.is_file())
        if any(token in root.name.lower() for token in ("mind", "mvs", "camera")):
            headers.update(root.rglob("CameraApi.h"))
            libraries.update(root.rglob("libMVSDK.so*"))
    available = bool(headers and libraries)
    if available:
        reason = "header and shared library candidates found; official sample still requires a hardware smoke test"
    elif headers and not libraries:
        reason = "CameraApi.h found but no libMVSDK.so candidate was found"
    elif libraries and not headers:
        reason = "libMVSDK.so candidate found but CameraApi.h was not found"
    else:
        reason = "MindVision SDK not found; set MINDVISION_SDK_ROOT after installing the official SDK"
    return MindVisionSdkProbe(
        available=available,
        headers=sorted(str(path) for path in headers),
        libraries=sorted(str(path) for path in libraries),
        machine_architecture=platform.machine(),
        reason=reason,
    )
