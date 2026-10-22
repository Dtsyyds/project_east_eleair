"""Explicit checkout/config/runtime locations shared by applications and tools."""
import os
from pathlib import Path


def project_root():
    configured = os.environ.get("EAST_ELEAIR_ROOT")
    if configured:
        candidate = Path(configured).expanduser().resolve()
        if not (candidate / "deploy/env.sh").is_file():
            raise RuntimeError(f"Invalid EAST_ELEAIR_ROOT: {candidate}")
        return candidate
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "deploy/env.sh").is_file() and (candidate / "config").is_dir():
            return candidate
    raise RuntimeError("Set EAST_ELEAIR_ROOT to the migrated project checkout")


def config_path(relative):
    return Path(os.environ.get("INSPECTION_CONFIG_ROOT", project_root() / "config")) / relative


def runtime_root():
    platform = os.environ.get("INSPECTION_PLATFORM", "s100p")
    root = (project_root() / ".runtime").resolve()
    selected = Path(os.environ.get("TANK_RUNTIME_ROOT", root / platform)).resolve()
    if not selected.is_relative_to(root):
        raise RuntimeError("TANK_RUNTIME_ROOT must be inside this checkout's .runtime")
    return selected


def ros_source(package):
    matches = [p.parent for p in (project_root() / "ros2_ws/src").rglob("package.xml")
               if __import__("xml.etree.ElementTree", fromlist=["parse"]).parse(p).getroot().findtext("name") == package]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one source for {package}, got {matches}")
    return matches[0]


def load_system_config():
    import yaml
    path = Path(os.environ.get("INSPECTION_SYSTEM_CONFIG", config_path("communication/system.yaml")))
    config = yaml.safe_load(path.read_text())
    if os.environ.get("FIELD_DATA_ROOT"):
        config["data_root"] = os.environ["FIELD_DATA_ROOT"]
    return config


def vision_evidence_root():
    """External frozen evidence is never copied with code; override explicitly."""
    return Path(os.environ.get("FIELD_VISION_EVIDENCE_ROOT", runtime_root() / "analysis"))
