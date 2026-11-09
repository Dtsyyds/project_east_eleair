#!/usr/bin/env python3
"""Shared, dependency-light helpers for FIELD-W5 vehicle collection tools."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Any


RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"expected boolean, got {value!r}")


def validate_run_id(value: str) -> str:
    if not RUN_ID_RE.fullmatch(value):
        raise ValueError(
            "run_id must start with an alphanumeric and contain only "
            "alphanumerics, dot, underscore, or hyphen (maximum 128 characters)"
        )
    return value


def atomic_write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.partial")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(value)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def atomic_json(path: Path, value: Any) -> None:
    atomic_write_text(path, json.dumps(value, indent=2, ensure_ascii=False) + "\n")


def load_json(path: Path, default: Any = None) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def command(args: list[str], timeout: float = 15.0, cwd: Path | None = None) -> dict[str, Any]:
    try:
        result = subprocess.run(
            args, cwd=str(cwd) if cwd else None, text=True, capture_output=True,
            timeout=timeout, check=False,
        )
        return {
            "argv": args, "returncode": result.returncode,
            "stdout": result.stdout.strip(), "stderr": result.stderr.strip(),
        }
    except (OSError, subprocess.SubprocessError) as exc:
        return {"argv": args, "returncode": None, "stdout": "", "stderr": str(exc)}


def directory_bytes(path: Path) -> int:
    total = 0
    if not path.exists():
        return total
    for item in path.rglob("*"):
        try:
            if item.is_file():
                total += item.stat().st_size
        except OSError:
            pass
    return total


def process_start_ticks(pid: int) -> int | None:
    try:
        # comm is enclosed in parentheses and may contain spaces. The starttime
        # field is number 22, or index 19 after the closing parenthesis.
        tail = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return int(tail[19])
    except (OSError, ValueError, IndexError):
        return None


def process_cmdline(pid: int) -> str | None:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(
            "utf-8", errors="replace"
        ).strip()
    except OSError:
        return None


def process_identity(pid: int, name: str) -> dict[str, Any]:
    return {
        "name": name,
        "pid": int(pid),
        "pgid": os.getpgid(pid),
        "start_ticks": process_start_ticks(pid),
        "cmdline": process_cmdline(pid),
        "registered_at": utc_now(),
    }


def process_matches(item: dict[str, Any]) -> bool:
    try:
        pid = int(item["pid"])
    except (KeyError, TypeError, ValueError):
        return False
    return (
        process_start_ticks(pid) is not None
        and process_start_ticks(pid) == item.get("start_ticks")
    )


def parse_topic_list(text: str) -> dict[str, str]:
    topics: dict[str, str] = {}
    for line in text.splitlines():
        match = re.match(r"^(\S+)\s+\[([^,\]]+)", line.strip())
        if match:
            topics[match.group(1)] = match.group(2).strip()
    return topics


def discover_topics() -> dict[str, str]:
    result = command(["ros2", "topic", "list", "-t"], timeout=15.0)
    return parse_topic_list(result["stdout"]) if result["returncode"] == 0 else {}


def topic_verbose(topic: str) -> dict[str, Any]:
    return command(["ros2", "topic", "info", topic, "--verbose"], timeout=10.0)


def update_manifest(run_dir: Path, **values: Any) -> dict[str, Any]:
    path = run_dir / "run_manifest.json"
    document = load_json(path, {}) or {}
    for key, value in values.items():
        if key == "status" and isinstance(value, dict):
            document.setdefault("status", {}).update(value)
        else:
            document[key] = value
    atomic_json(path, document)
    return document

