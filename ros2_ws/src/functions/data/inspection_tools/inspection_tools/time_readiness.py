#!/usr/bin/env python3
"""Report whether Unix wall time is trustworthy enough for formal records."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import time
from typing import Any


def _run(command: list[str]) -> dict[str, Any]:
    try:
        result = subprocess.run(command, text=True, capture_output=True, timeout=3, check=False)
        return {"available": True, "returncode": result.returncode,
                "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return {"available": False, "returncode": None, "stdout": "", "stderr": str(exc)}


def evaluate_time_readiness(
    *, system_time_ns: int, ntp_synchronized: bool, min_year: int = 2024,
    rtc_time_ns: int | None = None,
) -> dict[str, Any]:
    system_year = datetime.fromtimestamp(system_time_ns / 1e9, tz=timezone.utc).year
    rtc_year = (
        datetime.fromtimestamp(rtc_time_ns / 1e9, tz=timezone.utc).year
        if rtc_time_ns is not None and rtc_time_ns > 0 else None
    )
    plausible_epoch = system_year >= min_year
    ready = plausible_epoch and ntp_synchronized
    return {
        "status": "TIME_READY" if ready else "TIME_NOT_READY",
        "ready": ready,
        "system_year": system_year,
        "minimum_year": min_year,
        "plausible_unix_epoch": plausible_epoch,
        "ntp_synchronized": ntp_synchronized,
        "rtc_year": rtc_year,
        "rtc_valid": rtc_year is not None and rtc_year >= min_year,
        "policy": "formal records require plausible Unix epoch and confirmed network synchronization",
    }


def inspect(min_year: int) -> dict[str, Any]:
    timedate = _run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"])
    ntp_synced = timedate["returncode"] == 0 and timedate["stdout"].strip().lower() == "yes"
    hwclock = _run(["hwclock", "--show"])
    rtc_ns = None
    if hwclock["returncode"] == 0:
        try:
            rtc_ns = int(datetime.fromisoformat(hwclock["stdout"].replace(" ", "T", 1)).timestamp() * 1e9)
        except ValueError:
            rtc_ns = None
    report = evaluate_time_readiness(
        system_time_ns=time.time_ns(), ntp_synchronized=ntp_synced,
        min_year=min_year, rtc_time_ns=rtc_ns)
    report.update({
        "schema_version": "tank_time_readiness_v1",
        "checked_system_timestamp_ns": time.time_ns(),
        "checked_steady_timestamp_ns": time.monotonic_ns(),
        "timezone": datetime.now().astimezone().tzname(),
        "timedatectl": timedate,
        "chronyc": _run(["chronyc", "tracking"]) if shutil.which("chronyc") else {
            "available": False, "returncode": None, "stdout": "", "stderr": "chronyc not installed"
        },
        "hwclock": hwclock,
        "systemd_timesync_marker": Path("/run/systemd/timesync/synchronized").exists(),
        "basic_device_self_test_allowed": True,
        "formal_recording_allowed": report["ready"],
        "action": "do not change system time; wait for configured synchronization service",
    })
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output")
    parser.add_argument("--minimum-year", type=int, default=2024)
    parser.add_argument("--require-ready", action="store_true")
    args = parser.parse_args(argv)
    report = inspect(args.minimum_year)
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        output = Path(args.output).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report["ready"] or not args.require_ready else 3


if __name__ == "__main__":
    raise SystemExit(main())
