#!/usr/bin/env python3
"""Invoke the normal stop/finalize path when the monitor requests hard stop."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--stop-script", required=True)
    args = parser.parse_args()
    stopped = False

    def stop_signal(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    request = args.run_dir / "logs" / "hard_stop.request"
    while not stopped:
        if request.is_file():
            environment = dict(os.environ)
            environment["FIELD_STOP_FROM_GUARD"] = "1"
            return subprocess.run([args.stop_script, args.run_id], env=environment, check=False).returncode
        time.sleep(1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

