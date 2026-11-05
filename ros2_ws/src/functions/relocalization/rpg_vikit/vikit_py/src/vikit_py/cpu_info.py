#!/usr/bin/env python3
from pathlib import Path
import platform

def get_cpu_info():
    source = Path("/proc/cpuinfo")
    if source.exists():
        for line in source.read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip().replace("(R)", "").replace("(TM)", "")
    return platform.processor() or platform.machine()
