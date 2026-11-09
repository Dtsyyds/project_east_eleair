"""Test-only lookup of migrated baseline fixtures and implementations."""
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/migration"))
