"""Resolve historical test fixture names using the migration ledger (test tooling only)."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
MAPPING = {row["source"]: ROOT / row["target"] for row in json.loads((ROOT / "docs/architecture/migration_files.json").read_text())["files"]}
def migrated(source):
    return MAPPING[source]
