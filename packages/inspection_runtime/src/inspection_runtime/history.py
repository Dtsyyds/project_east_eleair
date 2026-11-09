"""Read existing run registrations without changing sealed data."""
import argparse
import json
import os
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default=os.environ.get("FIELD_DATA_ROOT", "/media/mcap_data"))
    args = parser.parse_args()
    for path in sorted((Path(args.data_root) / "runs").glob("*/run.json")):
        print(json.dumps({"path": str(path), "registration": json.loads(path.read_text())}, ensure_ascii=False))
if __name__ == "__main__":
    main()
