"""Persist asynchronous evidence status outside the algorithm hot path."""
import json
import os

def write_status(path, status):
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(status, indent=2) + "\n")
    os.replace(temporary, path)
