#!/usr/bin/env python3
"""FIELD-W6 entrypoint reusing the shared W5 validator."""
from inspection_runtime.data.validate_vehicle_collection import main
if __name__ == "__main__":
    raise SystemExit(main())
