"""Single dispatch used by the CLI and deployment wrappers."""
import os
import sys
from .paths import project_root

COMMANDS = {
    "run": ("module", "inspection_runtime.supervisor"),
    "preflight": ("module", "inspection_runtime.preflight"),
    "collect": ("script", "start_vehicle_collection.sh"),
    "stop": ("script", "stop_vehicle_collection.sh"),
    "status": ("script", "status_vehicle_collection.sh"),
    "replay": ("script", "replay_vehicle_collection.sh"),
    "validate": ("module", "inspection_runtime.data.validate_vehicle_collection"),
    "handoff": ("module", "inspection_runtime.data.build_role_handoff"),
    "verify-handoff": ("module", "inspection_runtime.data.verify_role_handoff"),
    "history": ("module", "inspection_runtime.history"),
}

def dispatch(command, arguments):
    kind, target = COMMANDS[command]
    argv = [sys.executable, "-m", target] if kind == "module" else ["bash", str(project_root() / "packages/inspection_runtime/scripts" / target)]
    os.execvp(argv[0], argv + list(arguments))
