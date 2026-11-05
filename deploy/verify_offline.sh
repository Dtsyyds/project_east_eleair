#!/usr/bin/env bash
set -Eeuo pipefail
if [[ "${1:-}" == --help ]]; then echo 'Usage: deploy/verify_offline.sh; run after the next clean build (ROS mock tests, no physical device startup)'; exit 0; fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/deploy/env.sh"
SYSTEM="$ROOT"
OUT="${TANK_VERIFICATION_DIR:-$TANK_RUNTIME_ROOT/validation/offline_$(date +%Y%m%dT%H%M%S)}"
mkdir -p "$OUT"
LOG="$OUT/verify_metadata_workflow.txt"
export ROS_LOG_DIR="$OUT/ros_logs"
cd "$ROOT"
RESULT=0
run_check() {
  local name="$1"; shift
  local rc=0
  "$@" >"$OUT/$name.txt" 2>&1 || rc=$?
  printf '%s\t%s\n' "$name" "$rc" >>"$OUT/exit_codes.tsv"
  printf '\n%s: exit %s\ncommand:' "$name" "$rc" >>"$LOG"
  printf ' %q' "$@" >>"$LOG"
  printf '\n' >>"$LOG"
  cat "$OUT/$name.txt" >>"$LOG"
  python3 - "$SYSTEM" "$OUT" "$name" "$rc" "$@" <<'RECORD'
from pathlib import Path
import json,re,sys,datetime
system,out,name,rc,*argv=sys.argv[1:]
report=Path(out)/'verification.json'
d=json.loads(report.read_text()) if report.exists() else {};log=(Path(out)/(name+'.txt')).read_text()
result={'status':'verified' if int(rc)==0 else 'failed','command':argv,'exit_code':int(rc),'evidence':'verify_metadata_workflow.txt','completed_at':datetime.datetime.now(datetime.timezone.utc).isoformat()}
summary=re.findall(r'[^\n]*(?:\d+ passed|\d+ failed)[^\n]*',log)
if summary:result['pytest_summary']=summary[-1]
if name=='eddy' and (Path(out)/'eddy.json').exists():result['fixture_result']=json.loads((Path(out)/'eddy.json').read_text())
if name=='wireless':
    for line in log.splitlines():
        if '{"wireless_preview_load"' in line:
            try:result['preview_load']=json.loads(line[line.index('{"wireless_preview_load"'):])['wireless_preview_load']
            except ValueError:pass
checks=d.setdefault('closeout',{}).setdefault('checks',{})
checks[name]=result
report.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
RECORD
  printf '%s: exit %s; evidence %s\n' "$name" "$rc" "$LOG"
  if ((rc)); then RESULT=1; fi
}
run_check core python3 -m pytest --import-mode=importlib -q packages/tank_vision/tests/test_async_lifecycle.py packages/tank_vision/tests/test_async_writer.py packages/tank_vision/tests/test_failure_event_recorder.py packages/tank_vision/tests/test_geometry_provenance_recorder.py packages/tank_vision/tests/test_roi_validation_recorder.py packages/tank_vision/tests/test_stable_seam_tracker.py packages/tank_vision/tests/test_snapshot_writer.py packages/tank_vision/tests/test_stable_seam_ros_contract.py
run_check data python3 -m pytest -q ros2_ws/src/functions/data/inspection_tools/test tests/data/test_timestamp_acceptance.py tests/data/test_mapping_eddy_collection.py tests/data/test_role_handoff.py tests/system/test_wrappers.py
# These tests need no pytest plugins. Keep ROS launch_testing's auto-loaded
# plugin (and its system-only Python dependencies) out of the Rerun venv.
run_check video_consumers env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$FIELD_RRD_PYTHON" -m pytest -q -s tests/system/test_video_consumers.py
run_check rrd_including_mcap env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$FIELD_RRD_PYTHON" -m pytest -q ros2_ws/src/functions/visualization/inspection_rerun/test/test_run_rrd_builder.py
run_check wireless env ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 HTTP_PROXY=http://127.0.0.1:9 HTTPS_PROXY=http://127.0.0.1:9 ALL_PROXY=http://127.0.0.1:9 python3 -m pytest -q -s ros2_ws/src/functions/communication/agv_bridge/test/test_inspection_bridge.py
# Every independent check runs even with an environment blocker; final exit remains nonzero.
run_check eddy env ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 python3 "$ROOT/tests/system/ros_native_eddy_e2e.py" --output "$OUT/eddy.json"
run_check snapshot env TANK_VISION_RUN_ROS2_E2E=1 ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 python3 -m pytest --import-mode=importlib -q packages/tank_vision/tests/test_stable_seam_snapshot_ros_e2e.py
exit "$RESULT"
