#!/usr/bin/env bash
set -Eeuo pipefail
SYSTEM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SYSTEM/.." && pwd)"
source "$SYSTEM/scripts/env.sh"
TEMP_OUT=0
if [[ -n "${TANK_VERIFICATION_DIR:-}" ]]; then
  OUT="$TANK_VERIFICATION_DIR"
else
  OUT="$(mktemp -d /tmp/tank_verification_XXXXXX)"
  TEMP_OUT=1
fi
mkdir -p "$OUT"
LOG="$SYSTEM/reports/refactor_v1/verify_metadata_workflow.txt"
trap 'if ((TEMP_OUT)); then rm -rf -- "$OUT"; fi' EXIT
printf '\ncloseout offline verification %s\n' "$(date -Iseconds)" >>"$LOG"
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
report=Path(system)/'reports/refactor_v1/final_report.json'
d=json.loads(report.read_text());log=(Path(out)/(name+'.txt')).read_text()
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
run_check core python3 -m pytest --import-mode=importlib -q tank_vision_inspection/tests/test_async_lifecycle.py tank_vision_inspection/tests/test_async_writer.py tank_vision_inspection/tests/test_failure_event_recorder.py tank_vision_inspection/tests/test_geometry_provenance_recorder.py tank_vision_inspection/tests/test_roi_validation_recorder.py tank_vision_inspection/tests/test_stable_seam_tracker.py tank_vision_inspection/tests/test_snapshot_writer.py tank_vision_inspection/tests/test_stable_seam_ros_contract.py
run_check data python3 -m pytest -q physic_bev/src/inspection_tools/test physic_bev/tests/test_timestamp_acceptance.py physic_bev/tests/test_mapping_eddy_collection.py physic_bev/tests/test_role_handoff.py tank_inspection_system/tests/test_wrappers.py
run_check video_consumers "$FIELD_RRD_PYTHON" -m pytest -q -s tank_inspection_system/tests/test_video_consumers.py
run_check rrd_including_mcap "$FIELD_RRD_PYTHON" -m pytest -q tank_vision_inspection/ros2_ws/src/inspection_rerun/test/test_run_rrd_builder.py
run_check wireless env ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 HTTP_PROXY=http://127.0.0.1:9 HTTPS_PROXY=http://127.0.0.1:9 ALL_PROXY=http://127.0.0.1:9 python3 -m pytest -q -s physic_bev/src/system_websocket/agv_bridge/test/test_inspection_bridge.py
# Every independent check runs even with an environment blocker; final exit remains nonzero.
run_check eddy env ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 python3 "$SYSTEM/tests/ros_native_eddy_e2e.py" --output "$OUT/eddy.json"
run_check snapshot env TANK_VISION_RUN_ROS2_E2E=1 ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 python3 -m pytest --import-mode=importlib -q tank_vision_inspection/tests/test_stable_seam_snapshot_ros_e2e.py
exit "$RESULT"
