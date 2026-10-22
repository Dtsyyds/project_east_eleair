#!/usr/bin/env python3
"""Build system/algorithm acceptance reports from one completed field run."""
from __future__ import annotations
import argparse, hashlib, json, os, subprocess, time
from pathlib import Path
import re


def load(path: Path, default=None):
    try: return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError): return default


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1024 * 1024), b""): h.update(b)
    return h.hexdigest()


def live_summary(run: Path) -> dict:
    files = sorted(run.glob("live_session/*/live_summary.json"))
    return load(files[-1], {}) if files else {}


def snapshots(run: Path) -> dict:
    entries=[]; bad=[]
    metadata_files = list(run.glob("failure_snapshots/**/metadata.json"))
    metadata_files += list(run.glob("live_session/**/failure_snapshots/**/metadata.json"))
    for meta in sorted(set(metadata_files)):
        d=load(meta,{}) or {}; event=meta.parent; hashes=d.get("files_sha256", {})
        checked={}; ok=bool(d.get("complete", False))
        for name, expected in hashes.items():
            p=event/name; actual=sha(p) if p.is_file() else None; checked[name]={"expected": expected, "actual": actual, "match": actual == expected}
            ok = ok and actual == expected
        item={"event": str(event), "complete": bool(d.get("complete", False)), "hashes": checked, "valid": ok}
        entries.append(item)
        if not ok: bad.append(item)
    return {"count":len(entries), "valid_count":len(entries)-len(bad), "invalid_count":len(bad), "all_complete":not bad, "events":entries}


def provenance(run: Path) -> dict:
    sessions=[]
    for complete in sorted(run.glob("geometry_provenance/*/complete.json")):
        d=load(complete,{}) or {}; sessions.append({"session":str(complete.parent), **d})
    return {"sessions":sessions, "count":len(sessions), "complete":all(x.get("complete",False) for x in sessions) if sessions else True,
            "dropped_event_count":sum(int(x.get("dropped_event_count",x.get("queue_dropped",0))) for x in sessions),
            "write_error_count":sum(int(x.get("write_error_count",len(x.get("writer_errors",[])))) for x in sessions),
            "capacity_suppressed_count":sum(int(x.get("capacity_suppressed_count",0)) for x in sessions)}


def bag_info(run: Path) -> str:
    bag=run/"mcap"
    try: return subprocess.run(["ros2","bag","info",str(bag)],capture_output=True,text=True,timeout=20,check=False).stdout
    except Exception as e: return str(e)


def main() -> int:
    ap=argparse.ArgumentParser(); ap.add_argument("--run-dir",required=True); ap.add_argument("--output",required=True); a=ap.parse_args()
    run=Path(a.run_dir).resolve(); out=Path(a.output).resolve(); out.mkdir(parents=True,exist_ok=True)
    manifest=load(run/"run_manifest.json",{}) or {}; live=live_summary(run); snap=snapshots(run); prov=provenance(run)
    validation=load(run/"mcap/validation_report.json",{}) or {}
    info=bag_info(run); (out/"ros2_bag_info.txt").write_text(info,encoding="utf-8")
    frozen={"schema_version":"field_visual_frozen_software_v1","run_id":manifest.get("run_id",run.name),"manifest":manifest,"preflight":load(run/"logs/preflight.json",{}),"formal_run_started":True}
    counts={};
    for line in info.splitlines():
        match = re.search(r"Topic:\s+(\S+).*Count:\s+(\d+)", line)
        if match:
            counts[match.group(1)]={"raw":line.strip(),"count":int(match.group(2))}
    counters=live.get("counters", {})
    rates={"input_fps":counters.get("input_fps"),"tracker_fps":counters.get("tracker_fps"),"camera_fps":counters.get("camera_fps"),"common_active_duration_sec":counters.get("common_active_duration_sec"),"common_input_frames":counters.get("common_input_frames"),"common_output_frames":counters.get("common_output_frames")}
    processing=counters.get("processing_ms", {})
    topic_rate={"run_id":manifest.get("run_id",run.name),"bag_info":str(out/"ros2_bag_info.txt"),"topic_counts":counts,"validated_topics":validation.get("topics",{}),"live_rates":rates,"processing":processing}
    validated_topics=validation.get("topics",{})
    timestamp={"source":"validation_report/live_summary","monotonic":all(int(v.get("non_monotonic_count",0)) == 0 for v in validated_topics.values()),"non_monotonic_by_topic":{k:v.get("non_monotonic_count",0) for k,v in validated_topics.items()},"input_first_last":[counters.get("input_first_host_timestamp_ns"),counters.get("input_last_host_timestamp_ns")],"output_first_last":[counters.get("output_first_host_timestamp_ns"),counters.get("output_last_host_timestamp_ns")],"common_active_window":[counters.get("common_active_window_start_ns"),counters.get("common_active_window_end_ns")]}
    launch_pid = None
    try: launch_pid = int((run/"launch.pid").read_text().strip())
    except (OSError, ValueError): pass
    alive = False
    if launch_pid:
        alive = subprocess.run(["kill","-0",str(launch_pid)],capture_output=True,check=False).returncode == 0
    process_lines=[]
    process_file=run/"logs/processes_after_stop.txt"
    if process_file.is_file():
        process_lines=[line for line in process_file.read_text(errors="replace").splitlines() if any(x in line for x in ("mindvision_camera_node","stable_seam_node","ros2 bag record","ros2 launch inspection_bringup"))]
    process={"run_dir":str(run),"launch_pid":launch_pid,"orphan_processes":process_lines,"graceful_stop":bool(not alive and process_file.is_file() and not process_lines)}
    states=counters.get("state_counts",{})
    processed=int(counters.get("processed_frames",0) or 0)
    detected=int(counters.get("detected_frames",0) or 0)
    algo={"detected_rate":detected/processed if processed else None,"tracking_rate":int(states.get("TRACKING",0))/processed if processed else None,"processed_frames":processed,"detected_frames":detected,"state_counts":states,"failure_reasons":live.get("failure_reason_counts",{}),"processing_ms":processing,"status":"observational_only","algorithm_pass":False}
    finalized=(run/"mcap/manifest.yaml").is_file() or bool(manifest.get("status",{}).get("finalized",False))
    output_rate=float(rates.get("tracker_fps") or 0.0)
    system_checks={"bag_validation":bool(validation.get("valid", validation.get("ok", False))),"finalized":finalized,"snapshots_complete":snap["all_complete"],"provenance_complete":prov["complete"],"timestamp_monotonic":timestamp["monotonic"],"replay_readable":bool(counts),"common_output_rate_ge_6hz":output_rate >= 6.0}
    system_pass=all(system_checks.values())
    replay={"mode":"lightweight_mcap_readback","available":bool(counts),"raw_image_replay":False,"note":"轻量MCAP无image_raw；仅验证geometry/state/diagnostics读取与回放入口可用。"}
    files={"topic_rate_summary.json":topic_rate,"timestamp_audit.json":timestamp,"recorder_health.json":{"provenance":prov,"live_summary_present":bool(live)},"snapshot_integrity.json":snap,"process_shutdown_audit.json":process,"replay_summary.json":replay,"system_acceptance.json":{"system_pass":system_pass,"checks":system_checks,"algorithm_pass_separate":False},"algorithm_observation.json":algo,"next_actions.json":{"system_integration_ready":system_pass,"algorithm_field_validation_pending":True,"production_ready":False,"actions":["common tracker output 当前低于 6 Hz；保持算法冻结，单独做性能剖析/系统级优化","保留当前算法冻结状态，继续收集中央焊缝正样本","不要将 LOST 算法输出解释为系统录制失败"]}}
    (out/"frozen_software_manifest.json").write_text(json.dumps(frozen,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    for name,d in files.items(): (out/name).write_text(json.dumps(d,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    summary={"run_dir":str(run),"system_pass":system_pass,"algorithm_pass":False,"system_checks":system_checks,"snapshot_count":snap["count"],"provenance":prov,"input_output_rates":rates,"processing":processing}
    (out/"diagnostic_summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    if run.joinpath("run_manifest.json").is_file():
        manifest.setdefault("status",{}).update({"complete":bool(system_pass),"finalized":bool(finalized),"validation":"passed" if system_checks["bag_validation"] else "failed","valid_for_formal_validation":bool(system_pass),"system_pass":bool(system_pass),"algorithm_pass":False})
        (run/"run_manifest.json").write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print(json.dumps(summary,ensure_ascii=False))
    return 0 if system_pass else 3


if __name__ == "__main__": raise SystemExit(main())
