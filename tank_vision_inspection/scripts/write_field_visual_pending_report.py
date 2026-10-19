#!/usr/bin/env python3
"""Emit explicit pending reports when preflight blocks a formal field run."""
from __future__ import annotations
import argparse, json, subprocess
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(); p.add_argument("--preflight", required=True); p.add_argument("--output", required=True); a = p.parse_args()
    out = Path(a.output); out.mkdir(parents=True, exist_ok=True)
    preflight = json.loads(Path(a.preflight).read_text(encoding="utf-8"))
    root = Path(__file__).resolve().parents[1]
    git = subprocess.run(["git", "-C", str(root), "status", "--short"], capture_output=True, text=True, check=False).stdout
    head = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, check=False).stdout.strip()
    ps = subprocess.run(["ps", "-eo", "pid,ppid,pgid,stat,cmd"], capture_output=True, text=True, check=False).stdout
    orphan = [line for line in ps.splitlines() if any(x in line for x in ("mindvision_camera", "stable_seam_node", "ros2 bag record"))]
    checks = {c["name"]: c for c in preflight.get("checks", [])}
    by_name = {c["name"]: c.get("detail", {}) for c in preflight.get("checks", [])}
    manifest = {"schema_version":"field_visual_frozen_software_v1","formal_run_started":False,
                "git":{"head":head,"status_short":git,"dirty":bool(git)},
                "files":{"tracker_node":by_name.get("tracker_node_source"),"core_tracker":by_name.get("core_tracker_source"),"config":by_name.get("config_source"),"reference":by_name.get("reference_source"),"launch":by_name.get("launch_source"),"message":by_name.get("message_source")},
                "camera":{"model":"MindVision SUF1205GC","serial":"046081820650","resolution":"4096x3072","encoding":"bgr8"},
                "runtime_parameters":{"camera_topic":"/camera/front/image_raw","camera_info_topic":"/camera/front/camera_info","exposure_mode":"auto","exposure_us":0.0,"record_raw_image":False,"record_mcap":True,"start_mid360":False,"start_sync":False,"publish_debug_image":False},
                "source_install":preflight.get("source_install",{}),"preflight":preflight,"git_status":"preserved_dirty_workspace"}
    reports = {
        "frozen_software_manifest.json": manifest,
        "topic_rate_summary.json": {"status":"pending","reason":"formal run blocked before launch","input_hz":None,"tracker_hz":None},
        "timestamp_audit.json": {"status":"pending","reason":"no MCAP was created","monotonic":None},
        "recorder_health.json": {"status":"pending","provenance":None,"writer_drop":None},
        "snapshot_integrity.json": {"status":"pending","count":0,"all_complete":None},
        "process_shutdown_audit.json": {"status":"preflight_blocked","orphan_processes":orphan,"preflight_only":True},
        "replay_summary.json": {"status":"pending","reason":"no finalized MCAP"},
        "system_acceptance.json": {"system_pass":False,"status":"blocked_before_A_60s_smoke","checks":{"data_root_write":checks.get("data_root_write",{}),"mindvision_usb":checks.get("mindvision_usb",{}),"source_install":preflight.get("source_install",{})}},
        "algorithm_observation.json": {"algorithm_pass":None,"status":"pending","reason":"algorithm was not run"},
        "next_actions.json": {"system_integration_ready":False,"algorithm_field_validation_pending":True,"production_ready":False,"actions":["在宿主机确认 /media/mcap_data 以 rw 挂载并通过 fsync 写入探针","在可访问 USB 的宿主机重新运行正式入口","正式 A 通过后再执行 B；不得据此推断算法通过"]},
    }
    summary={"status":"hardware_validation_pending","formal_run_started":False,"system_pass":False,"algorithm_pass":None,"blocking_checks":{k:v for k,v in checks.items() if not v.get("passed",False)},"source_install":preflight.get("source_install",{})}
    reports["diagnostic_summary.json"]=summary
    for name, data in reports.items(): (out/name).write_text(json.dumps(data,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
