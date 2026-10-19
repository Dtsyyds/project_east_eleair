from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import build_role_handoff as handoff  # noqa: E402


def put(path: Path, data: bytes = b"test\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


@pytest.fixture()
def fake_run(tmp_path: Path) -> Path:
    run = tmp_path / handoff.RUN_ID
    put(run / "complete.json", json.dumps({"complete": True, "finalized": True}).encode())
    put(run / "run_manifest.json", json.dumps({"run_id": handoff.RUN_ID, "ros_distro": "humble"}).encode())
    for rel in (
        "run_id.txt", "collection_status.json", "device_inventory.json", "launch_parameters.json",
        "topic_inventory.json", "field_notes.md", "frozen_configs/base.yaml",
        "reports/mcap_validation.json", "reports/timestamp_validation.json",
        "reports/source_inventory.json", "reports/system_acceptance.json",
        "reports/video_validation.json", "reports/replay_validation.json",
        "reports/topic_inventory.json", "reports/diagnostic_summary.json", "reports/stop_report.json",
        "reports/performance_matrix.json", "logs/mid360_launch.log", "logs/mid360_recorder.log",
        "logs/mid360_topics.txt", "logs/mid360_topics.record.txt", "logs/front_video.log",
        "logs/front_video_encoder.log", "logs/d405_launch.log", "logs/d405_parameter_config.log",
        "logs/d405_recorder.log", "logs/d405_topics.txt", "logs/d405_topics.record.txt",
        "logs/eddy_launch.log", "logs/eddy_recorder.log", "logs/eddy_topics.txt",
    ):
        if not (run / rel).exists():
            put(run / rel, b"{}\n" if rel.endswith(".json") else b"test\n")
    for group in ("core", "mid360", "d405", "eddy"):
        put(run / f"mcap/{group}/metadata.yaml")
        put(run / f"mcap/{group}/{group}_0.mcap")
    put(run / "video/front/front_000001.mkv")
    put(run / "video/front/front_000001.frames.jsonl")
    put(run / "video/front/front_000001.mux.log")
    put(run / "failure_snapshots/optional.json")
    put(run / "roi_validation/optional.json")
    put(run / ".partial/unfinished.mcap")
    for name in (
        f"{handoff.RUN_ID}.rrd", f"{handoff.RUN_ID}.rrd.sha256", "rrd_manifest.json",
        "rrd_validation.json", "conversion.log",
    ):
        put(run / f"rerun/{name}", b"{}\n" if name.endswith(".json") else b"test\n")
    sums = []
    for path in sorted(p for p in run.rglob("*") if p.is_file() and p.name != "sha256sums.txt"):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        sums.append(f"{digest}  {path.relative_to(run).as_posix()}")
    put(run / "sha256sums.txt", ("\n".join(sums) + "\n").encode())
    return run


@pytest.fixture()
def config() -> dict:
    return handoff.load_config(handoff.DEFAULT_CONFIG)


def entries(fake_run: Path, config: dict, profile: str):
    return handoff.build_entries(fake_run, profile, config)


def test_01_frozen_run_check_is_read_only(fake_run: Path):
    before = (fake_run / "complete.json").stat().st_mtime_ns
    handoff.ensure_frozen_run(fake_run)
    assert (fake_run / "complete.json").stat().st_mtime_ns == before


def test_02_profile_file_selection(fake_run: Path, config: dict):
    paths = {item["package_path"] for item in entries(fake_run, config, "fastlivo2")}
    assert "mcap/mid360/mid360_0.mcap" in paths


def test_03_mcap_metadata_is_selected(fake_run: Path, config: dict):
    for profile, group in (("fastlivo2", "mid360"), ("d405", "d405"), ("eddy", "eddy"), ("system_integration", "core")):
        paths = {item["package_path"] for item in entries(fake_run, config, profile)}
        assert f"mcap/{group}/metadata.yaml" in paths


def test_04_video_and_sidecars_are_selected(fake_run: Path, config: dict):
    paths = {item["package_path"] for item in entries(fake_run, config, "front_camera")}
    assert {"video/front/front_000001.mkv", "video/front/front_000001.frames.jsonl", "video/front/front_000001.mux.log"} <= paths


def test_05_profiles_do_not_mix_payloads(fake_run: Path, config: dict):
    paths = {item["package_path"] for item in entries(fake_run, config, "eddy")}
    assert not any(path.startswith("mcap/d405/") or path.startswith("video/front/") for path in paths)


def test_06_manifest_only_copies_no_large_payload(fake_run: Path, tmp_path: Path):
    out = tmp_path / "out"
    rc = handoff.main(["--run-dir", str(fake_run), "--profile", "front_camera", "--output-root", str(out), "--manifest-only"])
    assert rc == 0
    assert not list(out.rglob("*.mkv"))


def test_07_dry_run_does_not_create_destination(fake_run: Path, tmp_path: Path):
    destination = tmp_path / "absent"
    rc = handoff.main(["--run-dir", str(fake_run), "--profile", "common", "--output-root", str(tmp_path / "out"), "--materialize", "--dry-run", "--destination", str(destination)])
    assert rc == 0 and not destination.exists()


def test_08_insufficient_space_is_rejected(fake_run: Path, config: dict, tmp_path: Path):
    selected = entries(fake_run, config, "common")
    with mock.patch.object(handoff.shutil, "disk_usage", return_value=SimpleNamespace(free=0)):
        with pytest.raises(handoff.HandoffError, match="insufficient"):
            handoff.materialize(fake_run, "common", tmp_path / "dest", selected, handoff.run_facts(fake_run), resume=False, copy_method="python")


def test_09_existing_destination_is_rejected(fake_run: Path, config: dict, tmp_path: Path):
    destination = tmp_path / "dest"
    destination.mkdir()
    with pytest.raises(handoff.HandoffError, match="already exists"):
        handoff.materialize(fake_run, "common", destination, entries(fake_run, config, "common"), handoff.run_facts(fake_run), resume=False, copy_method="python")


def test_10_resume_validates_existing_file(fake_run: Path, config: dict, tmp_path: Path):
    selected = entries(fake_run, config, "common")
    first = selected[0]
    destination = tmp_path / "dest"
    target = destination / first["package_path"]
    put(target, b"wrong")
    with pytest.raises(handoff.HandoffError, match="conflict"):
        handoff.materialize(fake_run, "common", destination, selected, handoff.run_facts(fake_run), resume=True, copy_method="python")


def test_11_subset_sha_is_generated(fake_run: Path, config: dict, tmp_path: Path):
    destination = tmp_path / "dest"
    handoff.materialize(fake_run, "common", destination, entries(fake_run, config, "common"), handoff.run_facts(fake_run), resume=False, copy_method="python")
    assert (destination / "SHA256SUMS").is_file()


def test_12_subset_sha_verifies(fake_run: Path, config: dict, tmp_path: Path):
    destination = tmp_path / "dest"
    handoff.materialize(fake_run, "common", destination, entries(fake_run, config, "common"), handoff.run_facts(fake_run), resume=False, copy_method="python")
    assert handoff.verify_package(destination)["sha256_valid"]


def test_13_source_sha_is_unchanged_by_materialize(fake_run: Path, config: dict, tmp_path: Path):
    before = handoff.sha256_file(fake_run / "complete.json")
    handoff.materialize(fake_run, "common", tmp_path / "dest", entries(fake_run, config, "common"), handoff.run_facts(fake_run), resume=False, copy_method="python")
    assert handoff.sha256_file(fake_run / "complete.json") == before


def test_14_missing_custom_interface_stops_delivery(fake_run: Path, config: dict):
    altered = json.loads(json.dumps(config))
    altered["profiles"]["eddy"]["project_paths"] = ["src/does_not_exist/msg"]
    with pytest.raises(handoff.HandoffError, match="missing"):
        handoff.build_entries(fake_run, "eddy", altered)


def test_15_rerun_document_requires_0362():
    assert "0.36.2" in handoff.usage_document("rerun_viewer")


def test_16_no_install_binary_selected(fake_run: Path, config: dict):
    for profile in ("fastlivo2", "d405", "eddy"):
        assert not any("/install/" in item["source_path"] for item in entries(fake_run, config, profile))


def test_17_partial_is_never_selected(fake_run: Path, config: dict):
    for profile in handoff.ROLE_ORDER:
        assert not any(".partial" in item["package_path"] for item in entries(fake_run, config, profile))


def test_18_result_manifest_template_fields():
    value = handoff.result_manifest_template()
    assert value["source_run_id"] == handoff.RUN_ID and value["complete"] is False
    assert {"source_files", "source_sha256", "environment", "config_files", "warnings"} <= value.keys()


def test_19_documented_limitations_are_complete(fake_run: Path):
    text = handoff.known_limitations_text(handoff.run_facts(fake_run))
    for needle in ("Enabled writers", "Formal replay evidence", "unverified", "waveform", "ROS replay counts", "acceptance"):
        assert needle in text


def test_20_materialized_package_has_no_symlinks(fake_run: Path, config: dict, tmp_path: Path):
    destination = tmp_path / "dest"
    handoff.materialize(fake_run, "common", destination, entries(fake_run, config, "common"), handoff.run_facts(fake_run), resume=False, copy_method="python")
    assert not any(path.is_symlink() for path in destination.rglob("*"))


def test_handoff_accepts_new_run_identity_and_manifest_rrd(tmp_path, config):
    run=tmp_path / "NEW-RUN"
    put(run / "complete.json", json.dumps({"complete": True, "finalized": True}).encode())
    put(run / "run_manifest.json", json.dumps({"run_id": "NEW-RUN"}).encode())
    put(run / "rerun/rrd_manifest.json", json.dumps({"output_rrd": str(run / "rerun/custom.rrd")}).encode())
    handoff.ensure_frozen_run(run)
    assert handoff.rrd_relative_path(run) == "rerun/custom.rrd"
    assert handoff.resolve_role_path(run, "{rrd_path}.sha256") == "rerun/custom.rrd.sha256"
    assert handoff.result_manifest_template("NEW-RUN")["source_run_id"] == "NEW-RUN"
    assert handoff.vision_schema("NEW-RUN")["properties"]["run_id"]["const"] == "NEW-RUN"
    assert "custom.rrd" in handoff.usage_document("rerun_viewer", "custom.rrd")


def test_external_analysis_rrd_handoff_is_read_only(fake_run, config, tmp_path):
    out=fake_run.parent.parent/'analysis'/fake_run.name/'analysis-fixture/outputs'
    out.mkdir(parents=True)
    rrd=out/'arbitrary-current-name.rrd';rrd.write_bytes(b'verified derived fixture')
    digest=handoff.sha256_file(rrd)
    document={'run_id':fake_run.name,'complete':True,'hash_verified':True,
              'valid_for_formal_validation':True,'output_path':str(rrd),'rrd_sha256':digest}
    manifest=out/'rrd_manifest.json';manifest.write_text(json.dumps(document))
    (out/'rrd_validation.json').write_text('{"success":true}')
    (out/'conversion.log').write_text('fixture')
    Path(str(rrd)+'.sha256').write_text(digest+'  '+rrd.name+'\n')
    before={str(p):handoff.sha256_file(p) for p in fake_run.rglob('*') if p.is_file()}
    selected=handoff.build_entries(fake_run,'rerun_viewer',config,rrd_manifest_path=manifest)
    assert {p['package_path'] for p in selected}=={rrd.name,rrd.name+'.sha256','rrd_manifest.json','rrd_validation.json','conversion.log'}
    assert all(p['origin']=='analysis_rrd' for p in selected)
    facts=handoff.run_facts(fake_run,manifest)
    target=tmp_path/'derived_delivery'
    handoff.materialize(fake_run,'rerun_viewer',target,selected,facts,resume=False,copy_method='python')
    assert handoff.verify_package(target)['sha256_valid']
    assert before=={str(p):handoff.sha256_file(p) for p in fake_run.rglob('*') if p.is_file()}
    rrd.write_bytes(b'tampered')
    with pytest.raises(handoff.HandoffError,match='SHA mismatch'):
        handoff.build_entries(fake_run,'rerun_viewer',config,rrd_manifest_path=manifest)
