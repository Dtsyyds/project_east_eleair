from pathlib import Path
import os
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SYSTEM = ROOT / "tank_inspection_system"


def test_runtime_sources_do_not_depend_on_checkout_absolute_path():
    forbidden = ("/home/sunrise/east_eleair", "/media/mcap_data/east_eleair_plus")
    sources = [
        SYSTEM / "scripts" / "env.sh",
        SYSTEM / "scripts" / "build.sh",
        SYSTEM / "scripts" / "install_core.py",
        ROOT / "physic_bev/src/mindvision_camera_driver/launch/mindvision_camera.launch.py",
        ROOT / "physic_bev/src/mindvision_camera_driver/config/mindvision_suf1205gc_s100p.yaml",
        ROOT / "physic_bev/src/inspection_bringup/launch/hardware_pipeline.launch.py",
    ]
    for source in sources:
        text = source.read_text()
        assert all(value not in text for value in forbidden), source


def test_environment_derives_roots_from_script_location(tmp_path):
    runtime = tmp_path / "runtime"
    result = subprocess.run(
        [
            "bash",
            "-c",
            'set -eu; source "$1"; printf "%s\\n%s\\n%s\\n" '
            '"$EAST_ELEAIR_ROOT" "$TANK_RUNTIME_ROOT" "$TANK_INSTALL_ROOT"',
            "portability-test",
            str(SYSTEM / "scripts/env.sh"),
        ],
        env={**os.environ, "TANK_RUNTIME_ROOT": str(runtime)},
        check=True,
        capture_output=True,
        text=True,
    )
    root, actual_runtime, install = result.stdout.strip().splitlines()[-3:]
    assert Path(root) == ROOT
    assert Path(actual_runtime) == runtime
    assert Path(install) == runtime / "install"
