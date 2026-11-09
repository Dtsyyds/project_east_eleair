from inspection_runtime.paths import project_root
from pathlib import Path
import os
import subprocess


ROOT = project_root()
SYSTEM = project_root() / 'deploy'


def test_runtime_sources_do_not_depend_on_checkout_absolute_path():
    forbidden = ("/home/sunrise/east_eleair", "/media/mcap_data/east_eleair_plus")
    sources = [
        project_root() / 'deploy/compat/system/env.sh',
        project_root() / 'deploy/compat/system/build.sh',
        project_root() / 'tools/system/install_core.py',
        project_root() / 'ros2_ws/src/drivers/mindvision_camera_driver/launch/mindvision_camera.launch.py',
        project_root() / 'ros2_ws/src/drivers/mindvision_camera_driver/config/mindvision_suf1205gc_s100p.yaml',
        project_root() / 'ros2_ws/src/bringup/inspection_bringup/launch/hardware_pipeline.launch.py',
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
            str(project_root() / 'deploy/compat/system/env.sh'),
        ],
        env={**os.environ, "TANK_RUNTIME_ROOT": str(runtime)},
        check=True,
        capture_output=True,
        text=True,
    )
    root, actual_runtime, install = result.stdout.strip().splitlines()[-3:]
    assert Path(root) == ROOT
    expected = ROOT / ".runtime" / os.environ.get("INSPECTION_PLATFORM", "s100p")
    assert Path(actual_runtime) == expected
    assert Path(install) == expected / "install"
