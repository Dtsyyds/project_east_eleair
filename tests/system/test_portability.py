from inspection_runtime.paths import project_root
from pathlib import Path
import os
import subprocess
import pytest
import shutil


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
    expected_install = expected / "install"
    if not (expected_install / 'local_setup.bash').is_file() and (ROOT / 'ros2_ws/install/local_setup.bash').is_file():
        expected_install = ROOT / 'ros2_ws/install'
    assert Path(install) == expected_install


@pytest.mark.parametrize('layouts,selected', [
    (['ros2_ws/install'], 'ros2_ws/install'),
    (['.runtime/s100p/install'], '.runtime/s100p/install'),
    (['ros2_ws/install', '.runtime/s100p/install'], '.runtime/s100p/install'),
])
def test_environment_loads_selected_overlay_without_inherited_ros(tmp_path, layouts, selected):
    root = tmp_path / 'checkout'
    (root / 'deploy').mkdir(parents=True)
    shutil.copyfile(ROOT / 'deploy/env.sh', root / 'deploy/env.sh')
    for layout in layouts:
        directory = root / layout
        directory.mkdir(parents=True)
        (directory / 'local_setup.bash').write_text(f'export TEST_LOADED_OVERLAY="{layout}"\n')
    result = subprocess.run([
        'bash', '-c', 'set -eu; source "$1/deploy/env.sh" s100p; '
        'printf "%s\\n%s\\n%s\\n" "$TANK_INSTALL_ROOT" "$TEST_LOADED_OVERLAY" "$FIELD_WIRELESS_INSTALL"',
        'overlay-test', str(root)],
        env={'PATH': '/usr/bin:/bin', 'HOME': os.environ['HOME']},
        capture_output=True, text=True, check=True)
    install, loaded, wireless = result.stdout.strip().splitlines()[-3:]
    assert Path(install) == root / selected
    assert loaded == selected
    assert wireless == install
