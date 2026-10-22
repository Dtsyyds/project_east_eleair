from pathlib import Path

import rerun as rr

from inspection_rerun.rerun_session import RerunSession


def test_rerun_version_and_headless_save(tmp_path: Path):
    assert rr.__version__ == "0.36.2"
    output = tmp_path / "smoke.rrd"
    session = RerunSession("save", "pytest_smoke", str(output))
    session.set_time(1_700_000_000_000_000_001, 0)
    session.scalar("/test/value", 42.0)
    session.close()
    assert output.is_file()
    assert output.stat().st_size > 0
