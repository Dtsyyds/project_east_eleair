import hashlib

import pytest
import yaml

from experiments.vision.scripts.frozen_inputs import verify_config


def test_config_migration_checks_original_sha_and_all_algorithm_fields(tmp_path, monkeypatch):
    frozen = tmp_path / 'baseline/tank_vision_inspection/configs/guided.yaml'
    frozen.parent.mkdir(parents=True)
    baseline = {'camera': {'sdk_root': '/frozen/sdk', 'exposure_us': 100},
                'path': {'center_transition_weight': .42}}
    frozen.write_text(yaml.safe_dump(baseline))
    digest = hashlib.sha256(frozen.read_bytes()).hexdigest()
    current = tmp_path / 'guided.yaml'
    current.write_bytes(frozen.read_bytes())
    assert verify_config(current, digest)['changed_fields'] == []
    baseline['camera']['sdk_root'] = None
    current.write_text(yaml.safe_dump(baseline))
    monkeypatch.delenv('INSPECTION_BASELINE_FIXTURES', raising=False)
    with pytest.raises(RuntimeError, match='INSPECTION_BASELINE_FIXTURES'):
        verify_config(current, digest)
    monkeypatch.setenv('INSPECTION_BASELINE_FIXTURES', str(tmp_path / 'baseline'))
    assert verify_config(current, digest)['changed_fields'] == ['camera.sdk_root']
    baseline['path']['center_transition_weight'] = .24
    current.write_text(yaml.safe_dump(baseline))
    with pytest.raises(RuntimeError, match='beyond'):
        verify_config(current, digest)
    frozen.write_text('tampered')
    with pytest.raises(RuntimeError, match='SHA mismatch'):
        verify_config(current, digest)
