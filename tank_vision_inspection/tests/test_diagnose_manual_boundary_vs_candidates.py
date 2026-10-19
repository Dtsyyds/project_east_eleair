from pathlib import Path
import pytest
from scripts.diagnose_manual_boundary_vs_candidates import validate_manual_annotation

def test_manual_annotation_validation_requires_matching_hash(tmp_path: Path) -> None:
    image=tmp_path/'x.bmp'; image.write_bytes(b'image')
    data={'input_sha256':'bad','image_width':10,'image_height':10,'annotation_type':'manual','interpolation_used':False,'left_boundary_manual_points':[{'x':i,'y':i} for i in range(6)],'right_boundary_manual_points':[{'x':i+2,'y':i} for i in range(6)]}
    with pytest.raises(ValueError,match='sha256'):
        validate_manual_annotation(data,image,10,10)

def test_manual_annotation_validation_sorts_and_checks_bounds(tmp_path: Path) -> None:
    image=tmp_path/'x.bmp'; image.write_bytes(b'image')
    import hashlib
    data={'input_sha256':hashlib.sha256(b'image').hexdigest(),'image_width':10,'image_height':10,'annotation_type':'manual','interpolation_used':False,'left_boundary_manual_points':[{'x':1,'y':i} for i in range(6)],'right_boundary_manual_points':[{'x':8,'y':i} for i in range(6)]}
    assert validate_manual_annotation(data,image,10,10)[0][0]==(1.0,0.0)
    data['right_boundary_manual_points'][-1]={'x':10,'y':5}
    with pytest.raises(ValueError,match='invalid'):
        validate_manual_annotation(data,image,10,10)
