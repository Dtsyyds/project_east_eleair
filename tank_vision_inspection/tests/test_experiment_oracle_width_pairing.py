import math
from scripts.experiment_oracle_width_pairing import width_penalty

def test_lambda_zero_degenerates_to_original_score_penalty() -> None:
    assert width_penalty(10.0, 20.0, 0.0) == 0.0

def test_oracle_width_has_zero_penalty() -> None:
    assert width_penalty(20.0, 20.0, 4.0) == 0.0

def test_width_penalty_is_symmetric_in_log_space() -> None:
    assert math.isclose(width_penalty(40.0,20.0,2.0), width_penalty(10.0,20.0,2.0))

def test_invalid_width_is_finite() -> None:
    assert math.isfinite(width_penalty(0.0,20.0,4.0))
