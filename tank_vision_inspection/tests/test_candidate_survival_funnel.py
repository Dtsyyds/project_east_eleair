import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/audit_candidate_survival_funnel.py"
SPEC = importlib.util.spec_from_file_location("candidate_survival_funnel", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_non_equal_scale_coordinate_roundtrip() -> None:
    sx, sy = 0.7290480065, 0.7286821705
    original = (573.3333333, 898.6999999)
    processed = MODULE.roi_to_processed(*original, sx, sy)
    recovered = MODULE.processed_to_roi(*processed, sx, sy)
    assert abs(recovered[0] - original[0]) <= 0.5
    assert abs(recovered[1] - original[1]) <= 0.5
    assert sx != sy


def test_first_loss_categories_are_mutually_exclusive() -> None:
    categories = set()
    for kwargs in (
        dict(manual_inside=False, best_expected_response=1, threshold=1, raw_near=False, threshold_near=False,
             nms_near=False, topk_near=False, final_near=False, anchor_valid=False, anchor_near=False),
        dict(manual_inside=True, best_expected_response=0, threshold=1, raw_near=False, threshold_near=False,
             nms_near=False, topk_near=False, final_near=False, anchor_valid=False, anchor_near=False),
        dict(manual_inside=True, best_expected_response=2, threshold=1, raw_near=True, threshold_near=False,
             nms_near=False, topk_near=False, final_near=False, anchor_valid=False, anchor_near=False),
        dict(manual_inside=True, best_expected_response=2, threshold=1, raw_near=True, threshold_near=True,
             nms_near=False, topk_near=False, final_near=False, anchor_valid=False, anchor_near=False),
        dict(manual_inside=True, best_expected_response=2, threshold=1, raw_near=True, threshold_near=True,
             nms_near=True, topk_near=False, final_near=False, anchor_valid=False, anchor_near=False),
        dict(manual_inside=True, best_expected_response=2, threshold=1, raw_near=True, threshold_near=True,
             nms_near=True, topk_near=True, final_near=False, anchor_valid=True, anchor_near=False),
        dict(manual_inside=True, best_expected_response=2, threshold=1, raw_near=True, threshold_near=True,
             nms_near=True, topk_near=True, final_near=True, anchor_valid=True, anchor_near=True),
    ):
        categories.add(MODULE.first_loss_category(**kwargs))
    assert categories == {
        "manual_outside_guided_window", "expected_polarity_response_missing", "response_below_threshold",
        "removed_by_nms", "removed_by_side_topk", "anchor_merge_not_retained", "candidate_survived",
    }


def test_pair_funnel_categories() -> None:
    assert MODULE.pair_category(left_exists=False, right_exists=True, pair=None, pair_after=set(), pair_post=set()) == "one_side_candidate_missing"
    pair = {"pair_id": "p", "hard_gate_passed": False, "hard_gate_rejection_reason": "width_below_minimum"}
    assert MODULE.pair_category(left_exists=True, right_exists=True, pair=pair, pair_after=set(), pair_post=set()) == "width_hard_gate_rejected"
    pair = {"pair_id": "p", "hard_gate_passed": True}
    assert MODULE.pair_category(left_exists=True, right_exists=True, pair=pair, pair_after={"p"}, pair_post=set()) == "removed_by_pair_topk"
    assert MODULE.pair_category(left_exists=True, right_exists=True, pair=pair, pair_after={"p"}, pair_post={"p"}) == "entered_dp"
