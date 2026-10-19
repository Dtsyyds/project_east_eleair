#!/usr/bin/env python3
"""Controlled offline response-threshold ablation."""
from __future__ import annotations
import argparse, csv, hashlib, json, math, resource, sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import audit_candidate_survival_funnel as funnel
from tank_vision.seam_width_reference import SeamWidthReference
from tank_vision.stable_seam_tracker import StableSeamTracker, load_stable_config

SESSION = Path("/media/mcap_data/runs/FIELD-W2-A01-GEOMETRY-PROVENANCE-20260917_191452/geometry_provenance/20260917_191453")
ANNOTATIONS = ROOT / "output/field_w2_a01_geometry_provenance/manual_annotations"
CONFIG = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
EVENTS = (
    "event_1789643716320898790_000003",
    "event_1789643738325698790_000005",
    "event_1789643705995198790_000002",
)
SCALES = (1.0, 0.9, 0.8, 0.7, 0.6)
TOLERANCES = (15.0, 20.0, 25.0, 40.0)

def put(path: Path, obj: Any) -> None:
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")

def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

def stat(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "min": None, "mean": None, "p50": None, "p90": None, "p95": None, "max": None}
    a = np.asarray(values, dtype=np.float64)
    return {"count": int(a.size), "min": float(a.min()), "mean": float(a.mean()),
            "p50": float(np.percentile(a, 50)), "p90": float(np.percentile(a, 90)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}

def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))

def read_event(path: Path) -> tuple[np.ndarray, dict[str, Any], dict[str, Any], np.ndarray]:
    meta, old = load(path / "metadata.json"), load(path / "provenance.json")
    roi = cv2.imread(str(path / "roi_raw.bmp"), cv2.IMREAD_COLOR)
    if roi is None:
        raise RuntimeError(f"cannot read {path / 'roi_raw.bmp'}")
    x, y, w, h = map(int, meta["roi_original_px"])
    if roi.shape[:2] != (h, w):
        raise RuntimeError(f"ROI shape mismatch: {roi.shape}")
    full = np.zeros((3072, 4096, 3), np.uint8)
    full[y:y+h, x:x+w] = roi
    return full, meta, old, roi

def annotation_x(annotation: dict[str, Any], side: str, y_roi: float, roi_x: float) -> float:
    return roi_x + funnel.interpolate_manual(annotation, side, y_roi)

def effective_thresholds(roi: np.ndarray, config: dict[str, Any], ys: np.ndarray) -> np.ndarray:
    p = config["processing"]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if roi.ndim == 3 else roi.astype(np.uint8)
    scale = min(1.0, float(p["max_roi_width"]) / gray.shape[1])
    size = (max(1, round(gray.shape[1] * scale)), max(1, round(gray.shape[0] * scale)))
    gray = cv2.resize(gray, size, interpolation=cv2.INTER_AREA) if size != (gray.shape[1], gray.shape[0]) else gray.copy()
    gray = cv2.GaussianBlur(gray, (int(p["gaussian_kernel"]), int(p["gaussian_kernel"])), 0)
    response = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=int(p["sobel_kernel"]))
    half = int(p["scan_band_half_height_px"])
    profiles = cv2.blur(response, (1, 2 * half + 1))
    profiles = cv2.GaussianBlur(profiles, (int(p["profile_gaussian_kernel"]), 1), 0)[ys]
    return np.maximum(float(p["edge_min_response"]),
                      np.percentile(np.abs(profiles), float(p["edge_percentile"]), axis=1))

def stages(band: dict[str, Any], side: str, threshold: float) -> dict[str, list[dict[str, Any]]]:
    raw = list(band.get("all_local_signed_edge_peaks", {}).get(side, []))
    thresholded = [v for v in raw if float(v["signed_response"]) >= threshold]
    nms = list(band.get("separation_filtered_peaks", {}).get(side, []))
    # In the persisted candidate row, top_k_after is the post-anchor final
    # candidate list.  The guided local Top-K stage is the first configured
    # local_top_k_per_side entries of the separation-filtered list.
    topk = nms[:3]
    final = list(band.get("top_k_after", {}).get(side, []))
    return {"raw_local_extrema": raw, "thresholded_extrema": thresholded,
            "nms_survivor": nms, "side_topk_survivor": topk, "final_candidate": final}

def raw_local_extrema(roi: np.ndarray, config: dict[str, Any], reference: SeamWidthReference,
                      meta: dict[str, Any], ys: np.ndarray, sx: float, sy: float) -> dict[str, list[dict[str, Any]]]:
    """Reconstruct threshold-free local extrema for audit classification.

    The persisted production trace intentionally stores only threshold-surviving
    extrema in ``all_local_signed_edge_peaks``.  This function reproduces the
    same response profile and frozen reference-guided windows, but omits the
    response threshold.  It is not used for candidate selection.
    """
    p = config["processing"]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    max_width = int(p["max_roi_width"]); scale = min(1.0, max_width / gray.shape[1])
    size = (max(1, round(gray.shape[1] * scale)), max(1, round(gray.shape[0] * scale)))
    gray = cv2.resize(gray, size, interpolation=cv2.INTER_AREA) if size != (gray.shape[1], gray.shape[0]) else gray.copy()
    gray = cv2.GaussianBlur(gray, (int(p["gaussian_kernel"]), int(p["gaussian_kernel"])), 0)
    response = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=int(p["sobel_kernel"]))
    half = int(p["scan_band_half_height_px"])
    profiles = cv2.blur(response, (1, 2 * half + 1))
    profiles = cv2.GaussianBlur(profiles, (int(p["profile_gaussian_kernel"]), 1), 0)[ys]
    ox, oy, _, _ = map(float, meta["roi_original_px"])
    yo = oy + ys.astype(np.float64) / sy
    centers, widths = reference.at(yo, 4096, 3072)
    margin = max(int(p["edge_margin_min_px"]), int(round(profiles.shape[1] * float(p["edge_margin_ratio"]))))
    radius = float(config.get("guided_search", {}).get("local_search_radius_width_scale", 0.4)) * widths * sx
    result = {"left": [], "right": []}
    kernel = np.ones((1, 2 * int(p["edge_min_separation_px"]) + 1), dtype=np.uint8)
    for row, (center, width, rad) in enumerate(zip(centers, widths, radius)):
        expected_left = (center - 0.5 * width - ox) * sx
        expected_right = (center + 0.5 * width - ox) * sx
        for side, expected, positive in (("left", expected_left, True), ("right", expected_right, False)):
            lo = max(margin, int(np.floor(expected - rad)))
            hi = min(profiles.shape[1] - margin, int(np.ceil(expected + rad)) + 1)
            signed = profiles[row] if positive else -profiles[row]
            local_max = cv2.dilate(signed.reshape(1, -1), kernel).ravel()
            result[side].append([{"x": int(x), "signed_response": float(signed[x])}
                                 for x in range(max(0, lo), min(profiles.shape[1], hi))
                                 if signed[x] >= local_max[x]])
    return result

def has_near(items: list[dict[str, Any]], x: float, tol: float) -> bool:
    return any(abs(float(v["x"]) - x) <= tol for v in items)

def first_loss(stage: dict[str, list[dict[str, Any]]], x: float, tol: float, anchor: dict[str, Any]) -> str:
    if not has_near(stage["raw_local_extrema"], x, tol): return "local_extrema_not_detected"
    if not has_near(stage["thresholded_extrema"], x, tol): return "response_below_threshold"
    if not has_near(stage["nms_survivor"], x, tol): return "removed_by_nms"
    if not has_near(stage["side_topk_survivor"], x, tol): return "removed_by_side_topk"
    if not has_near(stage["final_candidate"], x, tol):
        return "anchor_merge_not_retained" if anchor.get("valid") else "removed_by_side_topk"
    return "candidate_survived"

def pair_funnel(band: dict[str, Any], ml: float, mr: float, sx: float, tol: float) -> str:
    ps = band.get("pair_stages", {})
    left = has_near(band.get("top_k_after", {}).get("left", []), ml * sx, tol)
    right = has_near(band.get("top_k_after", {}).get("right", []), mr * sx, tol)
    if not left or not right: return "one_side_candidate_missing"
    near = [p for p in ps.get("cartesian_pairs", [])
            if abs(float(p["left_processed"]) - ml*sx) <= tol and abs(float(p["right_processed"]) - mr*sx) <= tol]
    if not near: return "no_manual_near_cartesian_pair"
    p = min(near, key=lambda v: abs(float(v["left_processed"])-ml*sx)+abs(float(v["right_processed"])-mr*sx))
    if not p.get("hard_gate_passed", False):
        reason = p.get("hard_gate_rejection_reason")
        return {"invalid_left_right_order": "invalid_left_right_order",
                "width_below_minimum": "width_hard_gate_rejected",
                "width_above_maximum": "width_hard_gate_rejected",
                "invalid_interior_sample": "sampling_region_invalid",
                "invalid_left_exterior_sample": "sampling_region_invalid",
                "invalid_right_exterior_sample": "sampling_region_invalid"}.get(reason, "evidence_invalid")
    pid = str(p["pair_id"])
    pre = {str(v["pair_id"]) for v in ps.get("pair_hard_gate_after", [])}
    post = {str(v["pair_id"]) for v in ps.get("pair_after_max_pairs_per_band", [])}
    return "entered_dp" if pid in post else ("removed_by_pair_topk" if pid in pre else "evidence_invalid")

def path_errors(trace: dict[str, Any], annotation: dict[str, Any], meta: dict[str, Any]) -> dict[str, Any]:
    ox, oy, _, _ = map(float, meta["roi_original_px"])
    output = {}
    layers = [
        ("selected_raw", trace.get("selected_raw_path", [])),
        ("irls_fitted", [{"y": y, "left": l, "right": r, "center": c}
                         for y, l, r, c in zip(trace.get("result", {}).get("fitted_y", []),
                                               trace.get("result", {}).get("fitted_left", []),
                                               trace.get("result", {}).get("fitted_right", []),
                                               trace.get("result", {}).get("fitted_center", []))]),
        ("temporal_final", [{"y": y, "left": l, "right": r, "center": c, "width": w}
                            for y, l, r, c, w in zip(trace.get("result", {}).get("temporal_output", {}).get("y", []),
                                                     trace.get("result", {}).get("temporal_output", {}).get("left", []),
                                                     trace.get("result", {}).get("temporal_output", {}).get("right", []),
                                                     trace.get("result", {}).get("temporal_output", {}).get("center", []),
                                                     trace.get("result", {}).get("temporal_output", {}).get("width", []))]),
    ]
    for layer, items in layers:
        e = {k: [] for k in ("left", "right", "center", "width")}
        for item in items:
            y = float(item["y"]) - oy
            ml, mr = annotation_x(annotation, "left", y, ox), annotation_x(annotation, "right", y, ox)
            l, r = float(item["left"]), float(item["right"])
            e["left"].append(l - ml); e["right"].append(r - mr)
            e["center"].append(0.5*(l+r) - 0.5*(ml+mr))
            e["width"].append((r-l) - (mr-ml))
        output[layer] = {k: stat(v) for k, v in e.items()}
    return output

def analyze_event(trace: dict[str, Any], annotation: dict[str, Any], meta: dict[str, Any],
                  roi: np.ndarray, config: dict[str, Any], reference: SeamWidthReference,
                  scale: float) -> dict[str, Any]:
    ox, oy, _, _ = map(float, meta["roi_original_px"])
    bands = trace.get("candidate_stage_trace", [])
    ys = np.asarray([int(v["y_processed"]) for v in bands], dtype=np.int32)
    sx, sy = float(trace["scales"]["sx"]), float(trace["scales"]["sy"])
    thresholds = effective_thresholds(roi, config, ys)
    raw_extrema = raw_local_extrema(roi, config, reference, meta, ys, sx, sy)
    loss = {str(t): Counter() for t in TOLERANCES}
    side_loss = {str(t): {"left": Counter(), "right": Counter()} for t in TOLERANCES}
    candidate = {k: [] for k in ("raw", "thresholded", "nms", "topk")}
    anchors = {str(t): 0 for t in TOLERANCES}; pair_counts = Counter(); margins = []; band_rows = []
    for i, band in enumerate(bands):
        y = float(ys[i]); y_roi = y / sy; row = {"event": meta.get("captured_index"), "band_index": int(i), "y_original": oy+y/sy, "scale": scale}
        for side in ("left", "right"):
            mx = (annotation_x(annotation, side, y_roi, ox) - ox) * sx
            persisted = stages(band, side, float(thresholds[i] * scale))
            stage = {"raw_local_extrema": raw_extrema[side][i],
                     "thresholded_extrema": [v for v in raw_extrema[side][i]
                                             if float(v["signed_response"]) >= thresholds[i] * scale],
                     "nms_survivor": persisted["nms_survivor"],
                     "side_topk_survivor": persisted["side_topk_survivor"],
                     "final_candidate": persisted["final_candidate"]}
            anchor = band.get("reference_anchor", {}).get(side, {})
            for tol in TOLERANCES:
                category = first_loss(stage, mx, tol, anchor); loss[str(tol)][category] += 1; side_loss[str(tol)][side][category] += 1
                if anchor.get("valid") and anchor.get("position_processed") is not None and abs(float(anchor["position_processed"]) - mx) <= tol: anchors[str(tol)] += 1
            best = max((float(v["signed_response"]) for v in stage["raw_local_extrema"]
                        if abs(float(v["x"]) - mx) <= 20.0), default=0.0)
            if thresholds[i] > 0 and best > 0: margins.append(best / float(thresholds[i]))
            row[f"{side}_first_loss_20px"] = first_loss(stage, mx, 20.0, anchor); row[f"{side}_manual_x_processed"] = float(mx)
        candidate["raw"].append(sum(len(raw_extrema[s][i]) for s in ("left", "right")))
        candidate["thresholded"].append(sum(sum(float(v["signed_response"]) >= thresholds[i]*scale for v in raw_extrema[s][i]) for s in ("left", "right")))
        candidate["nms"].append(sum(len(band.get("separation_filtered_peaks", {}).get(s, [])) for s in ("left", "right")))
        candidate["topk"].append(sum(len(band.get("top_k_after", {}).get(s, [])) for s in ("left", "right")))
        pair_counts[pair_funnel(band, annotation_x(annotation, "left", y_roi, ox) - ox,
                                annotation_x(annotation, "right", y_roi, ox) - ox, sx, 20.0)] += 1
        band_rows.append(row)
    return {"event_name": meta.get("captured_index"), "band_count": len(bands), "side_records": len(bands)*2,
            "first_loss": {k: dict(v) for k, v in loss.items()},
            "side_first_loss": {k: {s: dict(c) for s, c in v.items()} for k, v in side_loss.items()},
            "candidate_count_stats": {k: stat(v) for k, v in candidate.items()}, "anchor_near": anchors,
            "response_margin": stat(margins), "pair_counts": dict(pair_counts),
            "pair_count_stats": stat([float(len(b.get("pair_stages", {}).get("pair_after_max_pairs_per_band", []))) for b in bands]),
            "path_errors": path_errors(trace, annotation, meta), "band_rows": band_rows, "result": trace.get("result", {})}

def exact_baseline(events: list[Path], config_path: Path, ref_path: Path) -> dict[str, Any]:
    config, reference = load_stable_config(config_path), SeamWidthReference.load(ref_path); rows=[]; ok=True
    for event in events:
        full, meta, old, _ = read_event(event); tracker=StableSeamTracker(config, reference=reference)
        tracker.process(full, diagnostics=True, provenance=True, response_threshold_scale=1.0); new=tracker.last_provenance or {}
        fields=sorted(set(old) | set(new)); mismatches=[f for f in fields if old.get(f)!=new.get(f)]
        rows.append({"event":event.name,"passed":not mismatches,"mismatched_fields":mismatches,"old_provenance_sha256":digest(event/"provenance.json")}); ok &= not mismatches
    return {"baseline_reproduction":bool(ok),"fields_compared":list(fields),"events":rows,"policy":"stop before non-1.0 scales if false"}

def run_scale(events: list[Path], annotations: dict[str, dict[str, Any]], config_path: Path, ref_path: Path, scale: float) -> dict[str, Any]:
    config, reference=load_stable_config(config_path),SeamWidthReference.load(ref_path); single=[]; loaded={}
    for event in events:
        full, meta, _, roi=read_event(event); loaded[event.name]=(full,meta,roi); tracker=StableSeamTracker(config,reference=reference)
        result=tracker.process(full,provenance=True,response_threshold_scale=scale)
        row=analyze_event(tracker.last_provenance or {},annotations[event.name],meta,roi,config,reference,scale)
        row.update({"event_name":event.name,"detected":bool(result.detected),"state":result.tracking_state.name}); single.append(row)
    ordered=sorted(events,key=lambda e:int(load(e/"metadata.json")["header_stamp_ns"])); tracker=StableSeamTracker(config,reference=reference); seq=[]
    for event in ordered:
        full,meta,_=loaded[event.name]; r=tracker.process(full,provenance=True,response_threshold_scale=scale); t=tracker.last_provenance or {}
        seq.append({"event":event.name,"header_stamp_ns":meta["header_stamp_ns"],"detected":bool(r.detected),"state":r.tracking_state.name,
                    "predicted_only":bool(r.predicted_only),"failure_reasons":list(r.failure_reasons),"processing_ms":float(r.processing_ms),
                    "joint_inlier_ratio":t.get("result",{}).get("joint_inlier_ratio"),"fit_residual_px":t.get("result",{}).get("fit_residual_px")})
    overall=Counter(); [overall.update(row["first_loss"]["20.0"]) for row in single]
    return {"scale":scale,"events":single,"overall_first_loss_20px":dict(overall),"overall_side_records":sum(v["side_records"] for v in single),
            "sequential":seq,"sequential_entered_tracking":any(v["state"]=="TRACKING" for v in seq)}

def aggregate(run: dict[str, Any]) -> dict[str, Any]:
    events=run["events"]; total=sum(v["side_records"] for v in events); bands=sum(v["band_count"] for v in events); loss=Counter(run["overall_first_loss_20px"]); recall={}
    for tol in map(str,TOLERANCES):
        c=Counter(); [c.update(e["first_loss"][tol]) for e in events]
        recall[tol]={"candidate_survived":c.get("candidate_survived",0),"candidate_survived_ratio":c.get("candidate_survived",0)/total if total else None,"first_loss":dict(c)}
    pairs=Counter(); [pairs.update(e["pair_counts"]) for e in events]; layers={}
    for layer in ("selected_raw","irls_fitted","temporal_final"):
        layers[layer]={}
        for field in ("left","right","center","width"):
            values=[e["path_errors"].get(layer,{}).get(field,{}).get("p95") for e in events if e["path_errors"].get(layer,{}).get(field,{}).get("p95") is not None]
            layers[layer][field]=stat([float(v) for v in values])
    return {"scale":run["scale"],"bands":bands,"side_records":total,"first_loss_20px":dict(loss),
            "first_loss_20px_ratio":{k:v/total for k,v in loss.items()},"recall_by_tolerance":recall,"pair_counts":dict(pairs),
            "layer_p95_across_events":layers,"single_states":dict(Counter(e["state"] for e in events)),
            "sequential_states":dict(Counter(e["state"] for e in run["sequential"])),"sequential_entered_tracking":run["sequential_entered_tracking"]}

def run_negative(config_path: Path, ref_path: Path, scales: tuple[float,...]) -> dict[str,Any]:
    directory=ROOT/"output/s1_p2d_field_samples/20260909_174450_negative"; files=sorted(directory.glob("frame_*.bmp")); config,reference=load_stable_config(config_path),SeamWidthReference.load(ref_path); output={}
    for scale in scales:
        rows=[]
        for path in files:
            tracker=StableSeamTracker(config,reference=reference); r=tracker.process(cv2.imread(str(path)),response_threshold_scale=scale)
            rows.append({"frame":path.name,"detected":bool(r.detected),"state":r.tracking_state.name,"crossing":int(r.fitted_output_crossing_count),"failure_reasons":list(r.failure_reasons)})
        output[str(scale)]={"count":len(rows),"all_detected_false":all(not r["detected"] for r in rows),"all_lost":all(r["state"]=="LOST" for r in rows),"all_crossing_zero":all(r["crossing"]==0 for r in rows),"rows":rows}
    return {"source":str(directory),"input_sha256":{p.name:digest(p) for p in files},"scales":output}

def run_v2(config_path: Path, ref_path: Path, scales: tuple[float,...]) -> dict[str,Any]:
    path=ROOT/"output/s1_p2h_three_frame_ground_truth_evaluation/annotation_manifest.json"; manifest=load(path); config,reference=load_stable_config(config_path),SeamWidthReference.load(ref_path); output={}
    for scale in scales:
        rows=[]
        for name,item in manifest["cases"].items():
            image=cv2.imread(item["raw"]); ann=load(Path(item["annotation"])); tracker=StableSeamTracker(config,reference=reference); r=tracker.process(image,provenance=True,response_threshold_scale=scale); t=tracker.last_provenance or {}; errors={k:[] for k in ("left","right","center","width")}
            for p in t.get("selected_raw_path",[]):
                y=float(p["y"]); ml=funnel.interpolate_manual(ann,"left",y); mr=funnel.interpolate_manual(ann,"right",y); l,rr=float(p["left"]),float(p["right"])
                errors["left"].append(abs(l-ml)); errors["right"].append(abs(rr-mr)); errors["center"].append(abs(.5*(l+rr)-.5*(ml+mr))); errors["width"].append(abs((rr-l)-(mr-ml)))
            rows.append({"case":name,"detected":bool(r.detected),"state":r.tracking_state.name,"crossing":int(r.fitted_output_crossing_count),"errors":{k:stat(v) for k,v in errors.items()},"input_sha256":digest(Path(item["raw"]))})
        output[str(scale)]={"rows":rows,"count":len(rows),"all_crossing_zero":all(r["crossing"]==0 for r in rows)}
    return {"manifest":str(path),"manifest_sha256":digest(path),"config_sha256":manifest["config_sha256"],"reference_sha256":manifest["reference_sha256"],"scales":output}

def benchmark(config_path: Path, ref_path: Path, scales: tuple[float,...], images: list[np.ndarray]) -> dict[str,Any]:
    config,reference=load_stable_config(config_path),SeamWidthReference.load(ref_path); output={}
    for scale in scales:
        tracker=StableSeamTracker(config,reference=reference); [tracker.process(images[i%len(images)],response_threshold_scale=scale) for i in range(30)]; samples=[]; before=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        for i in range(300):
            r=tracker.process(images[i%len(images)],response_threshold_scale=scale); samples.append({k:float(v) for k,v in r.stage_timings_ms.items()}|{"total":float(r.processing_ms)})
        after=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        output[str(scale)]={"warmup_frames":30,"formal_frames":300,"timings_ms":{k:stat([v.get(k,0.) for v in samples]) for k in ("preprocess","candidate_generation","pairing","dynamic_programming","fitting","total")},"rss_maxrss_delta_kb":int(max(0,after-before)),"benchmark_type":"offline_repeated_three_frame_cycle"}
    return output

def plot(out: Path, aggs: list[dict[str,Any]]) -> None:
    canvas=np.full((900,1400,3),255,np.uint8); plots=[("candidate survival",lambda a:a["first_loss_20px"].get("candidate_survived",0)/a["side_records"]),("threshold loss",lambda a:a["first_loss_20px"].get("response_below_threshold",0)/a["side_records"]),("Top-K loss",lambda a:a["first_loss_20px"].get("removed_by_side_topk",0)/a["side_records"]),("pair entered DP",lambda a:a["pair_counts"].get("entered_dp",0)/a["bands"]),("selected center P95",lambda a:a["layer_p95_across_events"]["selected_raw"]["center"].get("p95"))]
    for i,(title,fn) in enumerate(plots):
        x0,y0=70+(i%2)*670,60+(i//2)*270; w,h=560,180; values=[fn(a) for a in aggs]; finite=[float(v) for v in values if v is not None]; lo,hi=(min(finite),max(finite)) if finite else (0,1); hi=hi+1 if math.isclose(lo,hi) else hi
        cv2.putText(canvas,title,(x0,y0-12),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,0),2); cv2.rectangle(canvas,(x0,y0),(x0+w,y0+h),(210,210,210),1); pts=[]
        for j,v in enumerate(values):
            if v is None: continue
            p=(x0+int(j*w/max(1,len(values)-1)),y0+h-int((float(v)-lo)/(hi-lo)*h)); pts.append(p); cv2.circle(canvas,p,6,(0,0,210) if j==0 else (40,130,40),-1); cv2.putText(canvas,f"{aggs[j]['scale']:.2f}",(p[0]-16,y0+h+22),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1)
        for p,q in zip(pts,pts[1:]): cv2.line(canvas,p,q,(50,50,50),2)
    cv2.putText(canvas,"response_threshold_scale offline ablation; red point=baseline 1.00",(40,850),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,0),2); cv2.imwrite(str(out/"diagnostic_montage.jpg"),canvas)

def write_csv(path: Path, rows: list[dict[str,Any]]) -> None:
    keys=sorted({k for r in rows for k in r});
    with path.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=keys); w.writeheader()
        for row in rows: w.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in row.items()})

def main() -> int:
    ap=argparse.ArgumentParser(); ap.add_argument("--session",type=Path,default=SESSION); ap.add_argument("--annotations",type=Path,default=ANNOTATIONS); ap.add_argument("--config",type=Path,default=CONFIG); ap.add_argument("--reference",type=Path,default=REFERENCE); ap.add_argument("--output",type=Path,default=ROOT/"output/field_w2_a01_response_threshold_ablation"); ap.add_argument("--skip-performance",action="store_true"); a=ap.parse_args()
    out=a.output.resolve()
    if out.exists() and any(out.iterdir()): raise SystemExit(f"refusing to overwrite non-empty output: {out}")
    out.mkdir(parents=True,exist_ok=True); events=[a.session/e for e in EVENTS]; annotations={}
    for event in EVENTS:
        fs=list((a.annotations/event).glob("*.json"))
        if not fs: raise SystemExit(f"missing annotation: {event}")
        annotations[event]=load(fs[0])
    before={str(e/"roi_raw.bmp"):digest(e/"roi_raw.bmp") for e in events}; config=load_stable_config(a.config); source={"formula":"max(edge_min_response, percentile(abs(response_profile), edge_percentile))","edge_min_response":float(config["processing"]["edge_min_response"]),"edge_percentile":float(config["processing"]["edge_percentile"])}
    baseline=exact_baseline(events,a.config,a.reference); put(out/"baseline_reproduction.json",baseline)
    if not baseline["baseline_reproduction"]:
        put(out/"diagnostic_summary.json",{"status":"stopped_baseline_reproduction_failed","baseline_reproduction":False}); return 2
    margins=[]; margin_side=defaultdict(list); margin_event=defaultdict(list); margin_y=defaultdict(list)
    threshold_values=[]; threshold_floor_hits=0
    for event in events:
        _,meta,trace,roi=read_event(event); bands=trace["candidate_stage_trace"]; ys=np.asarray([int(b["y_processed"]) for b in bands]); sx,sy=float(trace["scales"]["sx"]),float(trace["scales"]["sy"]); thresholds=effective_thresholds(roi,config,ys); profiles,_=funnel.make_profiles(roi,config,ys); threshold_values.extend(float(v) for v in thresholds); threshold_floor_hits += int(np.count_nonzero(thresholds <= float(config["processing"]["edge_min_response"])+1e-12)); raw_extrema=raw_local_extrema(roi,config,SeamWidthReference.load(a.reference),meta,ys,sx,sy); ox,oy,_,_=meta["roi_original_px"]
        for i,band in enumerate(bands):
            y=float(ys[i])/sy
            for side in ("left","right"):
                mx=(annotation_x(annotations[event.name],side,y,float(ox))-float(ox))*sx; anchor=band.get("reference_anchor",{}).get(side,{})
                stage={"raw_local_extrema":raw_extrema[side][i],"thresholded_extrema":[v for v in raw_extrema[side][i] if float(v["signed_response"])>=thresholds[i]],"nms_survivor":stages(band,side,float(thresholds[i]))["nms_survivor"],"side_topk_survivor":stages(band,side,float(thresholds[i]))["side_topk_survivor"],"final_candidate":stages(band,side,float(thresholds[i]))["final_candidate"]}
                if first_loss(stage,mx,20.,anchor)=="response_below_threshold":
                    signed=profiles[i] if side=="left" else -profiles[i]; lo=max(0,int(np.floor(mx-20.0))); hi=min(len(signed),int(np.ceil(mx+20.0))+1); best=float(np.max(signed[lo:hi])) if hi>lo else 0.0; value=best/max(float(thresholds[i]),1e-12); margins.append(value); margin_side[side].append(value); margin_event[event.name].append(value); margin_y[str(int(meta["roi_original_px"][1]+ys[i]/sy)//300*300)].append(value)
    source.update({"effective_threshold_stats":stat(threshold_values),"floor_hit_count":threshold_floor_hits,"floor_hit_ratio":threshold_floor_hits/max(len(threshold_values),1)})
    put(out/"response_margin_distribution.json",{"definition":"best expected-polarity local-extrema response/current effective threshold","baseline_response_threshold_records":len(margins),"overall":stat(margins),"by_side":{k:stat(v) for k,v in margin_side.items()},"by_event":{k:stat(v) for k,v in margin_event.items()},"by_y_interval":{k:stat(v) for k,v in margin_y.items()}}); put(out/"threshold_source_analysis.json",source)
    runs=[]; aggs=[]
    for scale in SCALES:
        run=run_scale(events,annotations,a.config,a.reference,scale); runs.append(run); aggs.append(aggregate(run))
    put(out/"scale_comparison.json",{"scales":runs,"aggregates":aggs}); write_csv(out/"scale_comparison.csv",aggs); put(out/"candidate_recall_comparison.json",{str(v["scale"]):v["recall_by_tolerance"] for v in aggs}); put(out/"first_loss_transition.json",{str(v["scale"]):v["first_loss_20px"] for v in aggs}); put(out/"pair_recall_comparison.json",{str(v["scale"]):{"pair_counts":v["pair_counts"]} for v in aggs}); put(out/"layer_error_comparison.json",{str(v["scale"]):v["layer_p95_across_events"] for v in aggs}); put(out/"sequential_replay_comparison.json",{str(v["scale"]):v["sequential"] for v in runs})
    put(out/"negative_regression.json",run_negative(a.config,a.reference,SCALES)); put(out/"frozen_v2_regression.json",run_v2(a.config,a.reference,SCALES))
    put(out/"performance_comparison.json",{"status":"skipped"} if a.skip_performance else {"scales":benchmark(a.config,a.reference,SCALES,[read_event(e)[0] for e in events])}); plot(out,aggs)
    candidates=[]
    for v in aggs:
        if v["scale"]==1.: continue
        rec=v["recall_by_tolerance"]["20.0"]["candidate_survived_ratio"]; pair=v["pair_counts"].get("entered_dp",0)/max(v["bands"],1); candidates.append({"scale":v["scale"],"candidate_recall_20px":rec,"pair_entered_dp_recall":pair,"eligible_basic":bool(rec>=.90 and pair>=.85)})
    eligible=[v for v in candidates if v["eligible_basic"]]; recommendation=max((v["scale"] for v in eligible),default=None)
    summary={"status":"complete","baseline_reproduction":True,"input_session":str(a.session),"config":str(a.config),"reference":str(a.reference),"config_sha256":digest(a.config),"reference_sha256":digest(a.reference),"input_sha256_before":before,"scales":list(SCALES),"frozen_parameters":["ROI","reference","guided radius/window","polarity","local extrema","NMS","edge Top-K","anchor","pair","DP","IRLS","geometry gates","temporal"],"threshold_source":source,"aggregates":aggs,"basic_candidate_assessment":candidates,"recommendation":recommendation if recommendation is not None else "response_threshold_only_insufficient","recommendation_scope":"offline candidate only" if recommendation is not None else "no production value","strict_selection_note":"See negative_regression, frozen_v2_regression and performance_comparison; no production parameter is changed."}
    put(out/"diagnostic_summary.json",summary); put(out/"next_single_action.json",{"conclusion":"response_threshold_only_insufficient" if recommendation is None else "offline_candidate_only","recommended_scale":recommendation,"production_yaml_modified":False}); put(out/"input_sha256_after.json",{str(e/"roi_raw.bmp"):digest(e/"roi_raw.bmp") for e in events}); (out/"README.md").write_text("# Response-threshold ablation\n\nThe production effective threshold is computed first as max(edge_min_response, percentile(abs(response), edge_percentile)); the experimental scale is applied once after that computation. All other behavior and input files are frozen. Scale 1.00 must reproduce persisted provenance before other scales run. Results are offline evidence only.\n",encoding="utf-8")
    print(json.dumps({"status":"complete","baseline_reproduction":True,"recommendation":summary["recommendation"],"output":str(out)},ensure_ascii=False,indent=2)); return 0

if __name__=="__main__":
    raise SystemExit(main())
