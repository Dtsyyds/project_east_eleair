#!/usr/bin/env python3
"""S1-D8: compare fixed 0-degree D1/D4 evidence with manual boundaries."""
from __future__ import annotations

import argparse, csv, hashlib, json, math
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.experiment_real_seam_band import find_boundary_track, sha256
from scripts.experiment_real_seam_boundary_confidence import (
    SCAN_HALF_BAND, SCAN_STEP, _draw_accepted_overlay, _pair_key, _path_for_records,
    _robust_model, _scan_records, _track_jump_stats,
)
from scripts.experiment_real_seam_orientation import _track_inside_valid_region, rotate_processing_image
from tank_vision.config import load_config
from tank_vision.preprocessing.normalize import select_gray_channel
from tank_vision.preprocessing.roi import make_image_view

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "output/s1_real_seam_images/seam_center_lit_quality_01.bmp"
DEFAULT_ANNOTATION = ROOT / "output/s1_d7_manual_ground_truth/seam_center_lit_quality_01_annotation.json"
DEFAULT_OUTPUT = ROOT / "output/s1_d8_manual_boundary_diagnostic"


def _points(annotation: dict, key: str) -> list[tuple[float, float]]:
    return [(float(p["x"]), float(p["y"])) for p in annotation[key]]


def validate_manual_annotation(annotation: dict, input_path: Path, width: int, height: int) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    if annotation.get("input_sha256") != sha256(input_path):
        raise ValueError("manual annotation input_sha256 does not match input image")
    if annotation.get("image_width") != width or annotation.get("image_height") != height:
        raise ValueError("manual annotation image dimensions do not match input")
    if annotation.get("annotation_type") != "manual":
        raise ValueError("annotation_type is not manual")
    if annotation.get("interpolation_used") is not False:
        raise ValueError("interpolation_used must be false")
    left, right = _points(annotation, "left_boundary_manual_points"), _points(annotation, "right_boundary_manual_points")
    for name, points in (("left", left), ("right", right)):
        if len(points) < 6:
            raise ValueError(f"{name} manual boundary has fewer than 6 points")
        if points != sorted(points, key=lambda p: (p[1], p[0])):
            raise ValueError(f"{name} manual points are not sorted by y")
        if any(not (np.isfinite(x) and np.isfinite(y) and 0 <= x < width and 0 <= y < height) for x, y in points):
            raise ValueError(f"{name} manual point is invalid or out of bounds")
    return left, right


def _candidate_json(items) -> list[dict]:
    return [{"x_processed": int(x.x), "response": float(x.strength), "polarity": x.polarity} for x in items]


def _num_summary(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "p50": None, "p90": None, "p95": None, "max": None}
    a = np.asarray(values, dtype=float)
    return {"count": int(len(a)), "p50": float(np.percentile(a, 50)), "p90": float(np.percentile(a, 90)), "p95": float(np.percentile(a, 95)), "max": float(np.max(a))}


def _cross(canvas, x: int, y: int, colour: tuple[int, int, int]) -> None:
    cv2.drawMarker(canvas, (x, y), colour, cv2.MARKER_TILTED_CROSS, 16, 2, cv2.LINE_AA)


def run_diagnostic(input_path: Path, annotation_path: Path, config_path: Path, output_dir: Path) -> dict:
    image = cv2.imread(str(input_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot decode input image")
    h, w = image.shape[:2]
    annotation = json.loads(annotation_path.read_text(encoding="utf-8"))
    left_manual, right_manual = validate_manual_annotation(annotation, input_path, w, h)
    y_min, y_max = max(left_manual[0][1], right_manual[0][1]), min(left_manual[-1][1], right_manual[-1][1])
    if y_min >= y_max:
        raise ValueError("manual boundary common y interval is empty")
    left_y, left_x = np.asarray([p[1] for p in left_manual]), np.asarray([p[0] for p in left_manual])
    right_y, right_x = np.asarray([p[1] for p in right_manual]), np.asarray([p[0] for p in right_manual])
    config = load_config(config_path)
    view = make_image_view(image, config.pipeline.roi, 896)
    selected = cv2.GaussianBlur(select_gray_channel(view.image, config.seam.color_channel), (5, 5), 0)
    rotated, valid, matrix = rotate_processing_image(selected, 0.0)
    provisional = _track_inside_valid_region(find_boundary_track(rotated), valid)
    model = _robust_model(provisional)
    records = _scan_records(rotated, valid, model)
    path = _path_for_records(records, model)
    d1_by_y = {int(p.y): p for p in provisional.points}
    d4_by_y = {int(p.y): p for p in path.reliable_points}
    scan_rows = []
    for index, record in enumerate(records):
        y_proc = float(record.y); y_orig = y_proc / view.scale_y
        if not (y_min <= y_orig <= y_max):
            continue
        ml = float(np.interp(y_orig, left_y, left_x)); mr = float(np.interp(y_orig, right_y, right_x)); mc=(ml+mr)/2; mw=mr-ml
        d1 = d1_by_y.get(record.y); d4 = d4_by_y.get(record.y)
        near_left = [c for c in record.edge_candidates[0] if abs(c.x / view.scale_x - ml) <= 100]
        near_right = [c for c in record.edge_candidates[1] if abs(c.x / view.scale_x - mr) <= 100]
        d1_near = bool(d1 and abs(d1.left/view.scale_x-ml)<=100 and abs(d1.right/view.scale_x-mr)<=100)
        accepted = d4 is not None
        d4_reasons = []
        for ev in record.candidates:
            if d1 and _pair_key(ev.pair) == _pair_key(d1): d4_reasons = list(ev.reasons)
        if not near_left or not near_right: category="candidate_missing"
        elif not d1_near: category="pairing_selected_wrong"
        elif not accepted: category="correct_pair_rejected_by_d4"
        else:
            err=max(abs(d4.left/view.scale_x-ml),abs(d4.right/view.scale_x-mr))
            category="accepted_near_manual" if err <= 100 else "accepted_far_from_manual"
        def point_values(p):
            return None if p is None else {"left_x":p.left/view.scale_x,"right_x":p.right/view.scale_x,"center_x":p.center/view.scale_x,"width_px":p.width/view.scale_x}
        d1v=point_values(d1); d4v=point_values(d4)
        errors={}
        if d1v:
            errors["d1"]={k:{"signed_px":float(d1v[k]-v),"absolute_px":float(abs(d1v[k]-v))} for k,v in {"left_x":ml,"right_x":mr,"center_x":mc,"width_px":mw}.items()}
        if d4v:
            errors["d4"]={k:{"signed_px":float(d4v[k]-v),"absolute_px":float(abs(d4v[k]-v))} for k,v in {"left_x":ml,"right_x":mr,"center_x":mc,"width_px":mw}.items()}
        scan_rows.append({"scan_index":index,"y_processed":y_proc,"y_original":y_orig,"manual_left_x":ml,"manual_right_x":mr,"manual_center_x":mc,"manual_width_px":mw,"d1_left_candidates":_candidate_json(record.edge_candidates[0]),"d1_right_candidates":_candidate_json(record.edge_candidates[1]),"d1_selected":d1v,"d4_accepted":accepted,"d4_rejection_reasons":d4_reasons,"d4_filtered":d4v,"errors":errors,"category":category})
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir/"scan_band_diagnostics.json").write_text(json.dumps({"input_path":str(input_path),"manual_common_y_range_original":[y_min,y_max],"scan_bands":scan_rows},ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    fields=["scan_index","y_processed","y_original","manual_left_x","manual_right_x","manual_center_x","manual_width_px","d1_selected","d4_accepted","d4_rejection_reasons","d4_filtered","errors","category"]
    with (output_dir/"scan_band_diagnostics.csv").open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=fields); writer.writeheader()
        for row in scan_rows: writer.writerow({k:json.dumps(row[k],ensure_ascii=False) if isinstance(row[k],(dict,list)) else row[k] for k in fields})
    # Diagnostic overlay is in original image coordinates; no extrapolated manual lines.
    overlay=image.copy(); lm=np.asarray([(int(round(x)),int(round(y))) for x,y in left_manual]); rm=np.asarray([(int(round(x)),int(round(y))) for x,y in right_manual])
    cv2.polylines(overlay,[lm.reshape(-1,1,2)],False,(255,0,0),8,cv2.LINE_AA); cv2.polylines(overlay,[rm.reshape(-1,1,2)],False,(0,255,0),8,cv2.LINE_AA)
    common_y=np.linspace(y_min,y_max,160); lcx=np.interp(common_y,left_y,left_x); rcx=np.interp(common_y,right_y,right_x); mid=np.column_stack(((lcx+rcx)/2,common_y)).astype(np.int32); cv2.polylines(overlay,[mid.reshape(-1,1,2)],False,(0,255,255),8,cv2.LINE_AA)
    for row in scan_rows:
        y=int(round(row['y_original']));
        if row['d1_selected']:
            x=int(round(row['d1_selected']['center_x'])); _cross(overlay,x,y,(255,255,0))
        if row['d4_accepted'] and row['d4_filtered']:
            _cross(overlay,int(round(row['d4_filtered']['center_x'])),y,(255,255,255))
        elif row['category']=='candidate_missing': _cross(overlay,int(round(row['manual_center_x'])),y,(255,0,255))
        else: _cross(overlay,int(round(row['manual_center_x'])),y,(0,0,255))
    cv2.imwrite(str(output_dir/"manual_vs_d1_vs_d4_overlay.png"),overlay)
    # Simple cv2 error plot, preserving original y direction.
    plot=np.full((700,1200,3),255,np.uint8); cv2.putText(plot,'S1-D8 signed errors vs original y (px)',(20,30),0,.8,(0,0,0),2)
    if scan_rows:
        ys=np.asarray([r['y_original'] for r in scan_rows]); x0,x1=70,1160; y0,y1=80,650
        def px_y(v): return int(y0+(v-y_min)/(y_max-y_min)*(y1-y0))
        cv2.line(plot,(x0,y0),(x0,y1),(0,0,0),1); cv2.line(plot,(x0,px_y((y_min+y_max)/2)),(x1,px_y((y_min+y_max)/2)),(220,220,220),1)
        for key,col in [('left_x',(255,0,0)),('right_x',(0,160,0)),('center_x',(0,180,180)),('width_px',(100,0,180))]:
            pts=[]
            for r in scan_rows:
                if r['errors'].get('d4'): pts.append((int(x0+(r['errors']['d4'][key]['signed_px']+500)/1000*(x1-x0)),px_y(r['y_original'])))
            if len(pts)>1: cv2.polylines(plot,[np.asarray(pts)],False,col,2)
        cv2.putText(plot,'x axis: signed error -500..+500 px; y axis: original y',(20,685),0,.55,(0,0,0),1)
    cv2.imwrite(str(output_dir/"error_by_y_plot.png"),plot)
    metric_names={'left_x':[], 'right_x':[], 'center_x':[], 'width_px':[]}; d1_near=near_count=correct=correct_near=kept_near=wrong_accept=0
    counts={k:0 for k in ['candidate_missing','pairing_selected_wrong','correct_pair_rejected_by_d4','accepted_near_manual','accepted_far_from_manual']}
    for r in scan_rows:
        counts[r['category']]+=1
        near=bool(r['d1_selected'] and max(abs(r['errors']['d1'][k]['signed_px']) for k in ['left_x','right_x'])<=100)
        if r['d1_left_candidates'] and r['d1_right_candidates']: near_count+=1
        if near: correct+=1
        if near and r['d4_accepted']: kept_near+=1
        if r['category']=='accepted_far_from_manual': wrong_accept+=1
        if r['d4_accepted'] and r['errors'].get('d4'):
            for k in metric_names: metric_names[k].append(r['errors']['d4'][k]['absolute_px'])
    tolerance_ratios = {}
    for source in ("d1", "d4"):
        available = [r for r in scan_rows if r["errors"].get(source)]
        tolerance_ratios[source] = {str(t): {k: sum(r["errors"][source][k]["absolute_px"] <= t for r in available) / max(len(available), 1) for k in ("left_x", "right_x", "center_x", "width_px")} for t in (25, 50, 100)}
    summary={'input_path':str(input_path),'input_sha256_before':sha256(input_path),'annotation_sha256_before':sha256(annotation_path),'manual_common_y_range_original':[y_min,y_max],'evaluated_scan_band_count':len(scan_rows),'category_counts':{k:{'count':v,'ratio':v/max(len(scan_rows),1)} for k,v in counts.items()},'errors_absolute_px':{k:_num_summary(v) for k,v in metric_names.items()},'error_tolerance_ratios':tolerance_ratios,'d1_manual_boundary_near_candidate_recall':near_count/max(len(scan_rows),1),'d1_correct_pair_rate':correct/max(len(scan_rows),1),'d4_near_manual_pair_retention':kept_near/max(correct,1),'d4_wrong_acceptance_rate':wrong_accept/max(len(scan_rows),1),'parameters':{'angle_deg':0,'max_dimension':896,'scan_step_processed_px':SCAN_STEP,'scan_band_half_height_processed_px':SCAN_HALF_BAND},'conclusion':'diagnostic only; production algorithms and thresholds unchanged'}
    (output_dir/"error_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    # 2x2 diagnostic montage.
    imgs=[('original',image),('manual + D1/D4',overlay),('error by y',plot),('categories',overlay)]
    cells=[]
    for title,im in imgs:
        im=cv2.resize(im,(896,672)); cv2.putText(im,title,(18,34),0,.8,(255,255,255),2,cv2.LINE_AA); cells.append(im)
    cv2.imwrite(str(output_dir/"diagnostic_montage.jpg"),np.vstack([np.hstack(cells[:2]),np.hstack(cells[2:])]),[cv2.IMWRITE_JPEG_QUALITY,95])
    after_img,after_ann=sha256(input_path),sha256(annotation_path)
    summary['input_sha256_after']=after_img; summary['annotation_sha256_after']=after_ann; summary['inputs_unchanged']=(summary['input_sha256_before']==after_img and summary['annotation_sha256_before']==after_ann)
    (output_dir/"error_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return summary


def main() -> int:
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--input',type=Path,default=DEFAULT_INPUT); parser.add_argument('--annotation',type=Path,default=DEFAULT_ANNOTATION); parser.add_argument('--config',type=Path,default=ROOT/'configs/default.yaml'); parser.add_argument('--output',type=Path,default=DEFAULT_OUTPUT); args=parser.parse_args(); print(json.dumps(run_diagnostic(args.input,args.annotation,args.config,args.output),ensure_ascii=False,indent=2)); return 0

if __name__=='__main__': raise SystemExit(main())
