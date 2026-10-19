# FIELD-W3-PERF-ISOLATION-P0-P2 Engineering Checkpoint

Date: 2026-09-18
Repository: `tank_vision_inspection`
Pre-checkpoint commit: `33e59417a17e764a127c7d91be1b5098f9a348ba`

This checkpoint freezes the performance-isolation result before the next engineering phase. The Git commit containing this document preserves the complete source diff relative to the pre-checkpoint commit. Historical run directories are retained outside Git and are not deleted or overwritten.

## Frozen status

```text
system_performance_isolated=true
system_integration_ready=true
algorithm_field_validation_pending=true
production_ready=false
algorithm_behavior_modified=false
```

`system_integration_ready=true` is limited to the P0 result and the current system contract. It is not an algorithm field-validation or production-readiness approval.

## Default field profile

P1 is the current default field-running profile:

```text
geometry_provenance_sampling_enabled=false
auto_save_failures=true
auto_save_full_frame=false
publish_debug_image=false
roi_validation_recording_enabled=false
lightweight MCAP=true
per_frame_metrics_log=true
```

P2 is not a default profile. Its diagnostic-only parameters are explicitly scoped to the short P2 test: `geometry_provenance_sampling_enabled=true`, `sample_period_sec=30.0`, and `max_events=3`. They must not be copied into the production or normal field defaults.

## Source, install, and configuration identity

SHA-256 values captured at this checkpoint:

| Item | Path | SHA-256 |
|---|---|---|
| Stable seam tracker source | `tank_vision/stable_seam_tracker.py` | `121f3578ea5b47e1dc29c531d0da6ff42a22bf2b75d6a55be261d33e5905b23c` |
| Stable seam ROS node source | `ros2_ws/src/tank_vision_ros/tank_vision_ros/stable_seam_node.py` | `5b7ee0c056104df13de77e7e2350c1016db691fed7900387e5ae8bb67e9cd743` |
| Installed node entry point | `ros2_ws/install_s100p_humble/tank_vision_ros/lib/tank_vision_ros/stable_seam_node` | `0a406b6e0f8b17a2535fe90d522414cb2162b1b3724fc8cf3711e99184ee63af` |
| Installed validation launch source | `physic_bev/src/inspection_bringup/launch/visual_field_validation.launch.py` | `9dcb994b7af99769304b7e68be0a71984ccd1db9c1be30d7b4fe444f903957d7` |
| Guided YAML | `configs/s100p_guided_anchor_seam_4096x3072.yaml` | `6acf482a6cb42b8fff65c8067f4ab298b24e653d4e0453e324b79078550ba438` |
| Geometry reference | `configs/seam_geometry_reference_s100p_live_v1.json` | `0324547cf7d7d304b16deca1e06550c3c212fd1615c21caab5f6c4a49bde7935` |

The installed validation launch is a symlink to the recorded `physic_bev/src/...` source. The tracker source and production YAML/reference were not changed by this performance-isolation task.

## P0/P1/P2 runs

All runs used the same camera, central weld, exposure mode, algorithm configuration, and reference, with normal SIGINT shutdown. The detailed JSON/CSV reports remain at `output/field_w3_perf_isolation/`.

| Case | Run directory | Camera/Input Hz | Tracker Hz | `processing_ms` P50/P95/max (ms) | `process_packet_wall_ms` P50/P95/max (ms) | Slot overwritten | Result |
|---|---|---:|---:|---:|---:|---:|---|
| P0 baseline | `/media/mcap_data/runs/FIELD-W3-PERF-P0-20260918_110200` | 6.606 | 6.624 | 89.54 / 110.82 / 134.08 | 91.17 / 112.58 / 135.05 | 0 | pass |
| P1 snapshot | `/media/mcap_data/runs/FIELD-W3-PERF-P1-20260918_110400` | 6.453 | 6.471 | 88.03 / 109.97 / 135.05 | 89.80 / 112.78 / 136.83 | 0 | pass |
| P2 provenance | `/media/mcap_data/runs/FIELD-W3-PERF-P2-20260918_111300` | 5.053 | 4.893 | 88.53 / 108.92 / 208.60 | 90.13 / 110.97 / 2799.80 | 12 | diagnostic fail |

P1 requested 9 automatic snapshots, saved 7, and rejected 2; it still remained above 6 Hz. P2 wrote 3 provenance events, performed only 3 second-pass tracker runs and 3 task builds, then suppressed 275 further attempts before the expensive work. P2's provenance second-pass maximum was 2693.39 ms. Its `system_pass=false` is due to the rate/slot-overwrite result; MCAP validation, finalization, snapshots/provenance completion, timestamp monotonicity, replay readability, normal stop, and orphan-process checks passed.

## Interpretation

The full-resolution ROS image plus tracker chain reaches the 6 Hz target in P0. Automatic failure snapshots do not explain the rate loss under the tested P1 profile. P2 is clearly below target and has wall-time long tails consistent with provenance/recorder competition; GIL attribution is not proven by this checkpoint.

`processing_ms` retains the original tracker-processing semantics. `process_packet_wall_ms` includes the surrounding packet-processing wall time. In P0 the P95 gap is 2.04 ms and the maximum is 4.60 ms. In P2 the P95 gap remains 2.00 ms, but the maximum expands to 2697.13 ms because of the provenance second pass and associated work. These fields must not be mixed.

The current bottleneck decision is therefore: keep P1 as the default field profile, prohibit P2 for formal real-time acceptance, and continue with profiling/diagnostic overhead isolation before considering a dedicated seam-ROI topic or independent process. No weld algorithm, YAML, ROI, reference, gate, DP, IRLS, or temporal behavior was tuned or changed.

## CameraInfo and acceptance boundary

CameraInfo was received, but its `K/P` calibration matrices are zero. Camera calibration is not complete; only pixel-level tracking evidence is valid at this checkpoint. This is explicitly not production-ready.

## Validation record

The following checks passed after the final source changes:

```text
pytest: 16 passed
python -m compileall -q tank_vision scripts: passed
bash -n scripts/run_field_visual_e2e.sh scripts/stop_field_visual_e2e.sh scripts/run_field_w3_perf_isolation.sh: passed
git diff --check: passed
tank_vision ROS colcon build: 4 packages finished
tank_vision ROS colcon test: passed
```

The separate `physic_bev` full build remains blocked by its pre-existing `inspection_interfaces` symlink-install directory conflict; no clean/reset/delete operation was used. The relevant camera-driver rebuild and cross-workspace launch/contract checks passed.
