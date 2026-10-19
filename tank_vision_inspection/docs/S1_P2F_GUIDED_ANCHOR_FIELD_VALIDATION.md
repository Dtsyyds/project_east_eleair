# S1-P2F Guided-anchor field validation

This is an independent `field-validation candidate`, not the production
default. The default direct launch remains on the original stable YAML.

The candidate uses the already validated P2E4-B state: geometry reference and
reference anchor enabled, anchor radius scale `0.12`, center transition weight
`0.42`, and all other candidate, pair, DP, fitting, gate, and temporal values
unchanged. No local step response or new unary feature is included.

## Three terminals

Terminal 1 — guided-anchor direct candidate:

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
ros2 launch tank_vision_ros \
  stable_seam_direct_guided_anchor.launch.py
```

Terminal 2 — viewer:

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
ros2 param set /stable_seam_node publish_debug_image true
QT_QPA_PLATFORM=xcb ros2 run tank_vision_ros seam_debug_viewer
```

When the target image is visible, press `S` to request a same-frame snapshot;
press `q` or `ESC` to exit.

Terminal 3 — state check:

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
ros2 topic hz /vision/front/seam_geometry
ros2 topic echo /vision/front/seam_geometry --once
```

Without a GUI, request a snapshot with:

```bash
ros2 service call \
  /stable_seam_node/save_snapshot \
  std_srvs/srv/Trigger "{}"
```

The direct node owns the MindVision camera. Do not start the original camera
driver concurrently.

## Required snapshot groups

Collect at least these eight groups, with the node stable for at least three
seconds before each snapshot:

`seam_center_static_01`, `seam_center_static_02`, `seam_left_offset_01`,
`seam_right_offset_01`, `seam_tilt_left_01`, `seam_tilt_right_01`,
`seam_partial_blur_01`, and `no_seam_01`.

Every snapshot must contain same-frame `raw`, `overlay`, `roi_raw`,
`roi_overlay`, `seam_geometry`, and `metadata`, with `complete=true` and hashes
matching the metadata. For each 10-second run record geometry frequency,
detected ratio, TRACKING ratio, LOST count, lateral range, heading range,
confidence range, and processing P50/P95/max.

Positive samples must receive manual annotation before precision is assessed;
an overlay alone is not an accuracy measurement.

## Field acceptance, provisional

This candidate is not production-approved. Provisional positive-sample targets
are detected rate `>=95%`, TRACKING rate `>=90%`, continuous LOST time
`<=200 ms`, zero crossing, zero visibly wrong trajectories, final fitted center
P95 `<=25 px`, final fitted left/right P95 `<=30 px`, and width P95 `<=30 px`.
The no-seam sample must remain `detected=false`, never enter TRACKING, and must
not obtain a valid trajectory from the reference alone.

Record actual geometry Hz and processing P95; do not change the processing
thread model in this validation stage. The known direct-mode serial path is
approximately 8 Hz. Producer/consumer changes are deferred until geometry
generalization is established.
