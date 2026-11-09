# S1-P2B failure snapshot capture

`stable_seam_node` exposes `/stable_seam_node/save_snapshot` (`std_srvs/srv/Trigger`).
The service callback only queues a bounded one-shot request. The processing worker
copies a frame only when a request is present; a separate writer thread produces
the raw image, diagnostic overlay, ROI images, `seam_geometry.json`, and finally
`metadata.json` with `complete=true`. No raw image is published over DDS.

The default output root is
`/home/sunrise/east_eleair/tank_vision_inspection/output/stable_seam_failure_snapshots`.
Each frame gets a unique timestamp/stamp directory. `raw.bmp` is the copied BGR
input frame and can be passed back to `run_stable_seam_tracker.py`; every other
file records the same header stamp. A failed writer leaves `complete=false` and
an error in metadata.

The optional `seam_debug_viewer` listens for S/s, calls the service asynchronously,
and reports queued/rejected/unavailable status in the window and terminal. Q/ESC
and Ctrl+C exit through the executor's main loop; the subscription callback never
calls `rclpy.shutdown()`.

The feature is supported in both `ros_topic` and `mindvision_direct` modes. The
algorithm, ROI, reference, thresholds, camera settings, and existing topics are
unchanged. Real-camera acceptance remains pending until a MindVision device and
ROS transport are available.
