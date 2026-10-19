# Field data role handoff guide

`build_role_handoff.py` creates role-specific delivery definitions for the sealed run
`FIELD-VEHICLE-20260928_115534`. Its default is manifest-only: MCAP, MKV and RRD payloads
are not copied unless both `--materialize` and `--destination` are supplied.

## Profiles

- `common`: provenance, configuration and validation reports.
- `fastlivo2`: MID360 MCAP, actual FAST-LIVO2 configuration/source metadata and Livox interfaces.
- `front_camera`: all five MKV segments, frame sidecars, mux logs and vision contracts.
- `d405`: complete D405 MCAP directory, logs and RealSense message sources.
- `eddy`: complete Eddy MCAP directory, protocol documentation and message sources.
- `rerun_viewer`: only the RRD, its hash, manifest, validation and conversion log.
- `system_integration`: core MCAP, timing/status reports and replay scripts.

Every MCAP selector expands the complete directory, including `metadata.yaml`. Video selection
expands every MKV and matching `frames.jsonl`/`mux.log`. `.partial` is audited and is never
selected. Project `install/` binaries and symlinks are never delivered.

## Safe default

```bash
cd /home/sunrise/east_eleair/physic_bev
/usr/bin/python3 scripts/build_role_handoff.py \
  --run-dir /media/mcap_data/runs/FIELD-VEHICLE-20260928_115534 \
  --profiles all \
  --output-root output/field_data_role_handoff_v1 \
  --manifest-only --audit
```

This creates audit JSON, per-profile source manifests, expected sizes, dependency notes, recipient
usage documents and export commands. It does not copy source payloads.

## Preview and materialize

```bash
/usr/bin/python3 scripts/build_role_handoff.py \
  --run-dir /media/mcap_data/runs/FIELD-VEHICLE-20260928_115534 \
  --profile fastlivo2 \
  --destination /media/RECIPIENT_DISK/FIELD-VEHICLE-20260928_115534_FASTLIVO2 \
  --materialize --dry-run

/usr/bin/python3 scripts/build_role_handoff.py \
  --run-dir /media/mcap_data/runs/FIELD-VEHICLE-20260928_115534 \
  --profile fastlivo2 \
  --destination /media/RECIPIENT_DISK/FIELD-VEHICLE-20260928_115534_FASTLIVO2 \
  --materialize --copy-method rsync --resume
```

An existing destination is rejected unless `--resume` is set. Resume validates each existing file
by size and SHA256 before reusing it. It never deletes a partial result. On failure,
`HANDOFF_STATUS.json` remains with `complete=false`.

## Verification

```bash
/usr/bin/python3 scripts/verify_role_handoff.py \
  --package-dir /media/RECIPIENT_DISK/FIELD-VEHICLE-20260928_115534_FASTLIVO2
```

A package is complete only when `SOURCE_FILE_MANIFEST.json`, `HANDOFF_STATUS.json` and
`SHA256SUMS` agree and every hash verifies. Returned algorithm results belong under `result/` or
`derived/<algorithm_name>/`; they must never overwrite the sealed source run.

## FAST-LIVO2 compatibility gate

The recorded `/livox/lidar` type is `sensor_msgs/msg/PointCloud2`. The current local FAST-LIVO2
MID360 parameter chooses type 1, whose source subscription is `livox_ros_driver2/msg/CustomMsg`.
The handoff is therefore an input and reproducibility package, not a claim that this exact mapping
configuration can consume the bag. The mapping owner must return a reviewed adapter or compatible
preprocessor/config snapshot and its SHA before mapping results can be accepted.
