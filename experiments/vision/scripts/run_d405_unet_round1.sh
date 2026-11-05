#!/usr/bin/env bash
# Replay the frozen first-round protocol into a NEW output directory.
set -euo pipefail
prototype_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$prototype_repo"
prototype_output="${1:?Usage: bash run_d405_unet_round1.sh NEW_OUTPUT [ASSETS] [SOURCE_INDEX] [LABELS]}"
prototype_assets="${2:-.runtime/s100p/analysis/d405_unet_round1_20261102/assets}"
prototype_source="${3:-/media/mcap_data/vision_experiments/VISION-20261102_162137-a9150e48/index.json}"
prototype_labels="${4:-d405-human-labels.json}"
prototype_python="$prototype_repo/.runtime/s100p/d405_prototype_venv/bin/python"
if [[ -e "$prototype_output" ]]; then
  echo "Output already exists; choose a new directory: $prototype_output" >&2
  exit 2
fi
if [[ ! -x "$prototype_python" ]]; then
  echo 'Create the isolated prototype environment using the validation report first.' >&2
  exit 2
fi
source deploy/env.sh s100p
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
mkdir -p "$prototype_output/assets"
cp "$prototype_assets/mobilenet_v2.imagenet.safetensors" "$prototype_output/assets/"
cp "$prototype_assets/hf_tree.json" "$prototype_output/assets/"
cp "$prototype_assets/hf_model_metadata.json" "$prototype_output/assets/"
"$prototype_python" experiments/vision/scripts/d405_unet_round1.py prepare \
  --source-index "$prototype_source" --labels "$prototype_labels" --output "$prototype_output" \
  | tee "$prototype_output/prepare.log"
"$prototype_python" experiments/vision/scripts/d405_unet_round1.py run --output "$prototype_output" \
  | tee "$prototype_output/train.log"
"$prototype_python" experiments/vision/scripts/d405_unet_round1.py report --output "$prototype_output" \
  | tee "$prototype_output/report.log"
