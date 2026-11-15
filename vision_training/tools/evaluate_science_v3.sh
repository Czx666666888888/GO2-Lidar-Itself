#!/usr/bin/env bash
# Offline evaluation only. Requires completed V3 training; refuses existing outputs.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=vision_training/.venv/bin/python
RUN=vision_training/runs/science_targets_mixed_v3
REPORT=vision_training/reports/science_targets_mixed_v3
DATA=vision_training/datasets/science_targets_mixed_v3
[[ -f "$RUN/baseline/training_result.json" ]]
taskset -c 16-19 "$PY" vision_training/tools/evaluate_science_mixed_v1.py \
 --dataset "$DATA" --weights "$RUN/baseline/weights/best.pt" \
 --output "$RUN/evaluation_640" --imgsz 640 --groups synthetic_val phone_val
"$PY" vision_training/tools/compare_science_v3.py \
 --v3 "$RUN/evaluation_640/metrics.json" --output "$REPORT/comparison.json"
taskset -c 16-19 "$PY" vision_training/tools/analyze_go2_continuity_v3.py predict \
 --weights "$RUN/baseline/weights/best.pt" --output "$RUN/go2_prediction_v3"
