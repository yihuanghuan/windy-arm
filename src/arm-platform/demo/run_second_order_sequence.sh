#!/bin/bash
# Shadow first, then low-speed first-order/second-order A/B.
set -euo pipefail

OUT=${1:-/tmp/second_order_ab}
TRAJECTORY_SCALE=${2:-2.0}
BASE_SCALE=${3:-4.0}
TRACK_SEC=${TRACK_SEC:-24}
export TRACK_SEC
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}

DEMO_DIR=/home/windylab/code/windylab_ws/src/arm-platform/demo
DATA_DIR=$DEMO_DIR/data
STAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
cd "$DEMO_DIR"

run_one() {
  local label=$1
  local model=$2
  local before
  before=$(date +%s)
  ./run_ab_experiment.sh "$label" 1 "$OUT/$label" measured_delta \
    fixed_iterations 0.7 legacy_fixed_step 2 0.25 reset_nominal \
    "$model" "$TRAJECTORY_SCALE" "$BASE_SCALE"
  local loop
  loop=$(find "$DATA_DIR" -maxdepth 1 -name 'moving_base_mpc_loop_*.csv' \
    -newermt "@$before" -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)
  if [ -z "$loop" ]; then
    echo "未找到 $label 的 loop CSV" >&2
    return 1
  fi
  echo "$loop"
}

SHADOW_LABEL="second_shadow_$STAMP"
SHADOW_LOOP=$(run_one "$SHADOW_LABEL" shadow | tail -1)
python3 analyze_second_order_shadow.py "$SHADOW_LOOP" \
  --out "$OUT/shadow_metrics.json"

if [ "${RUN_CONTROL_AB:-0}" != "1" ]; then
  echo "shadow 已完成；默认不自动控制。确认后以 RUN_CONTROL_AB=1 重跑低速 A/B。"
  exit 0
fi

FIRST_LABEL="first_low_$STAMP"
FIRST_LOOP=$(run_one "$FIRST_LABEL" first_order | tail -1)
FIRST_PARAMS=${FIRST_LOOP/moving_base_mpc_loop_/moving_base_mpc_params_}
FIRST_PARAMS=${FIRST_PARAMS%.csv}.json
python3 analyze_moving_base_run.py analyze --loop-csv "$FIRST_LOOP" \
  --params-json "$FIRST_PARAMS" --label "$FIRST_LABEL" \
  --out "$OUT/first_order_metrics.json"

SECOND_LABEL="second_low_$STAMP"
SECOND_LOOP=$(run_one "$SECOND_LABEL" second_order | tail -1)
SECOND_PARAMS=${SECOND_LOOP/moving_base_mpc_loop_/moving_base_mpc_params_}
SECOND_PARAMS=${SECOND_PARAMS%.csv}.json
python3 analyze_moving_base_run.py analyze --loop-csv "$SECOND_LOOP" \
  --params-json "$SECOND_PARAMS" --label "$SECOND_LABEL" \
  --out "$OUT/second_order_metrics.json"

python3 compare_second_order_ab.py \
  "$OUT/first_order_metrics.json" "$OUT/second_order_metrics.json" \
  --out "$OUT/ab_comparison.json"
echo "结果目录: $OUT"
