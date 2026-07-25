#!/bin/bash
# 阶段1E 单轮实验: run_ab_experiment.sh(固定 lead=1/measured_delta/fixed_iterations/gain=0.7,
# 唯一变量 guard_mode)+ 从本轮日志**显式提取**文件路径(不做"最新文件"猜测)+ analyze。
# 用法: run_phase1e_round.sh <label> <guard_mode: legacy_fixed_step|expected_shadow|expected_residual>
LABEL=$1
GMODE=$2
if [ -z "$LABEL" ] || [ -z "$GMODE" ]; then
  echo "usage: run_phase1e_round.sh <label> <guard_mode>"
  exit 2
fi
case "$GMODE" in legacy_fixed_step|expected_shadow|expected_residual) ;; *) echo "非法 guard_mode: $GMODE"; exit 2;; esac
WS=/home/windylab/code/windylab_ws
DEMO=$WS/src/arm-platform/demo
BASE=$DEMO/data/ab_runs_1e
OUT=$BASE/$LABEL
mkdir -p "$OUT"

bash "$DEMO/run_ab_experiment.sh" "$LABEL" 1 "$OUT" measured_delta fixed_iterations 0.7 "$GMODE" || exit 1

LOOP=$(grep -hoE '控制环诊断 CSV 开始记录: \S+' "$OUT/demo_$LABEL.log" | tail -1 | awk '{print $NF}')
PARAMS=$(grep -hoE '运行参数快照已保存: \S+' "$OUT/demo_$LABEL.log" | tail -1 | awk '{print $NF}')
TRAJ=$(grep -hoE '/[^ ;]*moving_base_circle_trajectory_[0-9_]+\.csv' "$OUT/viz_$LABEL.log" | tail -1 | sed 's/\.csv$//')
if [ -z "$LOOP" ] || [ -z "$PARAMS" ]; then
  echo "[1e] ERROR: 无法从 demo 日志提取 loop/params 路径"; exit 1
fi
echo "$LABEL guard_mode=$GMODE params=$PARAMS loop=$LOOP prov=$DEMO/data/provenance_${LABEL}.json traj=$TRAJ" >> "$BASE/manifest.txt"

source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
source $WS/src/arm-platform/scripts/setup_acados_env.sh
cd "$DEMO"
python3 analyze_moving_base_run.py analyze --loop-csv "$LOOP" --params-json "$PARAMS" \
  ${TRAJ:+--prefix "$TRAJ"} --label "$LABEL" --out "$BASE/metrics_$LABEL.json" \
  > "$BASE/analyze_$LABEL.log" 2>&1
RC=$?
tail -5 "$BASE/analyze_$LABEL.log"
echo "[1e] $LABEL done rc=$RC metrics=$BASE/metrics_$LABEL.json"
exit $RC
