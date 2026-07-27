#!/bin/bash
# 单发布者、进程组隔离、时钟污染修复后的 moving_base_circle 单轮实验。
# 用法: run_ab_experiment.sh <label> <lead_steps> <out_dir> <dq_prev> <rti> <vff> <guard> [horizon] [orientation_scale] [posture_reference_mode] [solver_model] [trajectory_time_scale] [base_motion_time_scale]
# 关键:控制开始前用 collect_provenance.py 采集 SHA/git/param(避免 demo timer 阻塞);
#       每组件 setsid 独立进程组;预检 publisher==1;kill -PGID 收尾不残留。
LABEL=$1
LEAD=$2
OUT=$3
MODE=$4        # dq_prev_mode: zero | measured_delta        (**必填**,实验变量必须显式写出)
RTI_MODE=$5    # rti_iteration_mode: legacy_early_exit | fixed_iterations (**必填**)
VFF_GAIN=$6    # velocity_feedforward_gain: 0.7 | 1.0        (**必填**,阶段1D 成对传给 actual+plant model)
GUARD_MODE=$7  # guard_mode: legacy_fixed_step | expected_shadow | expected_residual (**必填**,阶段1E 实验变量)
HORIZON=${8:-2} # 阶段2A实验变量;省略时保持历史行为 h=2
ORI_SCALE=${9:-0.25} # 相对旧 ow/tow 的比例;0.25 为阶段2C生产值
POSTURE_MODE=${10:-reset_nominal} # 阶段3生产值;current 仅回归对照
SOLVER_MODEL=${11:-first_order} # first_order | second_order | shadow
TRAJECTORY_TIME_SCALE=${12:-1.0} # >1 表示降低画图速度
BASE_MOTION_TIME_SCALE=${13:-1.0} # >1 表示降低基座运动速度
SECOND_ORDER_HORIZON_STEPS=${SECOND_ORDER_HORIZON_STEPS:-10}
SECOND_ORDER_POSITION_WEIGHT=${SECOND_ORDER_POSITION_WEIGHT:-50.0}
SECOND_ORDER_TERMINAL_POSITION_WEIGHT=${SECOND_ORDER_TERMINAL_POSITION_WEIGHT:-500.0}
SECOND_ORDER_VELOCITY_WEIGHT=${SECOND_ORDER_VELOCITY_WEIGHT:-0.02}
SECOND_ORDER_ACCELERATION_WEIGHT=${SECOND_ORDER_ACCELERATION_WEIGHT:-0.01}
SECOND_ORDER_DQ_FF_COMPENSATION=${SECOND_ORDER_DQ_FF_COMPENSATION:-1.0}
SECOND_ORDER_REFERENCE_LEAD=${SECOND_ORDER_REFERENCE_LEAD:--1}
SECOND_ORDER_STATE_PREDICTION=${SECOND_ORDER_STATE_PREDICTION:-false}
SECOND_ORDER_VELOCITY_SOURCE=${SECOND_ORDER_VELOCITY_SOURCE:-measured}
SECOND_ORDER_MAX_VELOCITY=${SECOND_ORDER_MAX_VELOCITY:-2.0}
SECOND_ORDER_MAX_ACCELERATION=${SECOND_ORDER_MAX_ACCELERATION:-15.0}
CONTROL_RATE_HZ=${CONTROL_RATE_HZ:-50.0}
MAX_JOINT_STEP=${MAX_JOINT_STEP:-0.04}
PLANT_MAX_VELOCITY=${PLANT_MAX_VELOCITY:-3.0}
PLANT_MAX_ACCELERATION=${PLANT_MAX_ACCELERATION:-20.0}
PLANT_POSITION_FEEDBACK_TAU=${PLANT_POSITION_FEEDBACK_TAU:-0.15}
PLANT_FEEDBACK_VELOCITY_LIMIT=${PLANT_FEEDBACK_VELOCITY_LIMIT:-0.4}
# 实验变量必填,缺省即报错退出,避免后续实验忘传参数意外退回 baseline,并强制复现命令显式写出变量。
if [ -z "$LABEL" ] || [ -z "$LEAD" ] || [ -z "$OUT" ] || [ -z "$MODE" ] || [ -z "$RTI_MODE" ] || [ -z "$VFF_GAIN" ] || [ -z "$GUARD_MODE" ]; then
  echo "usage: run_ab_experiment.sh <label> <lead_steps> <out_dir> <dq_prev_mode> <rti_iteration_mode> <velocity_feedforward_gain> <guard_mode> [horizon_steps] [orientation_scale] [posture_reference_mode] [solver_model] [trajectory_time_scale] [base_motion_time_scale]"
  echo "  dq_prev_mode              : zero | measured_delta"
  echo "  rti_iteration_mode        : legacy_early_exit | fixed_iterations"
  echo "  velocity_feedforward_gain : 0.7 | 1.0   (阶段1D 只允许这两个值,防误传)"
  echo "  guard_mode                : legacy_fixed_step | expected_shadow | expected_residual (阶段1E)"
  echo "  (以上实验变量为必填,不提供即退出)"
  exit 2
fi
case "$MODE" in zero|measured_delta) ;; *) echo "非法 dq_prev_mode: $MODE"; exit 2;; esac
case "$RTI_MODE" in legacy_early_exit|fixed_iterations) ;; *) echo "非法 rti_iteration_mode: $RTI_MODE"; exit 2;; esac
# velocity_feedforward_gain 校验:非有限/负数/非 0.7|1.0 立即退出(本阶段只允许 0.7 与 1.0)。
case "$VFF_GAIN" in 0.7|1.0) ;; *) echo "非法 velocity_feedforward_gain: $VFF_GAIN(本阶段只允许 0.7|1.0)"; exit 2;; esac
case "$GUARD_MODE" in legacy_fixed_step|expected_shadow|expected_residual) ;; *) echo "非法 guard_mode: $GUARD_MODE"; exit 2;; esac
case "$HORIZON" in 2|4|6|8|12) ;; *) echo "非法 horizon_steps: $HORIZON (仅允许 2|4|6|8|12)"; exit 2;; esac
case "$ORI_SCALE" in
  1|1.0) ORI_WEIGHT=0.375; TERMINAL_ORI_WEIGHT=5.0;;
  0.5) ORI_WEIGHT=0.1875; TERMINAL_ORI_WEIGHT=2.5;;
  0.25) ORI_WEIGHT=0.09375; TERMINAL_ORI_WEIGHT=1.25;;
  *) echo "非法 orientation_scale: $ORI_SCALE (仅允许 1.0|0.5|0.25)"; exit 2;;
esac
case "$POSTURE_MODE" in current|reset_nominal) ;; *)
  echo "非法 posture_reference_mode: $POSTURE_MODE"; exit 2;; esac
case "$SOLVER_MODEL" in first_order|second_order|shadow) ;; *)
  echo "非法 solver_model: $SOLVER_MODEL"; exit 2;; esac
python3 - \
  "$TRAJECTORY_TIME_SCALE" "$BASE_MOTION_TIME_SCALE" \
  "$PLANT_MAX_VELOCITY" "$PLANT_MAX_ACCELERATION" \
  "$SECOND_ORDER_MAX_VELOCITY" "$SECOND_ORDER_MAX_ACCELERATION" \
  "$CONTROL_RATE_HZ" "$MAX_JOINT_STEP" <<'PY' || exit 2
import math
import sys
values = [float(x) for x in sys.argv[1:]]
if not all(math.isfinite(x) and x > 0.0 for x in values):
    raise SystemExit(1)
(_, _, plant_v, plant_a, solver_v, solver_a,
 control_rate, max_step) = values
if solver_v > plant_v or solver_a > plant_a:
    raise SystemExit(
        'solver velocity/acceleration limits must not exceed plant limits')
if max_step > solver_v / control_rate + 1e-12:
    raise SystemExit(
        'max_joint_step must not exceed second_order_max_velocity/control_rate')
PY
WS=/home/windylab/code/windylab_ws
DEMO_DIR=$WS/src/arm-platform/demo
DATA_DIR=$DEMO_DIR/data
TRACK_SEC=${TRACK_SEC:-45}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
source $WS/src/arm-platform/scripts/setup_acados_env.sh
source "$DEMO_DIR/setup_fastdds_udp_only.sh" "$ROS_DOMAIN_ID" || exit 2

PGIDS=()
cleanup() {
  for pg in "${PGIDS[@]}"; do kill -INT -"$pg" 2>/dev/null; done
  sleep 4
  for pg in "${PGIDS[@]}"; do kill -KILL -"$pg" 2>/dev/null; done
}
trap cleanup EXIT
echo "[run] === $LABEL lead=$LEAD dq_prev_mode=$MODE rti_mode=$RTI_MODE vff_gain=$VFF_GAIN guard_mode=$GUARD_MODE horizon=$HORIZON orientation_scale=$ORI_SCALE posture=$POSTURE_MODE solver=$SOLVER_MODEL trajectory_scale=$TRAJECTORY_TIME_SCALE base_scale=$BASE_MOTION_TIME_SCALE plant_v/a=$PLANT_MAX_VELOCITY/$PLANT_MAX_ACCELERATION solver_v/a=$SECOND_ORDER_MAX_VELOCITY/$SECOND_ORDER_MAX_ACCELERATION domain=$ROS_DOMAIN_ID ==="

setsid ros2 launch manipulator student_arm.launch.py use_rviz:=false \
  max_velocity:=$PLANT_MAX_VELOCITY velocity_feedforward_gain:=$VFF_GAIN \
  position_feedback_tau:=$PLANT_POSITION_FEEDBACK_TAU \
  feedback_velocity_limit:=$PLANT_FEEDBACK_VELOCITY_LIMIT \
  max_acceleration:=$PLANT_MAX_ACCELERATION \
  > "$OUT/launch_$LABEL.log" 2>&1 &
LAUNCH_PG=$!; PGIDS+=($LAUNCH_PG)

for i in $(seq 1 30); do
  timeout 2 ros2 topic echo /joint_states --once >/dev/null 2>&1 && { echo "[run] /joint_states after ${i}s"; break; }
  sleep 1
done
sleep 2
PUB=$(ros2 topic info /joint_states 2>/dev/null | grep -i "Publisher count" | grep -oE "[0-9]+")
echo "[run] preflight publisher count=${PUB:-?}"
[ "$PUB" != "1" ] && { echo "[run] ABORT publisher!=1"; ros2 topic info /joint_states -v 2>/dev/null | head -30; exit 2; }
SA=$(ros2 node list 2>/dev/null | grep -c "student_arm_node")
[ "$SA" != "1" ] && { echo "[run] ABORT student_arm_node count=$SA"; exit 2; }

cd "$DEMO_DIR"
setsid python3 trajectory_visualizer_demo.py --ros-args \
  -p trajectory_mode:=moving_base_circle -p frame_id:=world \
  -p actual_topic:=/joint_states -p command_topic:=/student/joint_command \
  -p record_command_fk:=true -p record_control_signal:=true \
  -p ignore_initial_sec:=3.0 -p show_matplotlib:=false \
  -p path_clock_mode:=required \
  -p trajectory_time_scale:=$TRAJECTORY_TIME_SCALE \
  -p base_motion_time_scale:=$BASE_MOTION_TIME_SCALE \
  > "$OUT/viz_$LABEL.log" 2>&1 &
VIZ_PG=$!; PGIDS+=($VIZ_PG)
sleep 3

# 控制开始前采集 provenance(节点已起);写入 data 目录,传路径给 demo
PROV="$DATA_DIR/provenance_${LABEL}.json"
python3 collect_provenance.py --out "$PROV" --label "$LABEL" \
  --urdf "$WS/install/manipulator/share/manipulator/arm.urdf" \
  > "$OUT/provenance_$LABEL.log" 2>&1
echo "[run] provenance -> $PROV"

setsid python3 move_base_circle_mpc_ik_demo.py --ros-args \
  -p reference_lead_steps:=$LEAD -p dq_prev_mode:=$MODE -p rti_iteration_mode:=$RTI_MODE \
  -p guard_mode:=$GUARD_MODE -p horizon_steps:=$HORIZON \
  -p orientation_weight:=$ORI_WEIGHT -p terminal_orientation_weight:=$TERMINAL_ORI_WEIGHT \
  -p posture_reference_mode:=$POSTURE_MODE \
  -p solver_model:=$SOLVER_MODEL \
  -p transport_delay_guard_enabled:=true \
  -p trajectory_time_scale:=$TRAJECTORY_TIME_SCALE \
  -p base_motion_time_scale:=$BASE_MOTION_TIME_SCALE \
  -p second_order_horizon_steps:=$SECOND_ORDER_HORIZON_STEPS \
  -p second_order_position_weight:=$SECOND_ORDER_POSITION_WEIGHT \
  -p second_order_terminal_position_weight:=$SECOND_ORDER_TERMINAL_POSITION_WEIGHT \
  -p second_order_velocity_weight:=$SECOND_ORDER_VELOCITY_WEIGHT \
  -p second_order_acceleration_weight:=$SECOND_ORDER_ACCELERATION_WEIGHT \
  -p second_order_dq_ff_compensation:=$SECOND_ORDER_DQ_FF_COMPENSATION \
  -p second_order_reference_lead_steps:=$SECOND_ORDER_REFERENCE_LEAD \
  -p second_order_state_prediction:=$SECOND_ORDER_STATE_PREDICTION \
  -p second_order_velocity_source:=$SECOND_ORDER_VELOCITY_SOURCE \
  -p second_order_max_velocity:=$SECOND_ORDER_MAX_VELOCITY \
  -p second_order_max_acceleration:=$SECOND_ORDER_MAX_ACCELERATION \
  -p plant_velocity_feedforward_gain:=$VFF_GAIN \
  -p plant_position_feedback_tau:=$PLANT_POSITION_FEEDBACK_TAU \
  -p plant_feedback_velocity_limit:=$PLANT_FEEDBACK_VELOCITY_LIMIT \
  -p plant_max_velocity:=$PLANT_MAX_VELOCITY \
  -p plant_max_acceleration:=$PLANT_MAX_ACCELERATION \
  -p control_rate_hz:=$CONTROL_RATE_HZ -p max_joint_step:=$MAX_JOINT_STEP \
  -p record_loop_csv:=true -p loop_data_dir:=$DATA_DIR -p provenance_json:=$PROV \
  > "$OUT/demo_$LABEL.log" 2>&1 &
DEMO_PG=$!; PGIDS+=($DEMO_PG)

STARTED=0
for i in $(seq 1 150); do
  grep -q "开始 MPC 轨迹跟踪" "$OUT/demo_$LABEL.log" 2>/dev/null && { STARTED=1; echo "[run] tracking after ${i}s"; break; }
  kill -0 -$DEMO_PG 2>/dev/null || { echo "[run] ERROR demo died"; tail -20 "$OUT/demo_$LABEL.log"; exit 1; }
  sleep 1
done
[ $STARTED -ne 1 ] && { echo "[run] ERROR track not started"; tail -20 "$OUT/demo_$LABEL.log"; exit 1; }

ros2 topic info /joint_states -v > "$OUT/jointstates_info_$LABEL.txt" 2>&1
ros2 node list > "$OUT/nodes_$LABEL.txt" 2>&1

sleep $TRACK_SEC
kill -INT -$VIZ_PG; sleep 4
kill -INT -$DEMO_PG; sleep 3
kill -INT -$LAUNCH_PG; sleep 4
trap - EXIT; cleanup
echo "[run] === $LABEL done ==="
