# Jetson MAXN_SUPER NMPC latency baseline - 2026-07-06

This directory contains the ROS end-to-end NMPC latency baseline measured on the
Jetson Orin NX Super after switching to `MAXN_SUPER` and locking clocks.

## Test Environment

- Board: NVIDIA Jetson Orin NX Engineering Reference Developer Kit Super
- JetPack/L4T: JetPack 6.2 / L4T 36.4.3
- Power mode: `MAXN_SUPER`
- Clocks: `jetson_clocks` enabled; CPU MinFreq/MaxFreq were both `1984000`
- Fan: `jetson_clocks --fan`
- ROS workspace: `/home/nvidia/code/windylab_ws`
- Environment:
  - `source /opt/ros/humble/setup.bash`
  - `source install/setup.bash`
  - `source src/arm-platform/scripts/setup_acados_env.sh`
  - `src/arm-platform/scripts/check_nmpc_env.sh` passed with 0 failures and 0 warnings
- RT launch: Codex was not running in a PAM login session, so each NMPC run used
  `sudo prlimit --rtprio=95:95 --pid $$ && chrt -f 80 taskset -c 4-7 ...` in
  the same shell command.
- Feedback source: `ll100_lh420_zero_joint_state_publisher.py` at 50 Hz.
- Latency profiling: 5 s warmup; CSV columns are seconds; summary converts to ms.
- Desktop session: left running; `systemctl isolate multi-user.target` was not used.

## Matrix

Scenarios:

- `lower_obstacle_circle`
- `circle`
- `front_wing_arc`

Backends:

- `qpoases`: `solver_backend:=acados`, `acados_qp_solver:=FULL_CONDENSING_QPOASES`
- `hpipm`: `solver_backend:=acados`, `acados_qp_solver:=PARTIAL_CONDENSING_HPIPM`
- `grampc`: `solver_backend:=grampc`

## Files

- `latency_summary_maxn_super.csv`: summary table with the same columns as the
  40 W baseline summary.
- `latency_<scenario>_<backend>.csv`: raw post-warmup latency samples for each
  scenario/backend pair.
- `logs/`: local run logs and temperature traces kept in the working tree; these
  are not intended as the primary benchmark artifact.

## Notes

- Temperature stayed well below thermal throttling range:
  - end `tj-thermal`: `56031` milli-Celsius
  - max logged `tj-thermal`: `57156` milli-Celsius
  - samples above 85 C: `0`
- The first acados launch for a new generated solver spent a long time in C code
  generation/compilation. Final CSVs were kept only after at least 1500
  post-warmup samples were available.
- `lower_obstacle_circle/qpoases` has 7994 samples because the first valid run was
  stopped later than the others while tuning the benchmark harness.
- Some node logs show shutdown tracebacks around Ctrl-C; the CSV files were
  flushed and the summary was computed from the CSV contents.
- A few max values include startup or scheduling outliers, especially HPIPM on
  `lower_obstacle_circle`. Use p95 together with max when comparing baselines.
