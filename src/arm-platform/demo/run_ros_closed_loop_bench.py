#!/usr/bin/env python3
"""Run LL100-LH420 ROS closed-loop NMPC bench configurations.

This script launches the plant node and the demo NMPC node as subprocesses,
waits until the plant observes the first command, samples for the requested
duration, and archives latency/plant CSVs for offline evaluation.
"""

import argparse
import csv
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
PLANT_SCRIPT = SCRIPT_DIR / "ll100_lh420_plant_node.py"
DEMO_SCRIPT = SCRIPT_DIR / "move_ll100_lh420_nmpc_demo.py"
DEMO_DATA_DIR = SCRIPT_DIR / "data"


@dataclass(frozen=True)
class BenchConfig:
    name: str
    path_mode: str
    params: Dict[str, object]


CONFIGS: Tuple[BenchConfig, ...] = (
    BenchConfig("default_lower", "lower_obstacle_circle", {}),
    BenchConfig("default_circle", "circle", {}),
    BenchConfig("default_front", "front_wing_arc", {}),
    BenchConfig(
        "tuned_lower",
        "lower_obstacle_circle",
        {
            "acados_rti_iterations": 3,
            "horizon_steps": 6,
            "position_weight": 180.0,
            "terminal_position_weight": 1400.0,
            "joint_delta_weight": 0.16,
            "joint_delta_change_weight": 0.084,
            "current_posture_weight": 0.014,
            "center_posture_weight": 0.0043,
        },
    ),
    BenchConfig(
        "tuned_circle",
        "circle",
        {
            "acados_rti_iterations": 3,
            "horizon_steps": 6,
            "position_weight": 252.0,
            "terminal_position_weight": 680.0,
            "joint_delta_weight": 0.224,
            "joint_delta_change_weight": 0.7,
            "current_posture_weight": 0.014,
            "center_posture_weight": 0.0035,
        },
    ),
    BenchConfig(
        "tuned_front",
        "front_wing_arc",
        {
            "acados_rti_iterations": 1,
            "horizon_steps": 8,
            "position_weight": 180.0,
            "terminal_position_weight": 1200.0,
            "joint_delta_weight": 0.08,
            "joint_delta_change_weight": 0.5,
            "current_posture_weight": 0.002,
            "center_posture_weight": 0.01,
        },
    ),
)


def _print(msg: str) -> None:
    print(msg, flush=True)


def _ros_value(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _ros_params(params: Dict[str, object]) -> List[str]:
    args: List[str] = ["--ros-args"]
    for key, value in params.items():
        args.extend(["-p", f"{key}:={_ros_value(value)}"])
    return args


def _taskset_prefix(cpu_list: str) -> Tuple[List[str], str]:
    taskset = shutil.which("taskset")
    if taskset is None:
        return [], "taskset not found"
    try:
        probe = subprocess.run(
            [taskset, "-c", cpu_list, "true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3.0,
            check=False,
        )
    except Exception as exc:
        return [], f"taskset probe failed for {cpu_list}: {exc}"
    if probe.returncode != 0:
        reason = probe.stderr.strip() or f"returncode={probe.returncode}"
        return [], f"taskset unavailable for {cpu_list}: {reason}"
    return [taskset, "-c", cpu_list], f"taskset -c {cpu_list}"


def _chrt_prefix() -> Tuple[List[str], str]:
    chrt = shutil.which("chrt")
    if chrt is None:
        return [], "chrt not found"
    try:
        probe = subprocess.run(
            [chrt, "-f", "80", "true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3.0,
            check=False,
        )
    except Exception as exc:
        return [], f"chrt probe failed: {exc}"
    if probe.returncode != 0:
        reason = probe.stderr.strip() or f"returncode={probe.returncode}"
        return [], f"chrt unavailable: {reason}"
    return [chrt, "-f", "80"], "chrt -f 80"


def _open_log(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.open("w", buffering=1)


def _start_process(cmd: List[str], env: Dict[str, str], stdout_path: Path, stderr_path: Path) -> Tuple[subprocess.Popen, object, object]:
    stdout_f = _open_log(stdout_path)
    stderr_f = _open_log(stderr_path)
    proc = subprocess.Popen(
        cmd,
        cwd=str(SCRIPT_DIR),
        env=env,
        stdout=stdout_f,
        stderr=stderr_f,
        start_new_session=True,
        text=True,
    )
    return proc, stdout_f, stderr_f


def _stop_process(proc: Optional[subprocess.Popen], name: str, timeout_sec: float = 20.0) -> Optional[int]:
    if proc is None:
        return None
    if proc.poll() is not None:
        return int(proc.returncode)
    _print(f"Stopping {name} with SIGINT")
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    except Exception:
        proc.send_signal(signal.SIGINT)
    try:
        return int(proc.wait(timeout=timeout_sec))
    except subprocess.TimeoutExpired:
        _print(f"{name} did not exit after SIGINT; terminating")
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except Exception:
            proc.terminate()
        try:
            return int(proc.wait(timeout=5.0))
        except subprocess.TimeoutExpired:
            _print(f"{name} did not exit after SIGTERM; killing")
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except Exception:
                proc.kill()
            return int(proc.wait(timeout=5.0))


def _close_logs(handles: Iterable[object]) -> None:
    for handle in handles:
        try:
            handle.flush()
            handle.close()
        except Exception:
            pass


def _latency_snapshot() -> Dict[Path, Tuple[int, int]]:
    DEMO_DATA_DIR.mkdir(parents=True, exist_ok=True)
    snapshot: Dict[Path, Tuple[int, int]] = {}
    for path in DEMO_DATA_DIR.glob("latency_*.csv"):
        try:
            st = path.stat()
            snapshot[path] = (st.st_mtime_ns, st.st_size)
        except FileNotFoundError:
            continue
    return snapshot


def _archive_latency_csv(before: Dict[Path, Tuple[int, int]], dest_dir: Path) -> List[str]:
    copied: List[str] = []
    candidates: List[Path] = []
    for path in DEMO_DATA_DIR.glob("latency_*.csv"):
        try:
            st = path.stat()
        except FileNotFoundError:
            continue
        old = before.get(path)
        if old is None or old != (st.st_mtime_ns, st.st_size):
            candidates.append(path)
    candidates.sort(key=lambda p: p.stat().st_mtime_ns)
    for i, src in enumerate(candidates):
        dest = dest_dir / ("latency.csv" if i == len(candidates) - 1 else f"latency_{i + 1}.csv")
        shutil.copy2(src, dest)
        copied.append(str(dest))
    return copied


def _first_command_time(record_path: Path) -> Optional[float]:
    if not record_path.exists():
        return None
    try:
        with record_path.open(newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    if int(float(row.get("command_seq", "0"))) > 0:
                        return float(row["t_sec"])
                except (KeyError, TypeError, ValueError):
                    continue
    except OSError:
        return None
    return None


def _select_configs(raw: Optional[List[str]]) -> List[BenchConfig]:
    if not raw:
        return list(CONFIGS)
    requested: List[str] = []
    for item in raw:
        requested.extend(part for part in item.split(",") if part)
    by_name = {cfg.name: cfg for cfg in CONFIGS}
    unknown = [name for name in requested if name not in by_name]
    if unknown:
        raise ValueError(f"Unknown configs: {unknown}; available={list(by_name)}")
    return [by_name[name] for name in requested]


def _write_json(path: Path, data: Dict[str, object]) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def _run_one(
    cfg: BenchConfig,
    index: int,
    output_root: Path,
    duration_sec: float,
    start_timeout_sec: float,
    max_velocity: float,
) -> Dict[str, object]:
    run_dir = output_root / cfg.name
    run_dir.mkdir(parents=True, exist_ok=True)
    plant_csv = run_dir / "plant.csv"
    domain_id = 90 + index
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)

    latency_before = _latency_snapshot()
    taskset_plant, taskset_plant_note = _taskset_prefix("2-3")
    taskset_demo, taskset_demo_note = _taskset_prefix("4-7")
    chrt_demo, chrt_note = _chrt_prefix()

    plant_params = {
        "publish_rate": 50.0,
        "max_velocity": max_velocity,
        "record_path": str(plant_csv),
    }
    demo_params: Dict[str, object] = {
        "latency_profiling": True,
        "marker_publish_hz": 0.0,
        "path_mode": cfg.path_mode,
    }
    demo_params.update(cfg.params)

    plant_cmd = [
        *taskset_plant,
        sys.executable,
        str(PLANT_SCRIPT),
        *_ros_params(plant_params),
    ]
    demo_cmd = [
        *chrt_demo,
        *taskset_demo,
        sys.executable,
        str(DEMO_SCRIPT),
        *_ros_params(demo_params),
    ]

    meta: Dict[str, object] = {
        "config_name": cfg.name,
        "path_mode": cfg.path_mode,
        "config_params": cfg.params,
        "demo_params": demo_params,
        "plant_params": plant_params,
        "duration_sec": duration_sec,
        "start_timeout_sec": start_timeout_sec,
        "ros_domain_id": domain_id,
        "plant_cmd": plant_cmd,
        "demo_cmd": demo_cmd,
        "prefix_notes": {
            "plant_taskset": taskset_plant_note,
            "demo_taskset": taskset_demo_note,
            "demo_chrt": chrt_note,
        },
        "status": "unknown",
        "failure_reason": "",
        "first_command_t_sec": None,
        "startup_wait_wall_sec": None,
        "latency_csvs": [],
        "plant_returncode": None,
        "demo_returncode": None,
    }

    plant_proc = None
    demo_proc = None
    log_handles: List[object] = []
    start_wall = time.monotonic()
    try:
        _print(f"[{cfg.name}] ROS_DOMAIN_ID={domain_id} starting plant")
        plant_proc, plant_out, plant_err = _start_process(
            plant_cmd, env, run_dir / "plant.stdout.log", run_dir / "plant.stderr.log"
        )
        log_handles.extend([plant_out, plant_err])
        time.sleep(1.0)
        if plant_proc.poll() is not None:
            raise RuntimeError(f"plant exited before demo start: rc={plant_proc.returncode}")

        _print(f"[{cfg.name}] starting NMPC demo")
        demo_proc, demo_out, demo_err = _start_process(
            demo_cmd, env, run_dir / "nmpc.stdout.log", run_dir / "nmpc.stderr.log"
        )
        log_handles.extend([demo_out, demo_err])

        _print(f"[{cfg.name}] waiting for first command, timeout={start_timeout_sec:.0f}s")
        first_command_t = None
        last_notice = time.monotonic()
        while time.monotonic() - start_wall < start_timeout_sec:
            if plant_proc.poll() is not None:
                raise RuntimeError(f"plant exited while waiting for first command: rc={plant_proc.returncode}")
            if demo_proc.poll() is not None:
                raise RuntimeError(f"NMPC exited before first command: rc={demo_proc.returncode}")
            first_command_t = _first_command_time(plant_csv)
            if first_command_t is not None:
                break
            if time.monotonic() - last_notice > 30.0:
                _print(f"[{cfg.name}] still waiting for first command...")
                last_notice = time.monotonic()
            time.sleep(0.5)
        if first_command_t is None:
            raise TimeoutError(f"no command observed within {start_timeout_sec:.0f}s")

        sample_start_wall = time.monotonic()
        meta["first_command_t_sec"] = first_command_t
        meta["startup_wait_wall_sec"] = sample_start_wall - start_wall
        _print(
            f"[{cfg.name}] first command at plant t={first_command_t:.3f}s; "
            f"sampling {duration_sec:.1f}s"
        )
        while time.monotonic() - sample_start_wall < duration_sec:
            if plant_proc.poll() is not None:
                raise RuntimeError(f"plant exited during sampling: rc={plant_proc.returncode}")
            if demo_proc.poll() is not None:
                raise RuntimeError(f"NMPC exited during sampling: rc={demo_proc.returncode}")
            time.sleep(0.5)
        meta["status"] = "ok"
    except Exception as exc:
        meta["status"] = "failed"
        meta["failure_reason"] = str(exc)
        meta["failure_traceback"] = traceback.format_exc()
        (run_dir / "FAILED").write_text(meta["failure_traceback"], encoding="utf-8")
        _print(f"[{cfg.name}] FAILED: {exc}")
    finally:
        meta["demo_returncode"] = _stop_process(demo_proc, "NMPC demo")
        meta["plant_returncode"] = _stop_process(plant_proc, "plant")
        _close_logs(log_handles)
        try:
            meta["latency_csvs"] = _archive_latency_csv(latency_before, run_dir)
        except Exception as exc:
            meta["latency_archive_error"] = str(exc)
        _write_json(run_dir / "run_meta.json", meta)
    return meta


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configs", nargs="*", help="Config names or comma-separated subsets")
    parser.add_argument("--duration", type=float, default=90.0, help="Sampling seconds after first command")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEMO_DATA_DIR / f"ros_closed_loop_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    parser.add_argument(
        "--start-timeout",
        type=float,
        default=900.0,
        help="Seconds to wait for first command; tuned_front horizon=8 may rebuild acados.",
    )
    parser.add_argument("--max-velocity", type=float, default=0.5, help="Plant max_velocity rad/s")
    args = parser.parse_args()

    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    try:
        selected = _select_configs(args.configs)
        _print(f"Output root: {output_root}")
        all_meta = []
        for index, cfg in enumerate(selected):
            all_meta.append(
                _run_one(
                    cfg=cfg,
                    index=index,
                    output_root=output_root,
                    duration_sec=float(args.duration),
                    start_timeout_sec=float(args.start_timeout),
                    max_velocity=float(args.max_velocity),
                )
            )
        _write_json(output_root / "run_index.json", {"runs": all_meta})
        (output_root / "ALL.DONE").write_text(datetime.now().isoformat() + "\n", encoding="utf-8")
        _print("ALL.DONE")
        return 0
    except Exception:
        tb = traceback.format_exc()
        (output_root / "FAILED").write_text(tb, encoding="utf-8")
        _print(tb)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
