#!/usr/bin/env python3
"""Compare baseline, observer-only and observer+compensation CSV runs."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def _values(rows, key):
    out = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            out.append(value)
    return np.asarray(out, dtype=float)


def _stats(rows):
    out = {}
    for key in ("pos_error_m", "j5_qdot_motor_rad_s",
                "j5_leso_z3_raw_rad_s2", "j5_leso_z3_filtered_rad_s2",
                "j5_tau_smc_nm", "j5_tau_dist_nm", "j5_tau_final_nm",
                "j5_s_norm", "qdd_commanded_j5", "qdd_actual_est_j5",
                "acceleration_residual_j5", "j5_torque_saturation",
                "watchdog_flag"):
        v = _values(rows, key)
        if not v.size:
            out[key] = {"rms": None, "p95_abs": None, "max_abs": None}
            continue
        av = np.abs(v)
        out[key] = {
            "rms": float(np.sqrt(np.mean(v * v))),
            "p95_abs": float(np.percentile(av, 95)),
            "max_abs": float(np.max(av)),
        }
    torque = []
    for row in rows:
        for i in range(1, 7):
            try:
                value = float(row[f"joint_{i}_torque_nm"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value):
                torque.append(value)
    out["six_axis_torque_nm"] = {
        "rms": float(np.sqrt(np.mean(np.square(torque)))) if torque else None,
        "p95_abs": float(np.percentile(np.abs(torque), 95)) if torque else None,
        "max_abs": float(np.max(np.abs(torque))) if torque else None,
    }
    return out


def _read(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _initial_q(path):
    summary = path.with_name(
        path.name.replace("trajectory_", "summary_")).with_suffix(".json")
    try:
        data = json.loads(summary.read_text(encoding="utf-8"))
        return data.get("approach", {}).get("start_q_rad")
    except (OSError, json.JSONDecodeError):
        return None


def _summary(path):
    summary = path.with_name(
        path.name.replace("trajectory_", "summary_")).with_suffix(".json")
    try:
        return json.loads(summary.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _psd(rows, key, output, label):
    t = _values(rows, "t")
    y = _values(rows, key)
    if t.size != y.size or t.size < 32:
        return None
    dt = float(np.median(np.diff(t)))
    if not math.isfinite(dt) or dt <= 0.0:
        return None
    grid = np.arange(t[0], t[-1], dt)
    if grid.size < 32:
        return None
    y = np.interp(grid, t, y)
    y = y - np.mean(y)
    window = np.hanning(y.size)
    spectrum = np.fft.rfft(y * window)
    freq = np.fft.rfftfreq(y.size, dt)
    psd = np.abs(spectrum) ** 2 / (np.sum(window * window) * (1.0 / dt))
    valid = (freq >= 0.1) & (freq <= min(100.0, 0.45 / dt))
    if not np.any(valid):
        return None
    peak_idx = np.argsort(psd[valid])[-8:][::-1]
    vf, vp = freq[valid], psd[valid]
    peaks = [{"frequency_hz": float(vf[i]), "psd": float(vp[i])}
             for i in peak_idx]
    with open(output / f"{label}_{key}_psd.csv", "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["frequency_hz", "psd"])
        writer.writerows(zip(freq, psd))
    return peaks


def _uniform_raw(rows, key):
    t = _values(rows, "t")
    y = _values(rows, key)
    if t.size != y.size or t.size < 32:
        return None, None, None
    dt = float(np.median(np.diff(t)))
    if not math.isfinite(dt) or dt <= 0.0:
        return None, None, None
    grid = np.arange(t[0], t[-1], dt)
    if grid.size < 32:
        return None, None, None
    return grid, np.interp(grid, t, y), dt


def _uniform_columns(rows, keys):
    values = []
    for row in rows:
        try:
            item = [float(row[key]) for key in keys]
            t = float(row["t"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(t) and all(math.isfinite(v) for v in item):
            values.append((t, *item))
    if len(values) < 32:
        return None
    data = np.asarray(values, dtype=float)
    dt = float(np.median(np.diff(data[:, 0])))
    if not math.isfinite(dt) or dt <= 0.0:
        return None
    grid = np.arange(data[0, 0], data[-1, 0], dt)
    if grid.size < 32:
        return None
    # Column 0 is always time; columns 1..N are the requested signals.
    return grid, [np.interp(grid, data[:, 0], data[:, i + 1])
                  for i in range(len(keys))], dt


def _zero_phase_lowpass(values, dt, cutoff_hz):
    """Offline-only zero-phase low-pass; never used by the controller."""
    try:
        from scipy.signal import butter, sosfiltfilt
    except ImportError as exc:
        raise RuntimeError("scipy is required for independent acceleration diagnostics") from exc
    nyquist = 0.5 / dt
    normalized = min(0.99, float(cutoff_hz) / nyquist)
    sos = butter(4, normalized, btype="lowpass", output="sos")
    return sosfiltfilt(sos, np.asarray(values, dtype=float))


def _independent_motor_sign_check(rows, output, label):
    """Use measured motor velocity, not LESO z2, for the primary sign check."""
    data = _uniform_columns(rows, ["joint_5_qd_meas_rad_s", "qdd_commanded_j5",
                                   "j5_leso_z3_filtered_rad_s2"])
    if data is None:
        return {"available": False, "reason": "independent diagnostic columns unavailable"}
    grid, (motor_qdot, qdd_commanded, z3), dt = data
    motor_qdot_smooth = _zero_phase_lowpass(motor_qdot, dt, 5.0)
    qdd_actual = np.gradient(motor_qdot_smooth, dt)
    residual = qdd_actual - qdd_commanded

    def lowpass(values, cutoff):
        return _zero_phase_lowpass(values, dt, cutoff)

    result = {"available": True, "velocity_smoothing_hz": 5.0,
              "residual_definition": "qdd_actual_motor_j5 - qdd_commanded_j5"}
    plot_data = []
    for cutoff in (3.0, 5.0):
        z_low = lowpass(z3, cutoff)
        r_low = lowpass(residual, cutoff)
        valid = np.isfinite(z_low) & np.isfinite(r_low)
        z = z_low[valid] - np.mean(z_low[valid])
        r = r_low[valid] - np.mean(r_low[valid])
        max_lag = min(z.size // 4, int(round(2.0 / dt)))
        correlations = []
        lags = range(-max_lag, max_lag + 1)
        for lag in lags:
            if lag >= 0:
                a, b = z[:z.size - lag], r[lag:]
            else:
                a, b = z[-lag:], r[:r.size + lag]
            correlations.append(float(np.corrcoef(a, b)[0, 1])
                                if np.std(a) and np.std(b) else float("nan"))
        best = int(np.nanargmax(correlations))
        best_lag = best - max_lag
        result[f"low_{int(cutoff)}hz"] = {
            "pearson_zero_lag": float(np.corrcoef(z, r)[0, 1]),
            "best_lag_samples": best_lag,
            "best_lag_sec": float(best_lag * dt),
            "best_cross_correlation": float(correlations[best]),
            "sign_agreement_fraction": float(np.mean(z * r >= 0.0)),
        }
        plot_data.append((cutoff, z_low, r_low))

    try:
        with open(output / f"{label}_motor_acceleration_diagnostic.csv", "w",
                  newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time", "motor_qdot_j5", "motor_qdot_j5_smooth",
                             "qdd_actual_motor_j5", "qdd_commanded_j5",
                             "acceleration_residual_motor_j5",
                             "z3_filtered_j5"])
            writer.writerows(zip(grid, motor_qdot, motor_qdot_smooth,
                                 qdd_actual, qdd_commanded, residual, z3))
    except OSError:
        pass
    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        axes[0].plot(grid, z3, label="z3_filtered", linewidth=0.6)
        axes[0].plot(grid, residual, label="motor acceleration residual", linewidth=0.6)
        for cutoff, z_low, r_low in plot_data:
            axes[1].plot(grid, z_low, label=f"z3 {cutoff:g} Hz", linewidth=0.6)
            axes[1].plot(grid, r_low, label=f"residual {cutoff:g} Hz", linewidth=0.6)
        axes[0].legend()
        axes[1].legend(ncol=2)
        axes[1].set_xlabel("time (s)")
        axes[0].set_ylabel("rad/s^2")
        axes[1].set_ylabel("zero-phase low-pass")
        fig.tight_layout()
        fig.savefig(output / f"{label}_motor_acceleration_sign_check.png", dpi=160)
        plt.close(fig)
    except ImportError:
        pass
    return result


def _sign_check(rows, output, label):
    """Compare filtered z3 with diagnostic (actual - commanded) acceleration."""
    t = _values(rows, "t")
    z3 = _values(rows, "j5_leso_z3_filtered_rad_s2")
    residual = _values(rows, "acceleration_residual_j5")
    if t.size != z3.size or t.size != residual.size or t.size < 32:
        return {"available": False, "reason": "diagnostic columns unavailable"}
    dt = float(np.median(np.diff(t)))
    if not math.isfinite(dt) or dt <= 0.0:
        return {"available": False, "reason": "invalid timestamp spacing"}
    grid = np.arange(t[0], t[-1], dt)
    if grid.size < 32:
        return {"available": False, "reason": "insufficient samples"}
    z3 = np.interp(grid, t, z3)
    residual = np.interp(grid, t, residual)
    z3 = z3 - np.mean(z3)
    residual = residual - np.mean(residual)
    max_lag = min(grid.size // 4, int(round(2.0 / dt)))
    correlations = []
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            a, b = z3[:z3.size - lag], residual[lag:]
        else:
            a, b = z3[-lag:], residual[:residual.size + lag]
        correlations.append(float(np.corrcoef(a, b)[0, 1])
                            if np.std(a) and np.std(b) else float("nan"))
    if not np.any(np.isfinite(correlations)):
        return {"available": False, "reason": "signals have zero variance"}
    best = int(np.nanargmax(correlations))
    best_lag = best - max_lag
    if best_lag >= 0:
        a, b = z3[:z3.size - best_lag], residual[best_lag:]
    else:
        a, b = z3[-best_lag:], residual[:residual.size + best_lag]
    low = np.fft.rfft(np.hanning(z3.size) * z3)
    low_r = np.fft.rfft(np.hanning(residual.size) * residual)
    freq = np.fft.rfftfreq(z3.size, dt)
    low_mask = (freq >= 0.1) & (freq <= 3.0)
    if np.any(low_mask):
        z3_low = np.fft.irfft(low * low_mask, n=z3.size)
        res_low = np.fft.irfft(low_r * low_mask, n=residual.size)
    else:
        z3_low, res_low = z3, residual
    same_sign = float(np.mean(z3_low * res_low >= 0.0))
    fig_path = output / f"{label}_z3_sign_check.png"
    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 4))
        ax.plot(grid, z3, label="z3_filtered", linewidth=0.7)
        ax.plot(grid, residual, label="qdd_actual_est - qdd_commanded", linewidth=0.7)
        ax.set_xlabel("time (s)")
        ax.set_ylabel("rad/s^2")
        ax.legend()
        fig.tight_layout()
        fig.savefig(fig_path, dpi=160)
        plt.close(fig)
    except ImportError:
        fig_path = None
    return {
        "available": True,
        "pearson_zero_lag": float(np.corrcoef(z3, residual)[0, 1]),
        "best_lag_samples": best_lag,
        "best_lag_sec": float(best_lag * dt),
        "best_cross_correlation": float(correlations[best]),
        "low_frequency_same_sign_fraction_0_3hz": same_sign,
        "plot": str(fig_path) if fig_path else None,
    }


def _offline_lpf(rows, output, label):
    t, raw, dt = _uniform_raw(rows, "j5_leso_z3_raw_rad_s2")
    if t is None:
        return {}
    result = {}
    for cutoff in (3.0, 5.0, 8.0):
        filtered = np.zeros_like(raw)
        alpha = 1.0 - np.exp(-2.0 * np.pi * cutoff * dt)
        for i in range(1, raw.size):
            filtered[i] = filtered[i - 1] + alpha * (raw[i] - filtered[i - 1])
        av = np.abs(filtered)
        max_lag = min(filtered.size // 4, int(round(2.0 / dt)))
        correlations = []
        for lag in range(max_lag + 1):
            a, b = raw[:raw.size - lag], filtered[lag:]
            if np.std(a) and np.std(b):
                correlations.append(float(np.corrcoef(a, b)[0, 1]))
            else:
                correlations.append(float("nan"))
        lag = int(np.nanargmax(correlations)) if correlations else 0
        window = np.hanning(filtered.size)
        spec = np.fft.rfft((filtered - np.mean(filtered)) * window)
        freq = np.fft.rfftfreq(filtered.size, dt)
        psd = np.abs(spec) ** 2 / (np.sum(window * window) * (1.0 / dt))
        bands = {}
        for name, lo, hi in (("0_3", 0.0, 3.0), ("3_5", 3.0, 5.0),
                             ("5_8", 5.0, 8.0), ("8_20", 8.0, 20.0)):
            mask = (freq >= lo) & (freq < hi)
            bands[name] = float(np.trapz(psd[mask], freq[mask])) if np.any(mask) else 0.0
        result[str(int(cutoff))] = {
            "rms": float(np.sqrt(np.mean(filtered * filtered))),
            "p95_abs": float(np.percentile(av, 95)),
            "peak_abs": float(np.max(av)),
            "delay_samples": lag,
            "delay_sec": float(lag * dt),
            "raw_to_filtered_corr": correlations[lag] if correlations else None,
            "band_energy": bands,
        }
        with open(output / f"{label}_offline_lpf_{int(cutoff)}hz.csv", "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time", "z3_raw", "z3_filtered"])
            writer.writerows(zip(t, raw, filtered))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", required=True,
                        metavar="LABEL=CSV", help="repeat for each run")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--gate-reference", default=None,
                        help="label used for initial-q validity comparison")
    parser.add_argument("--initial-q-tolerance-rad", type=float, default=0.05)
    parser.add_argument("--position-rms-max-mm", type=float, default=10.0)
    parser.add_argument("--position-peak-max-mm", type=float, default=25.0)
    args = parser.parse_args()
    runs = {}
    for item in args.run:
        label, path = item.split("=", 1)
        runs[label] = (Path(path), _read(path))
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    report = {label: {"csv": str(path), "samples": len(rows),
                      "initial_joint_position_rad": _initial_q(path),
                      "summary": _summary(path),
                      "stats": _stats(rows)}
              for label, (path, rows) in runs.items()}
    for label, (_, rows) in runs.items():
        report[label]["psd_peaks"] = {}
        for key in ("j5_leso_z3_raw_rad_s2", "j5_leso_z3_filtered_rad_s2"):
            peaks = _psd(rows, key, output, label)
            if peaks is not None:
                report[label]["psd_peaks"][key] = peaks
        report[label]["offline_lpf_from_raw"] = _offline_lpf(
            rows, output, label)
        report[label]["z3_sign_check"] = _sign_check(rows, output, label)
        report[label]["motor_acceleration_sign_check"] = (
            _independent_motor_sign_check(rows, output, label))
    if args.gate_reference in report:
        reference_q = report[args.gate_reference]["initial_joint_position_rad"]
        for label, item in report.items():
            q = item["initial_joint_position_rad"]
            s = item["stats"]
            q_ok = (reference_q is not None and q is not None
                    and len(q) == len(reference_q)
                    and max(abs(a - b) for a, b in zip(q, reference_q))
                    <= args.initial_q_tolerance_rad)
            pos_rms = (s["pos_error_m"]["rms"] or float("inf")) * 1e3
            pos_peak = (s["pos_error_m"]["max_abs"] or float("inf")) * 1e3
            summary = item["summary"]
            item["validity_gate"] = {
                "initial_q_ok": q_ok,
                "watchdog_ok": not bool(summary.get("watchdog", False)),
                "backend_clamp_ok": int(summary.get("backend_clamp_total", 0)) == 0,
                "position_rms_ok": pos_rms <= args.position_rms_max_mm,
                "position_peak_ok": pos_peak <= args.position_peak_max_mm,
            }
            item["validity_gate"]["valid"] = all(
                item["validity_gate"].values())
    (output / "leso_comparison.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=True), encoding="utf-8")
    keys = ["position_rms_mm", "position_peak_mm", "j5_velocity_rms_rad_s",
            "j5_velocity_peak_rad_s", "six_axis_torque_peak_nm",
            "z3_raw_rms_rad_s2", "z3_raw_p95_rad_s2", "z3_raw_peak_rad_s2",
            "z3_filtered_rms_rad_s2", "tau_dist_peak_nm",
            "torque_saturation_fraction", "watchdog_fraction"]
    with open(output / "leso_comparison.csv", "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["run", *keys])
        for label, (_, rows) in runs.items():
            s = report[label]["stats"]
            n = max(1, len(rows))
            writer.writerow([
                label,
                1e3 * (s["pos_error_m"]["rms"] or float("nan")),
                1e3 * (s["pos_error_m"]["max_abs"] or float("nan")),
                s["j5_qdot_motor_rad_s"]["rms"],
                s["j5_qdot_motor_rad_s"]["max_abs"],
                s["six_axis_torque_nm"]["max_abs"],
                s["j5_leso_z3_raw_rad_s2"]["rms"],
                s["j5_leso_z3_raw_rad_s2"]["p95_abs"],
                s["j5_leso_z3_raw_rad_s2"]["max_abs"],
                s["j5_leso_z3_filtered_rad_s2"]["rms"],
                s["j5_tau_dist_nm"]["max_abs"],
                float(np.mean(_values(rows, "j5_torque_saturation")))
                if _values(rows, "j5_torque_saturation").size else float("nan"),
                float(np.mean(_values(rows, "watchdog_flag")))
                if _values(rows, "watchdog_flag").size else float("nan"),
            ])

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("stats written; matplotlib unavailable, plots skipped")
        return
    labels = list(runs)
    pos = [1e3 * (report[x]["stats"]["pos_error_m"]["rms"] or np.nan)
           for x in labels]
    z3 = [report[x]["stats"]["j5_leso_z3_raw_rad_s2"]["rms"] or np.nan
          for x in labels]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].bar(labels, pos)
    axes[0].set_ylabel("position RMS (mm)")
    axes[1].bar(labels, z3)
    axes[1].set_ylabel("J5 z3 raw RMS (rad/s^2)")
    fig.tight_layout()
    fig.savefig(output / "leso_comparison_metrics.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=False)
    for label, (_, rows) in runs.items():
        t = _values(rows, "t")
        z = _values(rows, "j5_leso_z3_raw_rad_s2")
        if t.size == z.size and t.size:
            axes[0].plot(t, z, label=label, linewidth=0.7)
        e = _values(rows, "pos_error_m")
        if t.size == e.size and t.size:
            axes[1].plot(t, 1e3 * e, label=label, linewidth=0.7)
    axes[0].set_ylabel("J5 z3 raw (rad/s^2)")
    axes[1].set_ylabel("position error (mm)")
    axes[1].set_xlabel("time (s)")
    axes[0].legend()
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(output / "leso_comparison_timeseries.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
    for label, item in report.items():
        for row, key, style in ((0, "j5_leso_z3_raw_rad_s2", "-"),
                                (1, "j5_leso_z3_filtered_rad_s2", "-")):
            psd_path = output / f"{label}_{key}_psd.csv"
            if not psd_path.exists():
                continue
            data = np.loadtxt(psd_path, delimiter=",", skiprows=1)
            if data.ndim == 2 and data.shape[1] == 2:
                axes[row].semilogy(data[:, 0], data[:, 1], style,
                                   linewidth=0.8, label=label)
    axes[0].set_ylabel("raw z3 PSD")
    axes[1].set_ylabel("filtered z3 PSD")
    axes[1].set_xlabel("frequency (Hz)")
    for ax in axes:
        ax.set_xlim(0.1, 100.0)
        ax.grid(True, which="both", alpha=0.25)
        ax.legend()
    fig.tight_layout()
    fig.savefig(output / "leso_comparison_psd.png", dpi=160)
    plt.close(fig)

    # The LPF comparison is generated from the same raw z3 sequence, so it
    # cannot be mistaken for three different closed-loop robot experiments.
    for label, (_, rows) in runs.items():
        t, raw, _ = _uniform_raw(rows, "j5_leso_z3_raw_rad_s2")
        if t is None:
            continue
        fig, ax = plt.subplots(figsize=(11, 4))
        ax.plot(t, raw, label="raw", linewidth=0.6, alpha=0.55)
        for cutoff in (3, 5, 8):
            data = np.loadtxt(output / f"{label}_offline_lpf_{cutoff}hz.csv",
                              delimiter=",", skiprows=1)
            ax.plot(data[:, 0], data[:, 2], label=f"LPF {cutoff} Hz", linewidth=0.8)
        ax.set_xlabel("time (s)")
        ax.set_ylabel("J5 z3 (rad/s^2)")
        ax.grid(alpha=0.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output / f"{label}_offline_lpf_timeseries.png", dpi=160)
        plt.close(fig)
    print(output / "leso_comparison.csv")


if __name__ == "__main__":
    main()
