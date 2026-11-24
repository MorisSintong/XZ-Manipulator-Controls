"""Replayable analysis and Markdown/JSON/CSV report generator for motion_characterization."""

from __future__ import annotations

import csv
import json
import math
import statistics
from collections import defaultdict
from itertools import pairwise
from pathlib import Path
from typing import Any

from motion_diagnostics.metrics import summarize

COUNTS_PER_REV = 4096.0
USTEPS_PER_REV = 3200.0
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV

AXIS_UNITS = {
    "x": {"mm_per_rev": 40.0, "steps_per_unit": 80.0, "unit": "mm"},
    "z": {"mm_per_rev": 8.0, "steps_per_unit": 400.0, "unit": "mm"},
    "theta": {"mm_per_rev": None, "steps_per_unit": USTEPS_PER_REV / 360.0, "unit": "deg"},
}

CAMPAIGN_SPEED_TOL_FRAC = 0.05
CAMPAIGN_ACCEL_TOL_FRAC = 0.10


def unwrap_counts(raw_codes: list[int]) -> list[int]:
    """Unwrap 12-bit AS5600 raw codes [0, 4095] relative to the first sample."""
    if not raw_codes:
        return []
    out = [0]
    prev = raw_codes[0] & 0xFFF
    acc = 0
    for code in raw_codes[1:]:
        cur = code & 0xFFF
        diff = cur - prev
        if diff > 2048:
            diff -= 4096
        elif diff <= -2048:
            diff += 4096
        acc += diff
        out.append(acc)
        prev = cur
    return out


def shortest_angle_error_deg(measured_deg: float, reference_deg: float) -> float:
    """Signed shortest-angle error (reference - measured) wrapped to [-180, 180)."""
    diff = (reference_deg - measured_deg + 180.0) % 360.0 - 180.0
    return diff


def extend_timestamps_us(raw_t_us: list[int]) -> list[int]:
    """Extend 32-bit unsigned microsecond timestamps across wrap."""
    if not raw_t_us:
        return []
    out: list[int] = []
    wrap_acc = 0
    prev = raw_t_us[0] & 0xFFFFFFFF
    for t in raw_t_us:
        cur = t & 0xFFFFFFFF
        if cur < prev and (prev - cur) > 0x80000000:
            wrap_acc += 0x100000000
        out.append(wrap_acc + cur)
        prev = cur
    return out


def residual_usteps(delta_counts: int, emitted_edges: int, encoder_sign: int = 1) -> float:
    """Sign-normalized shaft residual in microsteps: delta_counts*3200/4096 - emitted_edges."""
    return (delta_counts * encoder_sign) * (USTEPS_PER_REV / COUNTS_PER_REV) - emitted_edges


def linear_fit_slope(t_s: list[float], y: list[float]) -> tuple[float | None, float | None]:
    """Ordinary least squares slope (velocity) and RMS residual."""
    n = len(t_s)
    if n < 5 or len(set(t_s)) < 5:
        return None, None
    t_mean = statistics.mean(t_s)
    y_mean = statistics.mean(y)
    sxx = sum((t - t_mean) ** 2 for t in t_s)
    if sxx <= 0.0:
        return None, None
    sxy = sum((t - t_mean) * (yi - y_mean) for t, yi in zip(t_s, y, strict=True))
    slope = sxy / sxx
    intercept = y_mean - slope * t_mean
    rms = math.sqrt(
        sum((yi - (slope * t + intercept)) ** 2 for t, yi in zip(t_s, y, strict=True)) / n
    )
    return slope, rms


def quadratic_fit_accel(t_s: list[float], y: list[float]) -> tuple[float | None, float | None]:
    """Fit y = c0 + c1*t + c2*t^2 and return acceleration = 2*c2 and RMS residual."""
    n = len(t_s)
    if n < 5 or len(set(t_s)) < 5:
        return None, None
    t0 = t_s[0]
    ts = [t - t0 for t in t_s]
    s0 = float(n)
    s1 = sum(ts)
    s2 = sum(t * t for t in ts)
    s3 = sum(t ** 3 for t in ts)
    s4 = sum(t ** 4 for t in ts)
    sy = sum(y)
    s1y = sum(t * yi for t, yi in zip(ts, y, strict=True))
    s2y = sum(t * t * yi for t, yi in zip(ts, y, strict=True))

    # Solve 3x3 normal equations via Gaussian elimination
    mat = [
        [s0, s1, s2, sy],
        [s1, s2, s3, s1y],
        [s2, s3, s4, s2y],
    ]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(mat[r][col]))
        if abs(mat[pivot][col]) < 1e-18:
            return None, None
        mat[col], mat[pivot] = mat[pivot], mat[col]
        piv_val = mat[col][col]
        for j in range(col, 4):
            mat[col][j] /= piv_val
        for r in range(3):
            if r != col:
                factor = mat[r][col]
                for j in range(col, 4):
                    mat[r][j] -= factor * mat[col][j]
    c0, c1, c2 = mat[0][3], mat[1][3], mat[2][3]
    rms = math.sqrt(
        sum((yi - (c0 + c1 * t + c2 * t * t)) ** 2 for t, yi in zip(ts, y, strict=True)) / n
    )
    return 2.0 * c2, rms


def _load_events(events_path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    if not events_path.exists():
        return events
    for line in events_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def _check_seq_integrity(events: list[dict[str, Any]]) -> dict[str, Any]:
    device_seqs = [
        int(ev["seq"])
        for ev in events
        if isinstance(ev.get("seq"), int) and ev.get("source", "device") == "device"
    ]
    missing_gaps = 0
    for prev, cur in pairwise(device_seqs):
        if cur > prev + 1:
            missing_gaps += cur - prev - 1
    return {
        "device_records": len(device_seqs),
        "missing_seq_gaps": missing_gaps,
        "integrity_ok": missing_gaps == 0,
    }


def _analyze_encoder_rate(events: list[dict[str, Any]]) -> dict[str, Any]:
    windows: dict[tuple[str, str, bool], list[dict[str, Any]]] = defaultdict(list)
    for ev in events:
        if ev.get("kind") == "sample" and ev.get("test") == "encoder_rate":
            key = (
                str(ev.get("axis", "none")),
                str(ev.get("mode", "raw")),
                bool(ev.get("fast", False)),
            )
            windows[key].append(ev)

    summary: dict[str, Any] = {}
    for (axis, mode, fast), rows in sorted(windows.items()):
        rates: list[float] = []
        clean_windows = 0
        total_attempts = 0
        total_ok = 0
        max_lat = 0
        max_gap = 0
        for row in rows:
            attempts = int(row.get("attempts", 0))
            ok = int(row.get("ok", 0))
            elapsed_us = int(row.get("elapsed_us", 0))
            total_attempts += attempts
            total_ok += ok
            max_lat = max(max_lat, int(row.get("max_lat_us", 0)))
            max_gap = max(max_gap, int(row.get("max_gap_us", 0)))
            if attempts > 0 and ok == attempts and elapsed_us > 0:
                clean_windows += 1
                rates.append(ok / (elapsed_us / 1_000_000.0))
        repeatable = min(rates) if clean_windows >= 3 else None
        key_name = f"{axis}_{mode}_{'fast' if fast else 'installed'}"
        summary[key_name] = {
            "axis": axis,
            "mode": mode,
            "fast_mode": fast,
            "windows": len(rows),
            "clean_windows": clean_windows,
            "total_attempts": total_attempts,
            "total_ok": total_ok,
            "error_fraction": (
                (total_attempts - total_ok) / total_attempts if total_attempts else None
            ),
            "window_rates_hz": rates,
            "highest_repeatable_rate_hz": repeatable,
            "max_lat_us": max_lat,
            "max_gap_us": max_gap,
        }
    return summary


def _analyze_static_precision(events: list[dict[str, Any]]) -> dict[str, Any]:
    by_trial: dict[int, list[dict[str, Any]]] = defaultdict(list)
    axis_for_trial: dict[int, str] = {}
    for ev in events:
        if ev.get("kind") == "sample" and ev.get("test") == "static":
            tid = int(ev.get("trial_id", 0))
            by_trial[tid].append(ev)
            axis_for_trial[tid] = str(ev.get("axis", "none"))

    out: dict[str, Any] = {}
    for tid, rows in sorted(by_trial.items()):
        raws = [int(r["raw"]) for r in rows if "raw" in r]
        if not raws:
            continue
        unwrapped = unwrap_counts(raws)
        p2p_cnt = max(unwrapped) - min(unwrapped)
        sd_cnt = statistics.stdev(unwrapped) if len(unwrapped) >= 2 else 0.0
        mean_cnt = statistics.mean(unwrapped)
        noise_band_cnt = max(2.0, 3.0 * sd_cnt)
        axis = axis_for_trial[tid]
        out.setdefault(axis, []).append(
            {
                "trial_id": tid,
                "n": len(unwrapped),
                "mean_rel_cnt": mean_cnt,
                "sd_cnt": sd_cnt,
                "p2p_cnt": p2p_cnt,
                "sd_deg": sd_cnt * DEG_PER_COUNT,
                "p2p_deg": p2p_cnt * DEG_PER_COUNT,
                "commissioning_noise_band_cnt": noise_band_cnt,
            }
        )
    return out


def _analyze_reference_accuracy(events: list[dict[str, Any]]) -> dict[str, Any]:
    ref_points = [
        ev for ev in events if ev.get("kind") == "reference_point" and "reference_deg" in ev
    ]
    if not ref_points:
        return {
            "absolute_accuracy_deg": None,
            "reason": "no_independent_reference",
            "n_points": 0,
        }

    errors: list[float] = []
    for pt in ref_points:
        ref_deg = float(pt["reference_deg"])
        meas_deg = float(pt["measured_deg"])
        errors.append(shortest_angle_error_deg(meas_deg, ref_deg))

    stats = summarize(errors)
    return {
        "absolute_accuracy_deg": stats["max_abs_error"],
        "bias_deg": stats["bias"],
        "mae_deg": stats["mae"],
        "rmse_deg": stats["rmse"],
        "sample_sd_deg": stats["sample_sd"],
        "n_points": stats["n"],
        "reason": "ok",
    }


def _analyze_spi_rate(events: list[dict[str, Any]]) -> dict[str, Any]:
    by_key: dict[tuple[str, int, int], list[dict[str, Any]]] = defaultdict(list)
    for ev in events:
        if ev.get("kind") == "sample" and ev.get("test") == "spi_rate":
            if ev.get("skipped"):
                continue
            key = (
                str(ev.get("axis", "none")),
                int(ev.get("div", 0)),
                int(ev.get("sck_hz", 0)),
            )
            by_key[key].append(ev)

    rows_out: list[dict[str, Any]] = []
    fastest_clean_sck_hz: dict[str, int | None] = {}
    for (axis, div, sck_hz), wins in sorted(by_key.items(), key=lambda item: (item[0][0], -item[0][1])):
        clean = 0
        rates: list[float] = []
        total_err = 0
        for w in wins:
            att = int(w.get("attempts", 0))
            ok = int(w.get("ok", 0))
            mis = int(w.get("mismatches", 0))
            err = int(w.get("errors", 0))
            el_us = int(w.get("elapsed_us", 0))
            total_err += mis + err
            if att > 0 and ok == att and mis == 0 and err == 0 and el_us > 0:
                clean += 1
                rates.append(ok / (el_us / 1_000_000.0))
        repeatable = min(rates) if clean >= 3 else None
        if repeatable is not None:
            cur_best = fastest_clean_sck_hz.get(axis)
            if cur_best is None or sck_hz > cur_best:
                fastest_clean_sck_hz[axis] = sck_hz
        rows_out.append(
            {
                "axis": axis,
                "prescaler": div,
                "sck_hz": sck_hz,
                "clean_windows": clean,
                "total_errors": total_err,
                "repeatable_reads_s": repeatable,
                "repeatable_frames_s": repeatable * 2.0 if repeatable is not None else None,
            }
        )
    return {"windows": rows_out, "fastest_clean_sck_hz": fastest_clean_sck_hz}


def _analyze_motion_trials(
    events: list[dict[str, Any]],
    manifest: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    samples_by_trial: dict[int, list[dict[str, Any]]] = defaultdict(list)
    fault_by_trial: dict[int, str] = {}
    results: list[dict[str, Any]] = []

    for ev in events:
        tid = int(ev.get("trial_id", 0))
        kind = ev.get("kind")
        if kind == "sample" and "counts" in ev:
            samples_by_trial[tid].append(ev)
        elif kind == "event" and ev.get("event") == "fault_latched":
            fault_by_trial[tid] = str(ev.get("reason", "fault"))
        elif kind == "result" and ev.get("test") in ("jog", "move", "stress"):
            results.append(ev)

    session_mode = manifest.get("session_mode", "xz")
    trials_rows: list[dict[str, Any]] = []
    by_axis: dict[str, list[dict[str, Any]]] = defaultdict(list)

    for res in results:
        tid = int(res.get("trial_id", 0))
        axis = str(res.get("axis", "none"))
        test = str(res.get("test", "move"))
        emitted = int(res.get("emitted_edges", 0))
        delta_cnt = int(res.get("delta_counts", 0))
        enc_sign = int(res.get("encoder_sign", 1))
        res_usteps = residual_usteps(delta_cnt, emitted, enc_sign)
        req_speed = res.get("requested_speed_steps_s")
        req_accel = res.get("requested_accel_steps_s2")
        plateau_ok = bool(res.get("plateau_achieved", True))
        late_fault = bool(res.get("lateness_fault", False)) or (
            int(res.get("max_lateness_us", 0)) > 20
        )
        seq_missing = bool(res.get("missing_seq", False))
        fault_reason = fault_by_trial.get(tid) or res.get("reason")
        ok = bool(res.get("ok", False)) and not late_fault and not seq_missing and not fault_reason
        if test == "move" and not plateau_ok:
            ok = False
            fault_reason = fault_reason or "plateau_not_achieved"

        # Fit cruise velocity from timestamped encoder samples if available
        meas_speed_steps_s: float | None = None
        trial_samples = samples_by_trial.get(tid, [])
        if len(trial_samples) >= 5:
            raw_t = [int(s.get("t1_us", s.get("t_us", 0))) for s in trial_samples]
            ext_t = [t / 1_000_000.0 for t in extend_timestamps_us(raw_t)]
            usteps_series = [
                int(s["counts"]) * enc_sign * (USTEPS_PER_REV / COUNTS_PER_REV)
                for s in trial_samples
            ]
            slope, _ = linear_fit_slope(ext_t, usteps_series)
            if slope is not None:
                meas_speed_steps_s = abs(slope)

        row = {
            "trial_id": tid,
            "axis": axis,
            "test": test,
            "ok": ok,
            "valid_for_bracket": (not late_fault) and (not seq_missing) and plateau_ok,
            "emitted_edges": emitted,
            "delta_counts": delta_cnt,
            "residual_usteps": round(res_usteps, 3),
            "residual_fullsteps_equiv": round(res_usteps / 16.0, 4),
            "requested_speed_steps_s": req_speed,
            "requested_accel_steps_s2": req_accel,
            "measured_speed_steps_s": round(meas_speed_steps_s, 2) if meas_speed_steps_s else None,
            "max_lateness_us": int(res.get("max_lateness_us", 0)),
            "fault_reason": fault_reason,
        }
        trials_rows.append(row)
        by_axis[axis].append(row)

    axis_summary: dict[str, Any] = {}
    for axis in ("x", "z", "theta"):
        if axis == "theta" and session_mode == "xz":
            axis_summary[axis] = {
                "mechanical_status": "not_measured",
                "reason": "no_shaft_feedback",
                "last_pass_speed_steps_s": None,
                "first_fail_speed_steps_s": None,
                "speed_limit_classification": "not_measured",
                "stress_status": "not_measured",
            }
            continue
        rows = by_axis.get(axis, [])
        move_rows = [r for r in rows if r["test"] == "move"]
        passing = [
            r for r in move_rows
            if r["ok"] and r["valid_for_bracket"] and r["requested_speed_steps_s"] is not None
        ]
        failing = [
            r for r in move_rows
            if (not r["ok"]) and r["valid_for_bracket"] and r["requested_speed_steps_s"] is not None
        ]
        invalid = [r for r in move_rows if not r["valid_for_bracket"]]

        last_pass = max((r["requested_speed_steps_s"] for r in passing), default=None)
        first_fail = None
        first_fail_reason = None
        if last_pass is not None:
            higher_fails = [
                r for r in failing if r["requested_speed_steps_s"] > last_pass
            ]
            if higher_fails:
                chosen = min(higher_fails, key=lambda r: r["requested_speed_steps_s"])
                first_fail = chosen["requested_speed_steps_s"]
                first_fail_reason = chosen["fault_reason"]

        if last_pass is not None and first_fail is not None:
            classification = "bracketed"
        elif last_pass is not None:
            classification = "cap_reached"
        elif invalid:
            classification = "invalid"
        else:
            classification = "not_measured"

        axis_summary[axis] = {
            "mechanical_status": "measured" if rows else "not_measured",
            "trials": len(rows),
            "passes": sum(1 for r in rows if r["ok"]),
            "fails": sum(1 for r in rows if not r["ok"]),
            "invalid_trials": len(invalid),
            "last_pass_speed_steps_s": last_pass,
            "first_fail_speed_steps_s": first_fail,
            "first_fail_reason": first_fail_reason,
            "speed_limit_classification": classification,
        }

    return trials_rows, axis_summary


def generate_report(run_dir: Path) -> dict[str, Any]:
    """Read manifest.json + events.jsonl in run_dir and write trials.csv, summary.json, report.md."""
    manifest_path = run_dir / "manifest.json"
    manifest = (
        json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest_path.exists()
        else {}
    )
    events = _load_events(run_dir / "events.jsonl")
    integrity = _check_seq_integrity(events)
    enc_rate = _analyze_encoder_rate(events)
    static_prec = _analyze_static_precision(events)
    ref_acc = _analyze_reference_accuracy(events)
    spi_rate = _analyze_spi_rate(events)
    trials_rows, axis_motion = _analyze_motion_trials(events, manifest)
    caveats: list[str] = [
        "MCU STEP timestamps are emitted command intervals, not oscilloscope pin measurements.",
        "Shaft residuals are shaft step-loss equivalents, not carriage accuracy or exact skipped-step counts below the detection floor.",
        "Absolute accuracy is null unless an independent mechanical reference instrument was captured.",
    ]
    summary: dict[str, Any] = {
        "schema_version": 1,
        "session_mode": manifest.get("session_mode", "xz"),
        "capture_id": manifest.get("capture_id"),
        "stream_integrity": integrity,
        "sensor_resolution_deg_per_count": DEG_PER_COUNT,
        "encoder_rate": enc_rate,
        "static_precision": static_prec,
        "reference_accuracy": ref_acc,
        "spi_rate": spi_rate,
        "motion": axis_motion,
        "campaign_tolerances": {
            "cruise_speed_frac": CAMPAIGN_SPEED_TOL_FRAC,
            "accel_frac": CAMPAIGN_ACCEL_TOL_FRAC,
        },
        "caveats": caveats,
    }

    # Write trials.csv
    csv_cols = [
        "trial_id",
        "axis",
        "test",
        "ok",
        "valid_for_bracket",
        "emitted_edges",
        "delta_counts",
        "residual_usteps",
        "residual_fullsteps_equiv",
        "requested_speed_steps_s",
        "requested_accel_steps_s2",
        "measured_speed_steps_s",
        "max_lateness_us",
        "fault_reason",
    ]
    with (run_dir / "trials.csv").open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=csv_cols)
        writer.writeheader()
        for row in trials_rows:
            writer.writerow(row)

    # Write summary.json
    (run_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    # Write report.md from the exact same derived summary
    lines = [
        "# Motion Characterization Report",
        "",
        f"- **Capture ID:** `{summary['capture_id']}`",
        f"- **Session Mode:** `{summary['session_mode']}`",
        f"- **Device Records:** {integrity['device_records']} (missing seq gaps: {integrity['missing_seq_gaps']})",
        f"- **AS5600 Resolution:** `{DEG_PER_COUNT:.6f}` deg/count (360/4096)",
        "",
        "## Absolute Angle Accuracy",
        "",
        f"- `absolute_accuracy_deg`: `{ref_acc['absolute_accuracy_deg']}` (reason: `{ref_acc['reason']}`)",
        "",
        "## Encoder Read Throughput",
        "",
        "| Mode | Clean Windows | Repeatable Rate (reads/s) | Error Fraction | Max Latency (us) |",
        "|------|---------------|---------------------------|----------------|------------------|",
    ]
    for key, item in sorted(enc_rate.items()):
        rate_str = (
            f"{item['highest_repeatable_rate_hz']:.1f}"
            if item["highest_repeatable_rate_hz"] is not None
            else "null"
        )
        err_str = (
            f"{item['error_fraction']:.5f}" if item["error_fraction"] is not None else "null"
        )
        lines.append(
            f"| `{key}` | {item['clean_windows']}/{item['windows']} | {rate_str} | {err_str} | {item['max_lat_us']} |"
        )

    lines.extend(
        [
            "",
            "## TMC2240 SPI Electrical Throughput",
            "",
            "| Target | Prescaler | SCK (Hz) | Clean Windows | Repeatable Reads/s | Repeatable Frames/s |",
            "|--------|-----------|----------|---------------|--------------------|---------------------|",
        ]
    )
    for row in spi_rate["windows"]:
        r_str = (
            f"{row['repeatable_reads_s']:.1f}" if row["repeatable_reads_s"] is not None else "null"
        )
        f_str = (
            f"{row['repeatable_frames_s']:.1f}" if row["repeatable_frames_s"] is not None else "null"
        )
        lines.append(
            f"| `{row['axis']}` | /{row['prescaler']} | {row['sck_hz']} | {row['clean_windows']} | {r_str} | {f_str} |"
        )

    lines.extend(
        [
            "",
            "## Motion Limits & Bracketing",
            "",
            "| Axis | Status | Classification | Last Pass (steps/s) | First Fail (steps/s) | First Fail Reason |",
            "|------|--------|----------------|---------------------|----------------------|-------------------|",
        ]
    )
    for ax in ("x", "z", "theta"):
        m = axis_motion[ax]
        lines.append(
            f"| `{ax}` | `{m['mechanical_status']}` | `{m['speed_limit_classification']}` | "
            f"`{m['last_pass_speed_steps_s']}` | `{m['first_fail_speed_steps_s']}` | `{m.get('first_fail_reason') or m.get('reason')}` |"
        )

    lines.extend(["", "## Caveats", ""])
    for caveat in caveats:
        lines.append(f"- {caveat}")

    (run_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary
