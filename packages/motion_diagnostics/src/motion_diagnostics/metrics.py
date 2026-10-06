"""Shaft-equivalent statistics, not carriage accuracy or ISO certification."""

import math
import statistics
from collections import Counter, defaultdict


def summarize(errors, positions=None):
    values = list(errors)
    n = len(values)
    result = {
        "n": n,
        "bias": None,
        "mae": None,
        "max_abs_error": None,
        "rmse": None,
        "sample_sd": None,
        "AP_1D_adaptation": None,
        "RP_1D_adaptation": None,
        "label": "ISO-style 1D analogue (not certified)",
    }
    if not n:
        return result
    mean = statistics.mean(values)
    result.update(
        bias=mean,
        mae=statistics.mean(map(abs, values)),
        max_abs_error=max(map(abs, values)),
        rmse=math.sqrt(statistics.mean(x * x for x in values)),
        AP_1D_adaptation=abs(mean),
    )
    if n >= 2:
        result["sample_sd"] = statistics.stdev(values)
        points = values if positions is None else list(positions)
        if len(points) != n:
            raise ValueError("positions/errors sample count differs")
        center = statistics.mean(points)
        radii = [abs(x - center) for x in points]
        result["RP_1D_adaptation"] = statistics.mean(radii) + 3 * statistics.stdev(
            radii
        )
    return result


def analyze(rows):
    groups = defaultdict(list)
    for row in rows:
        if str(row.get("record_type")) not in ("129", "0x81"):
            continue
        key = tuple(
            str(row.get(k, ""))
            for k in (
                "axis",
                "phase",
                "applied_target_01mm",
                "approach_direction",
                "config_id",
                "home_epoch",
                "session_id",
                "pose_id",
                "trial_approach_direction",
                "rx_x_01mm",
            )
        )
        groups[key].append(row)
    reports = []
    for key, population in sorted(groups.items()):
        eligible = [
            r
            for r in population
            if str(r.get("metrics_eligible")).lower() in ("true", "1")
            and str(r.get("scored", True)).lower() not in ("false", "0")
            and r.get("match_status", "matched") == "matched"
        ]
        excluded = Counter()
        for r in population:
            if r not in eligible:
                excluded[
                    r.get("exclusion_reason") or r.get("match_status") or "non_scored"
                ] += 1
        report = dict(
            zip(
                (
                    "axis",
                    "phase",
                    "target_01mm",
                    "approach_direction",
                    "config_id",
                    "home_epoch",
                    "session_id",
                    "pose_id",
                    "trial_approach_direction",
                    "rx_x_01mm",
                ),
                key,
            )
        )
        report.update(
            received=len(population),
            eligible=len(eligible),
            excluded=len(population) - len(eligible),
            exclusion_counts=dict(excluded),
        )
        for name, column in (
            ("endpoint", "endpoint_error_um"),
            ("total", "total_error_um"),
            ("tracking", "shaft_tracking_error_um"),
            ("angular_unwrapped", "shaft_error_deg"),
        ):
            vals = [
                float(r[column]) for r in eligible if r.get(column) not in ("", None)
            ]
            report[name] = summarize(vals)
            if name == "angular_unwrapped":
                sd = report[name]["sample_sd"]
                report[name]["three_sd_spread_deg"] = None if sd is None else 3 * sd
        reports.append(report)
    return reports
