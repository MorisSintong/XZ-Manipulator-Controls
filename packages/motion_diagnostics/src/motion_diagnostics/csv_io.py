"""Stable long-form CSV: one axis-phase row, with no zero-filled missing data."""

import csv
from dataclasses import asdict, fields
from pathlib import Path

from .frames import CommandResult, Header, HomeAxis, HomeResult, Phase, StatusPayload
from .status_bits import StatusBits, command_succeeded

DERIVED = [
    "commanded_um",
    "requested_um",
    "planned_step_um",
    "step_implied_um",
    "encoder_um",
    "clamp_error_um",
    "quant_command_error_um",
    "shaft_tracking_error_um",
    "total_error_um",
    "requested_total_error_um",
    "endpoint_um",
    "endpoint_error_um",
    "requested_endpoint_error_um",
    "step_implied_deg",
    "shaft_error_deg",
    "metrics_eligible",
    "exclusion_reason",
]
HOST = [
    "session_id",
    "run_id",
    "host_send_seq",
    "host_send_utc",
    "host_receive_utc",
    "match_status",
    "trial_id",
    "pose_id",
    "repeat_index",
    "approach_direction",
    "trial_approach_direction",
    "scored",
    "timestamp_ms_extended",
]
COLUMNS = list(
    dict.fromkeys(
        [f.name for f in fields(Header)]
        + HOST
        + [f.name for f in fields(Phase)]
        + [f"home_{f.name}" for f in fields(HomeAxis)]
        + [f.name for f in fields(StatusPayload)]
        + DERIVED
    )
)


def derive(phase: Phase, header: Header, config=None):
    config = config or {}
    lead = (
        config.get("x_lead_um", 40000)
        if phase.axis == 0
        else config.get("z_lead_um", 8000)
    )
    steps = config.get("steps_per_rev", 3200)
    sign = -1 if phase.axis_flags & (1 << 12) else 1
    reasons = []
    if config.get("config_id") != header.config_id or not config.get(
        "qualified", False
    ):
        reasons.append("config_unqualified")
    if phase.axis not in (0, 1) or (phase.axis, phase.phase) not in (
        (0, 1),
        (1, 2),
        (1, 3),
    ):
        reasons.append("phase_identity")
    if not command_succeeded(header.status_bits):
        reasons.append("global_flags")
    if header.status_bits & StatusBits.CLAMPED:
        reasons.append("clamped")
    if phase.axis_flags & ((1 << 8) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 14)):
        reasons.append("axis_fault_or_clamp")
    if phase.axis_flags & 0x3F != 0x3F:
        reasons.append("incomplete_or_invalid_reference")
    if phase.as_status_start & 0x38 != 0x20 or phase.as_status_end & 0x38 != 0x20:
        reasons.append("encoder_health_invalid")
    result = {"metrics_eligible": not reasons, "exclusion_reason": ";".join(reasons)}
    start = phase.start_position_steps * lead / steps
    c = phase.applied_target_01mm * 100 - start
    req = phase.requested_target_01mm * 100 - start
    s = phase.emitted_delta_steps * lead / steps
    result.update(
        commanded_um=c,
        requested_um=req,
        planned_step_um=phase.commanded_delta_steps * lead / steps,
        step_implied_um=s,
        clamp_error_um=c - req,
        quant_command_error_um=s - c,
        step_implied_deg=phase.emitted_delta_steps * 360 / steps,
    )
    if phase.axis_flags & 0x1C == 0x1C:
        delta = phase.unwrap_end - phase.unwrap_start
        e = sign * delta * lead / 4096
        result.update(
            encoder_um=e,
            shaft_tracking_error_um=e - s,
            total_error_um=e - c,
            requested_total_error_um=e - req,
            shaft_error_deg=sign * delta * 360 / 4096
            - phase.emitted_delta_steps * 360 / steps,
        )
        if phase.axis_flags & 0x20:
            endpoint = (
                sign * (phase.unwrap_end - phase.home_offset_counts) * lead / 4096
            )
            result.update(
                endpoint_um=endpoint,
                endpoint_error_um=endpoint - phase.applied_target_01mm * 100,
                requested_endpoint_error_um=endpoint
                - phase.requested_target_01mm * 100,
            )
    return result


def record_rows(record, context=None, config=None):
    base = {**asdict(record.header), **(context or {})}
    if isinstance(record, CommandResult):
        rows = []
        for p in record.phases:
            row = {**base, **asdict(p), **derive(p, record.header, config)}
            row["trial_approach_direction"] = base.get("approach_direction", "")
            if not row.get("approach_direction"):
                row["approach_direction"] = (
                    "positive"
                    if p.commanded_delta_steps > 0
                    else "negative"
                    if p.commanded_delta_steps < 0
                    else "stationary"
                )
            if p.axis == 1:
                row["approach_direction"] = "positive" if p.phase == 2 else "negative"
            rows.append(row)
        return rows
    if isinstance(record, HomeResult):
        return [
            {
                **base,
                "axis": p.axis,
                "phase": "home",
                **{f"home_{k}": v for k, v in asdict(p).items()},
                "metrics_eligible": False,
                "exclusion_reason": "home_separate_population",
            }
            for p in record.axes
        ]
    return [
        {
            **base,
            **asdict(record.status),
            "metrics_eligible": False,
            "exclusion_reason": "status",
        }
    ]


class CsvWriter:
    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, COLUMNS)
        self.writer.writeheader()

    def write(self, rows):
        self.writer.writerows(rows)
        self.file.flush()

    def close(self):
        self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))
