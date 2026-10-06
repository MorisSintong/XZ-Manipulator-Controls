import argparse
import csv
import json
import time
from pathlib import Path

from motion_diagnostics.commands import control_packet, encode_detection_packet
from motion_diagnostics.csv_io import read_csv
from motion_diagnostics.frames import CommandResult, HomeResult, Status
from motion_diagnostics.metrics import analyze
from motion_diagnostics.status_bits import (
    StatusBits,
    command_succeeded,
    has_fault,
    host_ready,
)

from .serial_io import SerialSession, open_serial


def parser():
    p = argparse.ArgumentParser(
        description="Shaft-equivalent diagnostics; software abort is not a safety-rated stop."
    )
    sub = p.add_subparsers(dest="action", required=True)
    for name in ("capture", "send-sequence", "heartbeat", "abort", "rehome"):
        cmd = sub.add_parser(name)
        cmd.add_argument("--port", required=True)
        cmd.add_argument("--csv", required=True, type=Path)
        cmd.add_argument("--config", required=True, type=Path)
        cmd.add_argument("--timeout", type=float, default=10)
        if name == "capture":
            cmd.add_argument("--duration", type=float, default=0, help="0 until Ctrl+C")
        if name == "send-sequence":
            cmd.add_argument("--targets-mm", required=True)
            cmd.add_argument("--approach-mm", required=True, type=float)
            cmd.add_argument(
                "--reverse-approach-mm",
                type=float,
                help="higher preposition for --direction both",
            )
            cmd.add_argument(
                "--direction",
                choices=("positive", "negative", "both"),
                default="positive",
            )
            cmd.add_argument("--repetitions", type=int, default=30)
            cmd.add_argument("--pacing", type=float, default=0.05)
            cmd.add_argument("--rehome-each-trial", action="store_true")
    cmd = sub.add_parser("analyze")
    cmd.add_argument("--csv", required=True, type=Path)
    cmd.add_argument(
        "--output", type=Path, help="report basename (default CSV basename + _report)"
    )
    return p


def write_report(args):
    report = analyze(read_csv(args.csv))
    output = args.output or args.csv.with_name(args.csv.stem + "_report")
    output.parent.mkdir(parents=True, exist_ok=True)
    health_path = args.csv.with_suffix(".health.json")
    quality = (
        json.loads(health_path.read_text(encoding="utf-8"))
        if health_path.exists()
        else {
            "missing": None,
            "note": "send manifest unavailable; missing completions cannot be inferred from CSV alone",
        }
    )
    output.with_suffix(".json").write_text(
        json.dumps({"groups": report, "transport_quality": quality}, indent=2),
        encoding="utf-8",
    )
    columns = [
        "axis",
        "phase",
        "target_01mm",
        "approach_direction",
        "pose_id",
        "trial_approach_direction",
        "metric",
        "units",
        "received",
        "eligible",
        "excluded",
        "n",
        "bias",
        "mae",
        "max_abs_error",
        "rmse",
        "sample_sd",
        "AP_1D_adaptation",
        "RP_1D_adaptation",
        "label",
        "exclusion_counts",
    ]
    with output.with_suffix(".csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, columns)
        writer.writeheader()
        print("Axis Phase Target(0.1mm) Direction n Bias(um) SD(um) RP-1D(um) Excluded")
        for group in report:
            for metric in ("endpoint", "total", "tracking", "angular_unwrapped"):
                row = {
                    **group,
                    **group[metric],
                    "metric": metric,
                    "units": "deg" if metric == "angular_unwrapped" else "um",
                    "exclusion_counts": json.dumps(
                        group["exclusion_counts"], sort_keys=True
                    ),
                }
                writer.writerow({k: row.get(k) for k in columns})
            row = group["endpoint"]
            print(
                f"axis={group['axis']} phase={group['phase']} target={group['target_01mm']} direction={group['approach_direction']} n={row['n']} bias_um={row['bias']} SD_um={row['sample_sd']} RP_1D_um={row['RP_1D_adaptation']} excluded={group['excluded']}"
            )
    return report


def require_ready(session):
    deadline = time.monotonic() + session.timeout
    while time.monotonic() < deadline:
        heartbeat = session.send(control_packet(2, 65534))
        record = session.wait(heartbeat)
        if isinstance(record, Status) and host_ready(
            record.status.homed_mask, record.header.status_bits
        ):
            return
        if isinstance(record, Status) and has_fault(record.header.status_bits):
            raise RuntimeError("firmware aborted/faulted; operator recovery required")
        time.sleep(0.1)
    raise TimeoutError("not homed/READY; no pick sent")


def sequence(args, session, config):
    targets = [float(t) for t in args.targets_mm.split(",")]
    if args.repetitions < 1 or not targets or args.pacing < 0:
        raise ValueError("positive repetitions/targets and nonnegative pacing required")
    directions = (
        ("positive", "negative") if args.direction == "both" else (args.direction,)
    )
    if not config.get("qualified") or config.get("config_id") != 1:
        raise ValueError("qualified matching config required for scored trials")
    limit = config.get("usable_x_mm", -1)
    approaches = {
        d: args.reverse_approach_mm
        if d == "negative" and args.direction == "both"
        else args.approach_mm
        for d in directions
    }
    if any(a is None for a in approaches.values()):
        raise ValueError("--direction both requires --reverse-approach-mm")
    if any(not 0 <= target <= limit for target in [*approaches.values(), *targets]):
        raise ValueError("target/approach outside qualified travel")
    for direction in directions:
        if any(
            (
                t <= approaches[direction]
                if direction == "positive"
                else t >= approaches[direction]
            )
            for t in targets
        ):
            raise ValueError(
                "fixed approach must lie on the selected side of every target"
            )
    safe, depth = config.get("safe_z_01mm", -1), config.get("pick_depth_01mm", -1)
    if not 0 <= safe <= depth:
        raise ValueError(
            "explicit qualified 0 <= safe_z_01mm <= pick_depth_01mm required"
        )
    require_ready(session)
    obj_id = 1

    def pick(target, **metadata):
        nonlocal obj_id
        if obj_id >= 65534:
            raise ValueError("trial ID space exhausted; start a new session")
        sent = session.send(
            encode_detection_packet(obj_id, 1, target, 0, 0, 0), **metadata
        )
        obj_id += 1
        result = session.wait(sent)
        if not isinstance(result, CommandResult) or not command_succeeded(
            result.header.status_bits
        ):
            raise RuntimeError("trial failed; stopped without automatic retry")
        time.sleep(args.pacing)

    for repetition in range(args.repetitions):
        for target in targets:
            for direction in directions:
                if args.rehome_each_trial:
                    result = session.wait(session.send(control_packet(4, 65533)))
                    if result.header.status_bits & StatusBits.REJECTED:
                        raise RuntimeError("rehome rejected")
                    if not isinstance(result, HomeResult) or not command_succeeded(
                        result.header.status_bits
                    ):
                        raise RuntimeError("rehome failed")
                    require_ready(session)
                pick(approaches[direction], scored=False, approach_direction=direction)
                pick(
                    target,
                    scored=True,
                    pose_id=str(target),
                    trial_id=str(obj_id),
                    repeat_index=repetition,
                    approach_direction=direction,
                )


def main(argv=None, serial_factory=open_serial):
    args = parser().parse_args(argv)
    try:
        if args.action == "analyze":
            write_report(args)
            return 0
        config = json.loads(args.config.read_text(encoding="utf-8"))
        if args.timeout <= 0:
            raise ValueError("timeout must be positive")
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with SerialSession(
            args.port, args.csv, config, args.timeout, serial_factory
        ) as session:
            metadata = {
                "config": config,
                "arguments": {k: str(v) for k, v in vars(args).items()},
                "session_id": session.correlator.session_id,
                "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            }
            args.csv.with_suffix(".metadata.json").write_text(
                json.dumps(metadata, indent=2), encoding="utf-8"
            )
            try:
                if args.action == "send-sequence":
                    sequence(args, session, config)
                elif args.action == "capture":
                    deadline = (
                        time.monotonic() + args.duration
                        if args.duration
                        else float("inf")
                    )
                    while time.monotonic() < deadline:
                        session.poll()
                else:
                    kind = {"heartbeat": 2, "abort": 3, "rehome": 4}[args.action]
                    result = session.wait(session.send(control_packet(kind, 1)))
                    print(result)
                    if args.action == "rehome":
                        if result.header.status_bits & StatusBits.REJECTED:
                            raise RuntimeError("rehome rejected")
                        if not isinstance(result, HomeResult) or not command_succeeded(
                            result.header.status_bits
                        ):
                            raise RuntimeError("rehome failed")
            finally:
                session.correlator.expire(float("inf"))
                args.csv.with_suffix(".health.json").write_text(
                    json.dumps(
                        {
                            **session.correlator.counts,
                            "decoder_errors": session.decoder.errors,
                            "record_gaps": session.decoder.record_gaps,
                            "sent_commands": session.correlator.counts["commands_sent"],
                        },
                        indent=2,
                    ),
                    encoding="utf-8",
                )
        return 0
    except KeyboardInterrupt:
        return 130
    except (ValueError, OSError, RuntimeError, TimeoutError) as exc:
        print(f"diagnostics failed: {exc}")
        return 1
